#!/usr/bin/env python3
"""Pure protocol checks; no ROS master, devices, or network required."""

import copy
import json
import os
from pathlib import Path
import tempfile
import unittest

from ag_uav_client.protocol import (ProtocolError, UINT64_MAX, build_request,
                                load_request, load_target, loads_json, point,
                                save_request, validate_request, validate_state)


def state(**changes):
    result = {"protocol_version": 1, "server_id": "test-boot", "last_sequence": None,
              "updated_stamp": 100.0, "task_id": None, "phase": "IDLE",
              "last_command": None}
    result.update(changes)
    return result


class ProtocolTests(unittest.TestCase):
    def test_point_optional_fields_are_null(self):
        self.assertEqual(point(0, 180), {"latitude": 0.0, "longitude": 180.0,
                                       "altitude_ellipsoid_m": None, "yaw": None})

    def test_point_rejects_bad_numbers_and_ranges(self):
        for args in ((True, 0), (0, False), (91, 0), (0, -181), (float("nan"), 0),
                     (0, float("inf")), ("1", 0), (None, 0), (10 ** 1000, 0),
                     (0, 0, True), (0, 0, None, False), (0, 0, float("inf"))):
            with self.subTest(args=str(args)[:100]), self.assertRaises(ProtocolError):
                point(*args)

    def test_strict_json(self):
        for source in ('{"a": 1, "a": 2}', '{"nested":{"a":1,"a":2}}',
                       '{"a":NaN}', '{"a":Infinity}', '{"a":-Infinity}',
                       '{"a":1e999}', '{"a":', 'null null'):
            with self.subTest(source=source), self.assertRaises(ProtocolError):
                loads_json(source)
        self.assertEqual(loads_json('{"a":1.2,"b":null}'), {"a": 1.2, "b": None})

    def test_state_is_copied_and_extra_evidence_preserved(self):
        original = state(extra={"nested": [1]})
        result = validate_state(original)
        result["extra"]["nested"].append(2)
        self.assertEqual(original["extra"], {"nested": [1]})

    def test_state_rejects_missing_or_invalid_safety_fields(self):
        for key, bad in (("protocol_version", True), ("protocol_version", 2),
                         ("server_id", " "), ("last_sequence", True),
                         ("last_sequence", 0), ("last_sequence", UINT64_MAX + 1),
                         ("updated_stamp", 0), ("updated_stamp", float("nan")),
                         ("task_id", ""), ("phase", ""), ("last_command", [])):
            with self.subTest(key=key, bad=bad), self.assertRaises(ProtocolError):
                validate_state(state(**{key: bad}))
        for key in state():
            bad = state()
            del bad[key]
            with self.subTest(missing=key), self.assertRaises(ProtocolError):
                validate_state(bad)

    def test_new_set_allocates_task_and_request_from_watermark(self):
        original = state(last_sequence=32, task_id="old-task", phase="EXECUTING")
        goal = point(1, 2, 3, 0)
        vias = [point(2, 3)]
        first = build_request(original, "SET_TASK", goal, vias)
        second = build_request(original, "SET_TASK", goal, vias)
        self.assertEqual(first["sequence"], 33)
        self.assertEqual(first["server_id"], "test-boot")
        self.assertNotEqual(first["task_id"], "old-task")
        self.assertNotEqual(first["request_id"], second["request_id"])
        self.assertNotEqual(first["task_id"], second["task_id"])
        first["via_points"][0]["latitude"] = 88
        self.assertEqual(vias[0]["latitude"], 2)
        self.assertEqual(original["last_sequence"], 32)

    def test_initial_sequence_is_one(self):
        self.assertEqual(build_request(state(), "SET_TASK", point(0, 0))["sequence"], 1)

    def test_uint64_exhaustion_fails(self):
        with self.assertRaises(ProtocolError):
            build_request(state(last_sequence=UINT64_MAX), "SET_TASK", point(0, 0))

    def test_task_operations_preserve_task_and_empty_payload(self):
        current = state(last_sequence=2, task_id="task", phase="PAUSED")
        for operation in ("REPLACE_VIA_POINTS", "PAUSE", "RESUME", "CANCEL"):
            request = build_request(current, operation)
            self.assertEqual(request["task_id"], "task")
            self.assertEqual(request["sequence"], 3)
            self.assertIsNone(request["final_goal"])
            self.assertEqual(request["via_points"], [])

    def test_non_set_requires_unfinished_task(self):
        for phase in ("IDLE", "CANCELED", "COMPLETED", "UNKNOWN"):
            with self.subTest(phase=phase), self.assertRaises(ProtocolError):
                build_request(state(task_id="old", phase=phase), "RESUME")
        with self.assertRaises(ProtocolError):
            build_request(state(phase="READY"), "PAUSE")

    def test_whole_route_validation(self):
        with self.assertRaises(ProtocolError):
            build_request(state(), "SET_TASK", point(0, 0), [point(1, 1, yaw=0)])
        for operation in ("PAUSE", "RESUME", "CANCEL"):
            with self.subTest(operation=operation), self.assertRaises(ProtocolError):
                build_request(state(task_id="task", phase="EXECUTING"), operation,
                              via_points=[point(0, 0)])
        with self.assertRaises(ProtocolError):
            build_request(state(task_id="task", phase="EXECUTING"), "PAUSE", point(0, 0))

    def test_invalid_operation_and_missing_goal(self):
        for operation in ("set", "", [], None):
            with self.subTest(operation=operation), self.assertRaises(ProtocolError):
                build_request(state(), operation, point(0, 0))
        with self.assertRaises(ProtocolError):
            build_request(state(), "SET_TASK")

    def test_validate_request_rejects_changed_shape_and_types(self):
        valid = build_request(state(), "SET_TASK", point(0, 0))
        for key, bad in (("protocol_version", True), ("protocol_version", 2),
                         ("sequence", True), ("sequence", 0),
                         ("sequence", UINT64_MAX + 1), ("server_id", ""),
                         ("request_id", ""), ("task_id", ""),
                         ("operation", "NOPE"), ("via_points", ()),
                         ("final_goal", None)):
            request = copy.deepcopy(valid)
            request[key] = bad
            with self.subTest(key=key), self.assertRaises(ProtocolError):
                validate_request(request)
        for change in ("extra", "missing", "point_extra", "point_missing"):
            request = copy.deepcopy(valid)
            if change == "extra":
                request["extra"] = 1
            elif change == "missing":
                del request["operation"]
            elif change == "point_extra":
                request["final_goal"]["has_yaw"] = False
            else:
                del request["final_goal"]["latitude"]
            with self.subTest(change=change), self.assertRaises(ProtocolError):
                validate_request(request)

    def test_save_load_exact_request_and_refuse_overwrite(self):
        request = build_request(state(), "SET_TASK", point(1, 2, yaw=0))
        with tempfile.TemporaryDirectory() as temp:
            path = Path(temp) / "request.json"
            save_request(request, path)
            self.assertEqual(load_request(path), request)
            self.assertEqual(path.stat().st_mode & 0o777, 0o600)
            with self.assertRaises(FileExistsError):
                save_request(request, path)
            self.assertEqual(load_request(path), request)

    def test_save_refuses_symlink(self):
        request = build_request(state(), "SET_TASK", point(1, 2))
        with tempfile.TemporaryDirectory() as temp:
            path = Path(temp) / "request.json"
            path.symlink_to(Path(temp) / "other")
            with self.assertRaises(FileExistsError):
                save_request(request, path)
            self.assertFalse((Path(temp) / "other").exists())

    def test_invalid_request_does_not_create_file(self):
        with tempfile.TemporaryDirectory() as temp:
            path = Path(temp) / "request.json"
            with self.assertRaises(ProtocolError):
                save_request({}, path)
            self.assertFalse(path.exists())

    def test_capture_layouts_and_explicit_yaw(self):
        metadata = {"kind": "slicer_geographic_target", "version": 1,
                    "datum": "WGS84", "physical_reference": "base_link_horizontal"}
        coordinates = {"latitude": 1, "longitude": 2, "altitude_ellipsoid_m": 3}
        with tempfile.TemporaryDirectory() as temp:
            path = Path(temp) / "capture.json"
            for layout in ({**metadata, **coordinates}, {**metadata, "point": coordinates}):
                path.write_text(json.dumps(layout), encoding="utf-8")
                self.assertEqual(load_target(path), point(1, 2, 3))
                self.assertEqual(load_target(path, yaw=0.5), point(1, 2, 3, 0.5))
                self.assertEqual(load_target(path, via=True), point(1, 2, 3))
                with self.assertRaises(ProtocolError):
                    load_target(path, yaw=0.5, via=True)

    def test_capture_rejects_wrong_provenance_and_invalid_coordinates(self):
        valid = {"kind": "slicer_geographic_target", "version": 1, "datum": "WGS84",
                 "physical_reference": "base_link_horizontal", "latitude": 1,
                 "longitude": 2, "altitude_ellipsoid_m": 3}
        with tempfile.TemporaryDirectory() as temp:
            path = Path(temp) / "capture.json"
            for key, bad in (("kind", "point"), ("version", True), ("version", 2),
                             ("datum", "GCJ02"), ("physical_reference", "antenna"),
                             ("latitude", 91), ("longitude", "2"),
                             ("altitude_ellipsoid_m", None), ("yaw", 0),
                             ("point", {"latitude": 1})):
                path.write_text(json.dumps({**valid, key: bad}), encoding="utf-8")
                with self.subTest(key=key), self.assertRaises(ProtocolError):
                    load_target(path)
            for key in valid:
                bad = dict(valid)
                del bad[key]
                path.write_text(json.dumps(bad), encoding="utf-8")
                with self.subTest(missing=key), self.assertRaises(ProtocolError):
                    load_target(path)


if __name__ == "__main__":
    unittest.main()
