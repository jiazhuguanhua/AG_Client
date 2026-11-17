"""ROS-independent validation and request persistence for air/ground V1.

Saved requests contain exactly the Command fields, with JSON points using null
for unspecified altitude/yaw and null final_goal on non-SET operations.
"""

import copy
import json
import math
import os
import uuid


UINT64_MAX = (1 << 64) - 1
OPERATIONS = frozenset(("SET_TASK", "REPLACE_VIA_POINTS", "PAUSE", "RESUME", "CANCEL"))
UNFINISHED_PHASES = frozenset(("READY", "EXECUTING", "PAUSING", "PAUSED", "BLOCKED", "CANCELING"))
REQUEST_FIELDS = frozenset(("protocol_version", "server_id", "sequence", "request_id",
                            "task_id", "operation", "final_goal", "via_points"))
POINT_FIELDS = frozenset(("latitude", "longitude", "altitude_ellipsoid_m", "yaw"))


class ProtocolError(ValueError):
    """Input does not satisfy the V1 client contract."""


def _object(value, name):
    if not isinstance(value, dict):
        raise ProtocolError("{} must be an object".format(name))
    return value


def _string(value, name):
    if not isinstance(value, str) or not value.strip():
        raise ProtocolError("{} must be a nonempty string".format(name))
    return value


def _integer(value, name, minimum=1, maximum=UINT64_MAX):
    if type(value) is not int or not minimum <= value <= maximum:
        raise ProtocolError("{} must be an integer in [{}, {}]".format(name, minimum, maximum))
    return value


def _number(value, name):
    if type(value) not in (int, float):
        raise ProtocolError("{} must be a finite number".format(name))
    try:
        result = float(value)
    except (OverflowError, ValueError):
        raise ProtocolError("{} must be a finite number".format(name)) from None
    if not math.isfinite(result):
        raise ProtocolError("{} must be a finite number".format(name))
    return result


def _pairs(pairs):
    result = {}
    for key, value in pairs:
        if key in result:
            raise ProtocolError("duplicate JSON key: {}".format(key))
        result[key] = value
    return result


def _constant(value):
    raise ProtocolError("nonfinite JSON constant: {}".format(value))


def _float(value):
    return _number(float(value), "JSON number")


def loads_json(value):
    """Parse strict JSON, rejecting duplicate keys and NaN/Infinity literals."""
    try:
        return json.loads(value, object_pairs_hook=_pairs, parse_constant=_constant,
                          parse_float=_float)
    except (ValueError, TypeError, UnicodeError) as exc:
        raise ProtocolError("invalid JSON: {}".format(exc)) from exc


def point(latitude, longitude, altitude_ellipsoid_m=None, yaw=None):
    """Create a canonical JSON WGS84 ground point; altitude/yaw may be None."""
    latitude = _number(latitude, "latitude")
    longitude = _number(longitude, "longitude")
    if not -90 <= latitude <= 90:
        raise ProtocolError("latitude must be in [-90, 90]")
    if not -180 <= longitude <= 180:
        raise ProtocolError("longitude must be in [-180, 180]")
    return {"latitude": latitude, "longitude": longitude,
            "altitude_ellipsoid_m": (None if altitude_ellipsoid_m is None else
                                     _number(altitude_ellipsoid_m, "altitude_ellipsoid_m")),
            "yaw": None if yaw is None else _number(yaw, "yaw")}


def _validate_point(value, name, via=False):
    _object(value, name)
    if set(value) != POINT_FIELDS:
        raise ProtocolError("{} must contain exactly {}".format(name, sorted(POINT_FIELDS)))
    point(**value)
    if via and value["yaw"] is not None:
        raise ProtocolError("via points cannot specify yaw")


def validate_state(state):
    """Validate the state fields used for safe request construction.

    Remaining server execution/evidence fields are preserved unchanged. Timestamp
    age is transport-dependent and is checked by the ROS client.
    """
    _object(state, "state")
    try:
        _integer(state["protocol_version"], "protocol_version", 1, 1)
        _string(state["server_id"], "server_id")
        if state["last_sequence"] is not None:
            _integer(state["last_sequence"], "last_sequence")
        if _number(state["updated_stamp"], "updated_stamp") <= 0:
            raise ProtocolError("updated_stamp must be positive")
        if state["task_id"] is not None:
            _string(state["task_id"], "task_id")
        _string(state["phase"], "phase")
        if state["last_command"] is not None:
            _object(state["last_command"], "last_command")
    except KeyError as exc:
        raise ProtocolError("state is missing {}".format(exc.args[0])) from None
    return copy.deepcopy(state)


def load_target(path, yaw=None, via=False):
    """Read a capture with coordinates at root or in a single ``point`` object.

    Metadata must be at root. A capture must contain a finite ellipsoid height.
    File yaw is not part of the capture contract: provide final yaw explicitly.
    Both coordinate layouts together are rejected instead of guessing.
    """
    with open(path, "r", encoding="utf-8") as stream:
        value = loads_json(stream.read())
    _object(value, "target file")
    for key, expected in (("kind", "slicer_geographic_target"), ("datum", "WGS84"),
                          ("physical_reference", "base_link_horizontal")):
        if value.get(key) != expected:
            raise ProtocolError("target {} must be {}".format(key, expected))
    _integer(value.get("version"), "target version", 1, 1)
    if "point" in value:
        if any(key in value for key in POINT_FIELDS):
            raise ProtocolError("target has ambiguous root and point coordinates")
        coordinates = _object(value["point"], "target point")
    else:
        coordinates = value
    if coordinates.get("yaw") is not None:
        raise ProtocolError("capture yaw is unsupported; provide final yaw explicitly")
    if via and yaw is not None:
        raise ProtocolError("via points cannot specify yaw")
    try:
        height = _number(coordinates["altitude_ellipsoid_m"], "altitude_ellipsoid_m")
        return point(coordinates["latitude"], coordinates["longitude"], height, yaw)
    except KeyError as exc:
        raise ProtocolError("target is missing {}".format(exc.args[0])) from None


def validate_request(request):
    """Validate a canonical saved/client request without changing its payload."""
    _object(request, "request")
    if set(request) != REQUEST_FIELDS:
        raise ProtocolError("request must contain exactly {}".format(sorted(REQUEST_FIELDS)))
    _integer(request["protocol_version"], "protocol_version", 1, 1)
    for name in ("server_id", "request_id", "task_id"):
        _string(request[name], name)
    _integer(request["sequence"], "sequence")
    operation = _string(request["operation"], "operation")
    if operation not in OPERATIONS:
        raise ProtocolError("unsupported operation: {}".format(operation))
    via = request["via_points"]
    if not isinstance(via, list):
        raise ProtocolError("via_points must be an array")
    for index, value in enumerate(via):
        _validate_point(value, "via_points[{}]".format(index), via=True)
    if operation == "SET_TASK":
        _validate_point(request["final_goal"], "final_goal")
    elif request["final_goal"] is not None:
        raise ProtocolError("final_goal must be null for non-SET operations")
    if operation not in ("SET_TASK", "REPLACE_VIA_POINTS") and via:
        raise ProtocolError("{} cannot carry via_points".format(operation))
    return copy.deepcopy(request)


def build_request(state, operation, final_goal=None, via_points=None):
    """Allocate one new request from a fresh state (freshness checked by caller)."""
    state = validate_state(state)
    _string(operation, "operation")
    if operation not in OPERATIONS:
        raise ProtocolError("unsupported operation: {}".format(operation))
    sequence = (state["last_sequence"] or 0) + 1
    _integer(sequence, "sequence")
    if operation == "SET_TASK":
        task_id = str(uuid.uuid4())
    else:
        if state["task_id"] is None or state["phase"] not in UNFINISHED_PHASES:
            raise ProtocolError("operation requires a current unfinished task")
        task_id = state["task_id"]
    request = {"protocol_version": 1, "server_id": state["server_id"],
               "sequence": sequence, "request_id": str(uuid.uuid4()),
               "task_id": task_id, "operation": operation,
               "final_goal": copy.deepcopy(final_goal),
               "via_points": [] if via_points is None else copy.deepcopy(via_points)}
    return validate_request(request)


def save_request(request, path):
    """Persist before publication, atomically refusing to overwrite any path."""
    request = validate_request(request)
    payload = json.dumps(request, ensure_ascii=False, allow_nan=False, indent=2) + "\n"
    # O_EXCL also rejects symlinks. Only remove an incomplete file created here.
    descriptor = os.open(path, os.O_WRONLY | os.O_CREAT | os.O_EXCL, 0o600)
    try:
        with os.fdopen(descriptor, "w", encoding="utf-8") as stream:
            stream.write(payload)
            stream.flush()
            os.fsync(stream.fileno())
    except BaseException:
        os.unlink(path)
        raise


def load_request(path):
    """Load the exact validated saved request; no identities are regenerated."""
    with open(path, "r", encoding="utf-8") as stream:
        return validate_request(loads_json(stream.read()))
