#!/usr/bin/env python3
"""Compare this client to an explicitly supplied vehicle source tree, offline.

Source the client's built Noetic workspace first. This script imports generated
messages and pure vehicle helpers; it never initializes ROS or starts a node.
Vehicle sources remain external and are neither copied nor modified.
"""
import argparse
import hashlib
import io
import json
from pathlib import Path
import sys
import tempfile

# Vehicle sources may be a read-only reference tree. Do not create pycache there.
sys.dont_write_bytecode = True


def sha256(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def message_definition(path):
    """Comments/blank lines do not participate in ROS message compatibility."""
    lines = (line.split('#', 1)[0].strip() for line in path.read_text().splitlines())
    return '\n'.join(line for line in lines if line)


def check_vehicle(vehicle_root):
    vehicle_root = vehicle_root.resolve(strict=True)
    repository = Path(__file__).resolve().parents[2]
    scripts = vehicle_root / 'src/ugv_navigation/scripts'
    required = [
        'src/ugv_navigation/scripts/air_ground.py',
        'src/ugv_navigation/scripts/air_ground_wire.py',
        'src/ugv_navigation/scripts/air_ground_core.py',
        'src/ugv_navigation/scripts/geographic_goal_core.py',
        'src/ugv_navigation/scripts/geographic_core.py',
        'src/ugv_navigation/scripts/geographic_state.py',
        'src/cup_gnss/src/cup_gnss/enu.py',
        'src/slicer_air_ground_msgs/msg/Command.msg',
        'src/slicer_air_ground_msgs/msg/GeoPoint.msg',
    ]
    baseline = {name: sha256(vehicle_root / name) for name in required}
    # Only explicit source directories are imported. Build/install/runtime
    # artifacts, configuration, logs, recorded observations and credentials are
    # not used.
    sys.path[:0] = [str(repository / 'ag_client/src'), str(scripts),
                   str(vehicle_root / 'src/cup_gnss/src')]
    import air_ground as vehicle_cli
    import air_ground_wire as vehicle_wire
    from air_ground_core import TaskCore
    from geographic_goal_core import capture_target
    from ag_client import protocol
    from ag_client.client import to_message
    from slicer_air_ground_msgs.msg import Command

    passed = []

    def require(condition, name):
        if not condition:
            raise AssertionError(name)
        passed.append(name)

    local_message_hashes = {}
    for name in ('Command.msg', 'GeoPoint.msg'):
        local_path = repository / 'slicer_air_ground_msgs/msg' / name
        remote_path = vehicle_root / 'src/slicer_air_ground_msgs/msg' / name
        require(message_definition(local_path) == message_definition(remote_path),
                name + ': identical ordered field/constant definitions')
        local_message_hashes[name] = sha256(local_path)
    require(Command._md5sum == 'b574bb9bb1240c0d2ae262d6c3feb2bf',
            'generated Command matches V1 MD5')

    # Artificial data passed directly to a pure function. No observations are
    # taken from a robot; no navigation or motion-producing function is called.
    origin = dict(latitude=30., longitude=120., altitude_ellipsoid_m=10.)
    samples = [dict(stamp=100. + index, base_enu=[.02 * index, 0., 2.],
                    antenna_height_m=12., session='offline-test') for index in range(2)]
    captured = capture_target(samples, origin,
                              dict(confirmed=True, window_start_stamp=100., window_end_stamp=101.), 1.)
    with tempfile.TemporaryDirectory(prefix='ag-vehicle-compat-') as temporary:
        directory = Path(temporary)
        target_path = directory / 'captured.json'
        target_path.write_text(json.dumps(captured), encoding='utf-8')
        goal = protocol.load_target(target_path)
        require(goal == vehicle_wire.load_target(target_path),
                'actual capture_target output loads identically without recompensation')

        core = TaskCore('offline-receiver-boot')

        def current_state():
            return dict(core.snapshot(), updated_stamp=100.)

        for operation in ('SET_TASK', 'REPLACE_VIA_POINTS', 'PAUSE', 'RESUME', 'CANCEL'):
            kwargs = {}
            if operation == 'SET_TASK':
                kwargs = dict(final_goal=goal, via_points=[protocol.point(30., 120.)])
            elif operation == 'REPLACE_VIA_POINTS':
                kwargs = dict(via_points=[])
            request = protocol.build_request(current_state(), operation, **kwargs)
            our_file = directory / (operation + '-client.json')
            vehicle_file = directory / (operation + '-vehicle.json')
            protocol.save_request(request, our_file)
            vehicle_cli.save_request(vehicle_file, request)
            require(protocol.load_request(vehicle_file) == request and
                    vehicle_wire.normalize_command(json.loads(our_file.read_text())) == request,
                    operation + ': saved requests cross-load in both directions')

            ours = to_message(request)
            theirs = vehicle_wire.make_command(request)
            stream = io.BytesIO()
            ours.serialize(stream)
            restored = Command().deserialize(stream.getvalue())
            decoded = vehicle_wire.decode_command(restored)
            require(ours == theirs and decoded == request,
                    operation + ': identical ROS mapping and receiver decode')
            receipt = core.handle(decoded)
            require(receipt['accepted'] is True and
                    all(receipt[name] == request[name] for name in
                        ('request_id', 'sequence', 'task_id', 'operation')),
                    operation + ': real TaskCore accepts and returns correlated receipt')
            before_replay = core.snapshot()
            replay = core.handle(vehicle_wire.decode_command(to_message(protocol.load_request(our_file))))
            require(replay['accepted'] is True and replay['duplicate'] is True and
                    core.snapshot() == before_replay,
                    operation + ': replay is a duplicate without state mutation')
            if operation == 'PAUSE':
                core.mark_stopped()  # Pure state transition, not a physical stop.
            elif operation == 'CANCEL':
                require(core.phase == 'CANCELING', 'real CANCEL first enters CANCELING')
                for blocked_operation in ('REPLACE_VIA_POINTS', 'PAUSE', 'RESUME', 'CANCEL'):
                    try:
                        protocol.build_request(current_state(), blocked_operation)
                    except protocol.ProtocolError:
                        passed.append('CANCELING: client rejects ' + blocked_operation + ' before publication')
                    else:
                        raise AssertionError('CANCELING: client allowed ' + blocked_operation)
                core.mark_stopped()
                require(core.phase == 'CANCELED', 'real cancellation ends after stop transition')

    return dict(result='PASS', checks=len(passed), passed=passed,
                protocol_version=1, command_md5=Command._md5sum,
                vehicle_source_sha256=baseline, client_message_source_sha256=local_message_hashes,
                limits='Offline source compatibility only; no ROS node, network, hardware, vehicle motion or radio test')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--vehicle-root', required=True, type=Path,
                        help='Explicit source root containing src/ugv_navigation and src/cup_gnss')
    args = parser.parse_args()
    try:
        result = check_vehicle(args.vehicle_root)
    except Exception as error:
        print(json.dumps(dict(result='FAIL', error=str(error)), ensure_ascii=False), file=sys.stderr)
        return 1
    print(json.dumps(result, ensure_ascii=False, indent=2, sort_keys=True))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
