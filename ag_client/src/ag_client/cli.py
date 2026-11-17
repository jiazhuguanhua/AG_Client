"""Command-line entry point. Commands never retry automatically."""
import argparse
import json
import math
import signal
import sys

from .protocol import ProtocolError, point, load_target, load_request


class _InitializationTimeout(BaseException):
    # rospy registration catches Exception internally; bypass that catch so the
    # alarm cannot be swallowed and leave initialization waiting forever.
    pass


def _init_ros(rospy, timeout):
    """Bound ROS registration as well as state waiting (Noetic on Linux)."""
    def expired(*_):
        raise _InitializationTimeout()

    old = signal.signal(signal.SIGALRM, expired)
    signal.setitimer(signal.ITIMER_REAL, timeout)
    try:
        rospy.init_node('ag_client', anonymous=True)
    except _InitializationTimeout:
        raise TimeoutError('ROS initialization timed out; check ROS_MASTER_URI and local node address') from None
    finally:
        signal.setitimer(signal.ITIMER_REAL, 0)
        signal.signal(signal.SIGALRM, old)


def parser():
    root = argparse.ArgumentParser(description='Air-ground V1 client; accepted is not task completion')
    subs = root.add_subparsers(dest='command', required=True)
    for name in ('status', 'health', 'task-status', 'set', 'replace-via', 'pause', 'resume', 'cancel', 'replay'):
        sub = subs.add_parser(name)
        for option, default in [('state-timeout', 5.), ('state-max-age', 2.),
                                ('connect-timeout', 3.), ('receipt-timeout', 3.), ('execution-max-age', 2.)]:
            sub.add_argument('--' + option, type=float, default=default)
        sub.add_argument('--command-topic', default='/air_ground/command')
        sub.add_argument('--state-topic', default='/air_ground/state')
        if name not in ('status', 'health', 'task-status', 'replay'):
            sub.add_argument('--save-request')
        if name in ('set', 'replace-via'):
            sub.add_argument('--via-file', action='append', default=[])
        if name == 'set':
            sub.add_argument('--target-file')
            sub.add_argument('--latitude', type=float)
            sub.add_argument('--longitude', type=float)
            sub.add_argument('--altitude-ellipsoid', type=float)
            sub.add_argument('--yaw', type=float)
        if name == 'replay':
            sub.add_argument('request_file')
        if name == 'task-status':
            sub.add_argument('task_id')
    return root


def main(argv=None):
    args = parser().parse_args(argv)
    try:
        for name in ('state_timeout', 'state_max_age', 'connect_timeout', 'receipt_timeout', 'execution_max_age'):
            value = getattr(args, name)
            if not math.isfinite(value) or value <= 0:
                raise ProtocolError(name + ' must be finite and positive')
        # Resolve files/coordinates before connecting to ROS.
        goal, via, request = None, None, None
        if args.command == 'set':
            if args.target_file:
                if any(value is not None for value in (args.latitude, args.longitude, args.altitude_ellipsoid)):
                    raise ProtocolError('--target-file cannot be mixed with coordinate options')
                goal = load_target(args.target_file, yaw=args.yaw)
            else:
                if args.latitude is None or args.longitude is None:
                    raise ProtocolError('set requires --target-file or both --latitude and --longitude')
                goal = point(args.latitude, args.longitude, args.altitude_ellipsoid, args.yaw)
        if args.command in ('set', 'replace-via'):
            via = [load_target(path, via=True) for path in args.via_file]
        if args.command == 'replay':
            request = load_request(args.request_file)
        import rospy
        from .client import AirGroundClient, CommandRejected, OutcomeUnknown
        _init_ros(rospy, args.state_timeout)
        options = {key: getattr(args, key) for key in
                   ('command_topic', 'state_topic', 'state_timeout', 'state_max_age', 'connect_timeout', 'receipt_timeout', 'execution_max_age')}
        with AirGroundClient(**options) as client:
            try:
                if args.command == 'status':
                    output = client.status()
                elif args.command == 'health':
                    output = client.health()
                elif args.command == 'task-status':
                    output = client.task_status(args.task_id)
                elif args.command == 'replay':
                    output = client.replay(request)
                else:
                    operations = {'set': 'SET_TASK', 'replace-via': 'REPLACE_VIA_POINTS',
                                  'pause': 'PAUSE', 'resume': 'RESUME', 'cancel': 'CANCEL'}
                    output = client.command(operations[args.command], final_goal=goal, via_points=via,
                                            save_path=args.save_request)
                print(json.dumps(output, ensure_ascii=False, indent=2, allow_nan=False))
                return 0
            except CommandRejected as exc:
                print(json.dumps(exc.receipt, ensure_ascii=False, indent=2), file=sys.stderr)
                return 1
            except OutcomeUnknown as exc:
                print(str(exc), file=sys.stderr)
                print(json.dumps({'published': True, 'request': exc.request}, ensure_ascii=False), file=sys.stderr)
                return 2
    except Exception as exc:
        # Includes ROSException/ROSInitException, which do not inherit RuntimeError.
        # Explicit remote rejection has already returned 1 above.
        print(str(exc), file=sys.stderr)
        return 2


if __name__ == '__main__':
    sys.exit(main())
