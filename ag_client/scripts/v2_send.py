#!/usr/bin/env python3
"""Send one local V2 route intent to the UAV gateway."""
import argparse
import json
import time

import rospy
from std_msgs.msg import String

from ag_client.cli import _init_ros
from ag_client.v2 import INTENT, OPERATIONS, validate


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('op', choices=OPERATIONS)
    parser.add_argument('task_id')
    parser.add_argument('--lat', type=float)
    parser.add_argument('--lon', type=float)
    parser.add_argument('--mvp-id')
    parser.add_argument('--before-id')
    args = parser.parse_args(rospy.myargv()[1:])
    command = dict(op=args.op, task_id=args.task_id)
    if args.lat is not None or args.lon is not None:
        command['target' if args.op == 'SET_TARGET' else 'point'] = dict(lat=args.lat, lon=args.lon)
    if args.mvp_id is not None:
        command['mvp_id'] = args.mvp_id
    if args.before_id is not None:
        command['before_id'] = args.before_id
    try:
        payload = json.dumps(validate(command, wire=False), allow_nan=False)
    except ValueError as exc:
        parser.error(str(exc))
    _init_ros(rospy, 5.)
    publisher = rospy.Publisher(INTENT, String, queue_size=1, latch=False)
    deadline = time.monotonic() + 3.
    while not publisher.get_num_connections():
        if rospy.is_shutdown() or time.monotonic() >= deadline:
            raise SystemExit('UAV gateway is not connected')
        time.sleep(.05)
    publisher.publish(String(data=payload))
    time.sleep(.2)
    print('一次意图已提交给 UAV gateway，请看 UAV/UGV state。')


if __name__ == '__main__':
    try:
        main()
    except (TimeoutError, rospy.ROSException) as exc:
        raise SystemExit(str(exc))
