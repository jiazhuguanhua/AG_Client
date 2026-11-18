#!/usr/bin/env python3
"""Interactive V1 wire tester for the UAV terminal."""
import math
import threading
import time
import uuid

import rosgraph
import rospy
from slicer_air_ground_msgs.msg import Command
from std_msgs.msg import String

from ag_client.cli import _init_ros
from ag_client.client import to_message
from ag_client.protocol import loads_json, point

STATE_TOPIC = '/air_ground/state'
COMMAND_TOPIC = '/air_ground/command'
OPERATIONS = {'2': 'SET_TASK', '3': 'PAUSE', '4': 'RESUME', '5': 'CANCEL'}


class StateReader:
    def __init__(self):
        self.changed = threading.Condition()
        self.frame = None
        self.counter = 0
        self.subscriber = rospy.Subscriber(STATE_TOPIC, String, self.receive, queue_size=1)

    def receive(self, message):
        try:
            state = loads_json(message.data)
            stamp = state['updated_stamp']
            if type(stamp) not in (int, float) or not math.isfinite(stamp) or stamp <= 0:
                return
        except (ValueError, TypeError, KeyError, OverflowError):
            return
        with self.changed:
            self.counter += 1
            self.frame = (self.counter, time.monotonic(), state, message.data)
            self.changed.notify_all()

    def fresh_frame(self, after=None):
        deadline = time.monotonic() + 5.0
        with self.changed:
            if after is None:
                after = self.counter
            while not rospy.is_shutdown():
                now = time.monotonic()
                if self.frame:
                    counter, received, state, raw = self.frame
                    age = rospy.Time.now().to_sec() - state['updated_stamp']
                    if counter > after and now - received <= 2.0 and 0 <= age <= 2.0:
                        return state, raw
                if now >= deadline:
                    break
                self.changed.wait(min(0.1, deadline - now))
        raise TimeoutError(f'5 秒内没有读到 {STATE_TOPIC} 的新鲜帧')


def send(publisher, reader, operation, goal, task_id):
    deadline = time.monotonic() + 3.0
    while not publisher.get_num_connections():
        if rospy.is_shutdown() or time.monotonic() >= deadline:
            raise TimeoutError(f'{COMMAND_TOPIC} 没有连接的订阅者，本次未发布')
        time.sleep(0.05)

    state, _ = reader.fresh_frame()
    request = {
        'protocol_version': 1,
        'server_id': state['server_id'],
        'sequence': (state['last_sequence'] or 0) + 1,
        'request_id': str(uuid.uuid4()),
        'task_id': task_id or (str(uuid.uuid4()) if goal is not None else state['task_id']),
        'operation': operation,
        'final_goal': goal,
        'via_points': [],
    }
    message = to_message(request)
    with reader.changed:
        after = reader.counter
        publisher.publish(message)
    print(f'已发布一条 {operation} · seq={message.sequence} · '
          f'request_id={message.request_id} · task_id={message.task_id}', flush=True)
    try:
        _, raw = reader.fresh_frame(after)
        print(f'{STATE_TOPIC} 发布后收到的一帧（不等同于回执）')
        print(raw, flush=True)
    except TimeoutError as exc:
        print(f'命令已发布，{exc}。')


def main():
    _init_ros(rospy, 5.0)
    reader = StateReader()
    publisher = rospy.Publisher(COMMAND_TOPIC, Command, queue_size=1, latch=False)
    print(f'UAV → UGV 话题测试 · {rosgraph.get_master_uri()}')
    print('选发送或任务操作会直接发布给真车。请只输入现场可用目标。')
    while not rospy.is_shutdown():
        print('\n1 读一帧状态  2 发送目标  3 暂停  4 恢复  5 取消  0 退出')
        choice = input('选择 > ').strip()
        if choice == '0':
            return
        try:
            if choice == '1':
                _, raw = reader.fresh_frame()
                print(raw, flush=True)
            elif choice in OPERATIONS:
                goal, task_id = None, None
                if choice == '2':
                    lat = float(input('WGS84 纬度 > '))
                    lon = float(input('WGS84 经度（回车发送） > '))
                    goal = point(lat, lon)
                else:
                    task_id = input('task_id（回车使用车端状态中的任务并发送） > ').strip()
                send(publisher, reader, OPERATIONS[choice], goal, task_id)
            else:
                print('请选择菜单中的编号。')
        except (ValueError, KeyError, TypeError, TimeoutError, rospy.ROSException) as exc:
            print(f'操作失败：{exc}', flush=True)


if __name__ == '__main__':
    try:
        main()
    except (EOFError, KeyboardInterrupt, rospy.ROSInterruptException):
        pass
    except (TimeoutError, rospy.ROSException) as exc:
        raise SystemExit(str(exc))
