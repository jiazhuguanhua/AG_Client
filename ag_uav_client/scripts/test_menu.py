#!/usr/bin/env python3
"""Interactive V2 tester. All commands go through the running UAV gateway."""
import threading
import time
import uuid

import rosgraph
import rospy
from std_msgs.msg import String

from ag_uav_client.ros_runtime import init_ros, publish_intent
from ag_uav_client.v2 import INTENT, STATE, UAV_STATE, decode, number

OPERATIONS = {
    '2': 'SET_TARGET', '3': 'INSERT_MVP', '4': 'DEL_MVP', '5': 'UPDATE_MVP',
    '6': 'PAUSE', '7': 'RESUME', '8': 'STOP',
}
TOPICS = {'ugv': STATE, 'uav': UAV_STATE}
MENU = ('\n1 读 UGV 状态  2 设置/更新终点  3 插入 MVP  4 删除 MVP  5 更新 MVP\n'
        '6 暂停         7 恢复            8 停止任务  9 读 UAV 状态  0 退出')


class StateReader:
    def __init__(self):
        self.changed = threading.Condition()
        self.frames = {}
        self.counter = 0
        self.subscribers = [
            rospy.Subscriber(topic, String, self.receive, callback_args=vehicle, queue_size=1)
            for vehicle, topic in TOPICS.items()
        ]

    def receive(self, message, vehicle):
        try:
            state = decode(message.data)
        except ValueError:
            return
        stamp = state.get('updated_at')
        if (state.get('version') != 2 or state.get('vehicle_id') != vehicle
                or not number(stamp) or stamp <= 0):
            return
        with self.changed:
            self.counter += 1
            self.frames[vehicle] = (self.counter, state, message.data)
            self.changed.notify_all()

    def fresh_frame(self, vehicle):
        deadline = time.monotonic() + 5.
        with self.changed:
            after = self.counter
            while not rospy.is_shutdown():
                frame = self.frames.get(vehicle)
                if frame:
                    counter, state, raw = frame
                    age = rospy.Time.now().to_sec() - state['updated_at']
                    if counter > after and 0 <= age <= 2.:
                        return state, raw
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    break
                self.changed.wait(min(.1, remaining))
        raise TimeoutError(f'5 秒内没有读到 {TOPICS[vehicle]} 的新鲜帧')


def read_intent(operation, last_task):
    default = last_task or ('run-' + uuid.uuid4().hex[:12] if operation == 'SET_TARGET' else '')
    task = input(f'任务 ID（回车使用 {default or "无默认值"}）> ').strip() or default
    if not task:
        raise ValueError('请填写任务 ID')
    intent = dict(op=operation, task_id=task)
    if operation in ('INSERT_MVP', 'DEL_MVP', 'UPDATE_MVP'):
        intent['mvp_id'] = input('MVP ID > ').strip()
    if operation == 'INSERT_MVP':
        before = input('插在这个 MVP ID 前（回车追加到终点前）> ').strip()
        if before:
            intent['before_id'] = before
    if operation in ('SET_TARGET', 'INSERT_MVP', 'UPDATE_MVP'):
        lat = float(input('WGS84 纬度 > '))
        lon = float(input('WGS84 经度 > '))
        intent['target' if operation == 'SET_TARGET' else 'point'] = dict(lat=lat, lon=lon)
    return intent


def main():
    init_ros(rospy, name='ag_v2_test_menu')
    reader = StateReader()
    publisher = rospy.Publisher(INTENT, String, queue_size=1, latch=False)
    print(f'UAV → UGV V2 测试 · {rosgraph.get_master_uri()}')
    print('发送操作将提交给 UAV gateway，可能使已就绪的小车行驶。')
    last_task = None
    while not rospy.is_shutdown():
        print(MENU)
        choice = input('选择 > ').strip()
        if choice == '0':
            return

        vehicle = 'uav' if choice == '9' else 'ugv'
        try:
            if choice in OPERATIONS:
                operation = OPERATIONS[choice]
                intent = read_intent(operation, last_task)
                publish_intent(rospy, publisher, intent)
                last_task = intent['task_id']
                print(f'已向本机 gateway 提交 {operation} · {last_task}')
                print('随后打印一帧 UGV 状态。受理结果见匹配的 last_command。')
            elif choice not in ('1', '9'):
                print('请选择菜单中的编号。')
                continue
            state, raw = reader.fresh_frame(vehicle)
            print(raw, flush=True)
            if vehicle == 'ugv':
                phase, task = state.get('phase'), state.get('task_id')
                if phase in ('STOPPED', 'COMPLETED') and task == last_task:
                    last_task = None
                elif (choice == '1' and isinstance(task, str) and task
                      and phase not in ('STOPPED', 'COMPLETED')):
                    last_task = task
        except (ValueError, TimeoutError, rospy.ROSException) as exc:
            print(f'操作提示：{exc}', flush=True)


if __name__ == '__main__':
    try:
        main()
    except (EOFError, KeyboardInterrupt, rospy.ROSInterruptException):
        pass
    except (TimeoutError, rospy.ROSException) as exc:
        raise SystemExit(str(exc))
