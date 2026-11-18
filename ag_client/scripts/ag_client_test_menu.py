#!/usr/bin/env python3
"""Interactive UAV ground-test menu using the existing AG_Client CLI."""
import os
from pathlib import Path
import subprocess
import sys
import tempfile

from ag_client.protocol import point


OPERATIONS = {
    '1': 'health', '2': 'status', '3': 'set', '4': 'task-status',
    '5': 'pause', '6': 'resume', '7': 'cancel',
}
MUTATIONS = {'set', 'pause', 'resume', 'cancel'}
MENU = (
    '\n1 健康检查  2 查看状态  3 发送目标  4 查询任务\n'
    '5 暂停      6 恢复      7 取消      0 退出'
)


def main():
    print('AG_Client UAV → UGV 地面测试')
    print('ROS master:', os.environ.get('ROS_MASTER_URI', 'http://localhost:11311'))
    print('本机 ROS 地址:', os.environ.get('ROS_HOSTNAME') or os.environ.get('ROS_IP') or '自动选择')
    print('使用 /air_ground/command 和 /air_ground/state。暂停、恢复和取消作用于车端当前任务。')
    while True:
        print(MENU)
        choice = input('选择 > ').strip()
        if choice == '0':
            return
        operation = OPERATIONS.get(choice)
        if operation is None:
            print('请选择菜单中的编号。')
            continue
        args = [operation]
        if operation == 'set':
            try:
                goal = point(float(input('WGS84 纬度 > ')), float(input('WGS84 经度 > ')))
            except ValueError as exc:
                print('坐标无效：', exc)
                continue
            lat, lon = goal['latitude'], goal['longitude']
            print(f'将发送地面目标：纬度 {lat:.9f}，经度 {lon:.9f}')
            args.extend(['--latitude', str(lat), '--longitude', str(lon)])
        elif operation == 'task-status':
            task_id = input('回执中的 task_id > ').strip()
            if not task_id:
                continue
            args.extend(['--', task_id])

        if operation in MUTATIONS:
            print('这项操作会改变真车任务状态。请确认现场人员已知悉。')
            if input(f'输入 {operation} 确认 > ').strip() != operation:
                print('已取消，本次未发送。')
                continue
            request = Path(tempfile.mkdtemp(prefix='ag-ugv-test-')) / 'request.json'
            args.extend(['--save-request', str(request)])
            print('请求记录路径：', request, flush=True)

        result = subprocess.run([sys.executable, '-m', 'ag_client.cli', *args], check=False)
        if operation in MUTATIONS:
            if result.returncode == 0:
                print('车端已受理。可用菜单 4 查询完成状态。')
            elif result.returncode == 1:
                print('车端拒绝，原因见上方回执。')
            else:
                print('未取得明确受理结果。先查状态，避免重复发送。')


if __name__ == '__main__':
    try:
        main()
    except (EOFError, KeyboardInterrupt):
        print('\n已退出。若发送时中断，请先查看车端状态。')
