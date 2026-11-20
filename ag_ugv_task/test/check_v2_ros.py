#!/usr/bin/env python3
"""Exercise both V2 nodes on a private loopback ROS master and fake navigator."""
import json
import os
import signal
import socket
import subprocess
import tempfile
import threading
import time
from xmlrpc.client import ServerProxy


class FakeNavigation:
    def __init__(self):
        self.goals = []
        self.cancels = 0
        self.state = 2
        self.xy = (0., 0.)
        self.held = False

    def validate_route(self, final, vias):
        assert final and all(vias)

    def target(self, point):
        return dict(map_xyz=[point['latitude'], point['longitude'], 0])

    def start(self, target):
        assert self.state != 1
        self.goals.append(target['map_xyz'])
        self.state = 1

    def active_server_goal(self):
        return self.state == 1

    def health(self, target=None):
        return None

    def mux_state(self):
        return dict(manual_hold=self.held, emergency_stop_latched=False, teleop_fresh=False,
                    mux_instance_id='test', takeover_generation=1)

    def hold(self):
        self.held = True
        return self.mux_state()

    def release(self, expected):
        assert expected == ('test', 1)
        self.held = False
        return self.mux_state()

    def poll(self):
        return dict(state=self.state)

    def cancel(self):
        self.cancels += 1

    def position(self):
        return self.xy

    def stopped(self, since):
        return dict(confirmed=self.held and self.state != 1)

    def review(self, target, action, since):
        return dict(geographic=dict(qualified=True, confirmed=True), stop=dict(confirmed=True))


def wait_for(check, label, timeout=5):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        result = check()
        if result:
            return result
        time.sleep(.02)
    raise AssertionError('Timed out waiting for ' + label)


def run(executor_type):
    import rospy
    from sensor_msgs.msg import NavSatFix
    from std_msgs.msg import String
    from ag_uav_client.v2 import STATE, UAV_STATE, vehicle_position
    from ag_ugv_task.node import GroundNode

    rospy.init_node('ag_v2_integration_test')
    backend = FakeNavigation()
    ground = GroundNode(backend, executor_type, lambda: vehicle_position(30.5, 118.5, 1.))
    worker = threading.Thread(target=ground.run, daemon=True)
    worker.start()
    gateway = subprocess.Popen(['rosrun', 'ag_uav_client', 'ag_uav_gateway.py'], start_new_session=True)
    last = {}
    observed = [0]
    def record(msg, key):
        last[key] = json.loads(msg.data)
    listeners = [rospy.Subscriber(topic, String, record, callback_args=key)
                 for topic, key in [(STATE, 'ugv'), (UAV_STATE, 'uav')]]

    def state(check, label):
        return wait_for(lambda: last.get('ugv') if check(last.get('ugv', {})) else None, label)

    def send(op, task='line', accepted=True, **options):
        args = ['rosrun', 'ag_uav_client', 'v2_send.py', op, task]
        for key, value in options.items():
            args.extend(['--' + key.replace('_', '-'), str(value)])
        subprocess.run(args, check=True, timeout=10, stdout=subprocess.DEVNULL)
        receipt = 'last_qr_report' if op == 'REPORT_QR' else 'last_command'
        def check(value):
            cmd = value.get(receipt) or {}
            return cmd.get('op') == op and cmd['seq'] > observed[0]
        cmd = state(check, op)[receipt]
        assert cmd['accepted'] is accepted, cmd
        assert cmd['received_at'] >= cmd['sent_at']
        observed[0] = cmd['seq']
        return cmd

    try:
        rospy.loginfo('Simulation only: no flight, real navigation or chassis nodes.')
        state(lambda s: s.get('phase') == 'IDLE', 'UGV idle')
        wait_for(lambda: last.get('uav'), 'UAV initial state')
        fix_pub = rospy.Publisher('/uav/fix', NavSatFix, queue_size=1)
        wait_for(fix_pub.get_num_connections, 'UAV fix subscription')
        fix = NavSatFix(latitude=31.4, longitude=118.5)
        fix.header.stamp = rospy.Time.now()
        fix_pub.publish(fix)
        wait_for(lambda: last.get('uav', {}).get('vehicle_position', {}).get('lat') == 31.4,
                 'UAV GPS snapshot')

        first_qr = dict(qr_id='QR002', lat=30, lon=118, measured_at=1, revision=1,
                        altitude_m=40, image_id='qr002-photo')
        receipt = send('REPORT_QR', **first_qr)
        assert receipt['qr_id'] == 'QR002'
        state(lambda s: s.get('qr_reports') == [dict(first_qr, task_id='line')], 'first QR report')
        wait_for(lambda: (last.get('uav', {}).get('last_qr_report') or {}).get('seq') == receipt['seq'],
                 'UAV QR send status')
        assert last['ugv']['phase'] == 'IDLE' and last['ugv']['last_command'] is None
        assert last['uav']['task_id'] is None and not backend.goals
        send('REPORT_QR', **first_qr)
        assert len(last['ugv']['qr_reports']) == 1
        changed_qr = dict(first_qr, revision=2, image_id='qr002-new-photo')
        send('REPORT_QR', **changed_qr)
        stale = send('REPORT_QR', accepted=False, **first_qr)
        assert stale['reason'] == 'STALE_QR_REVISION'
        assert last['ugv']['qr_reports'] == [dict(changed_qr, task_id='line')]

        send('INSERT_MVP', accepted=False, mvp_id='missing', lat=31, lon=118)
        send('SET_TARGET', lat=32, lon=119, image_id='target-photo')
        state(lambda s: (s.get('active_point') or {}).get('kind') == 'TARGET', 'original target')
        target_receipt = send('SET_TARGET', lat=32, lon=119, image_id='updated-target-photo')
        assert last['ugv']['target'] == dict(lat=32, lon=119, image_id='updated-target-photo')
        send('REPORT_QR', qr_id='QR003', lat=31, lon=118, measured_at=2, revision=1)
        assert len(last['ugv']['qr_reports']) == 2
        assert last['ugv']['last_command'] == target_receipt
        assert last['ugv']['phase'] == 'EXECUTING'
        assert backend.cancels == 0 and backend.goals == [[32, 119, 0]]
        send('INSERT_MVP', mvp_id='p1', lat=33, lon=120)
        state(lambda s: s.get('phase') == 'SWITCHING', 'switching to p1')
        assert backend.cancels == 1 and len(backend.goals) == 1
        backend.state = 2
        state(lambda s: (s.get('active_point') or {}).get('mvp_id') == 'p1', 'p1')
        send('SET_TARGET', lat=34, lon=121)
        assert backend.cancels == 1 and len(backend.goals) == 2
        send('INSERT_MVP', mvp_id='p0', before_id='p1', lat=35, lon=122)
        state(lambda s: s.get('phase') == 'SWITCHING', 'switching to p0')
        assert backend.cancels == 2 and len(backend.goals) == 2
        backend.state = 2
        state(lambda s: (s.get('active_point') or {}).get('mvp_id') == 'p0', 'p0')
        send('UPDATE_MVP', mvp_id='p0', lat=36, lon=123)
        state(lambda s: s.get('phase') == 'SWITCHING', 'switching to edited p0')
        backend.state = 2
        state(lambda s: (s.get('active_point') or {}).get('lat') == 36, 'edited p0')
        send('DEL_MVP', mvp_id='p1')
        assert backend.cancels == 3
        send('PAUSE')
        state(lambda s: s.get('phase') == 'PAUSING', 'pausing')
        backend.state = 2
        state(lambda s: s.get('phase') == 'PAUSED', 'paused')
        send('INSERT_MVP', mvp_id='p2', lat=37, lon=124)
        assert last['ugv']['active_point'] is None
        send('RESUME')
        state(lambda s: (s.get('active_point') or {}).get('mvp_id') == 'p0', 'resumed p0')
        backend.xy = (36, 123)
        state(lambda s: [p['mvp_id'] for p in s.get('remaining_mvps', [])] == ['p2'], 'p0 passed')
        backend.state = 2
        state(lambda s: (s.get('active_point') or {}).get('mvp_id') == 'p2', 'p2')
        backend.xy = (37, 124)
        state(lambda s: not s.get('remaining_mvps'), 'p2 passed')
        backend.state = 2
        state(lambda s: (s.get('active_point') or {}).get('lat') == 34, 'updated final target')
        backend.state = 3
        state(lambda s: s.get('phase') == 'COMPLETED', 'completed')
        send('SET_TARGET', task='next', lat=38, lon=125)
        state(lambda s: s.get('phase') == 'EXECUTING', 'second task')
        send('STOP', task='next')
        state(lambda s: s.get('phase') == 'STOPPING', 'stopping')
        backend.state = 2
        stopped = state(lambda s: s.get('phase') == 'STOPPED', 'stopped')
        assert stopped['target'] is None and stopped['active_point'] is None
        send('RESUME', task='next', accepted=False)
        print('PASS: all 8 operations, QR reports and image IDs, arrival and confirmed stop')
    finally:
        rospy.signal_shutdown('test finished')
        worker.join(timeout=4)
        if gateway.poll() is None:
            os.killpg(gateway.pid, signal.SIGINT)
            gateway.wait(timeout=8)


def main():
    from ag_ugv_task.executor import TaskExecutor

    with socket.socket() as free:
        free.bind(('127.0.0.1', 0))
        port = free.getsockname()[1]
    with tempfile.TemporaryDirectory(prefix='ag-v2-ros-') as logs:
        uri = 'http://127.0.0.1:' + str(port)
        os.environ.update(ROS_MASTER_URI=uri, ROS_IP='127.0.0.1', ROS_LOG_DIR=logs)
        os.environ.pop('ROS_HOSTNAME', None)
        core = subprocess.Popen(['roscore', '-p', str(port)], stdout=subprocess.DEVNULL,
                                stderr=subprocess.DEVNULL, start_new_session=True)
        try:
            def master_alive():
                try:
                    return ServerProxy(uri).getPid('/v2_test')[2]
                except OSError:
                    return False
            wait_for(master_alive, 'private ROS master')
            run(TaskExecutor)
        finally:
            os.killpg(core.pid, signal.SIGINT)
            core.wait(timeout=10)


if __name__ == '__main__':
    main()
