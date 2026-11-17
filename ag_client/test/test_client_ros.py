#!/usr/bin/env python3
"""Transport tests against a fake receiver. No vehicle or flight interfaces."""
import copy
import io
import json
import os
import socket
import subprocess
import sys
import tempfile
import threading
import time
import unittest

import rospy
import rostest
from std_msgs.msg import String
from slicer_air_ground_msgs.msg import Command

from ag_client.client import AirGroundClient, ClientError, CommandRejected, OutcomeUnknown, to_message
from ag_client.protocol import build_request, load_request, point


def until(predicate, timeout=3.):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if predicate():
            return
        time.sleep(.01)
    raise AssertionError('Timed out waiting for test condition')


class FakeReceiver:
    def __init__(self, namespace):
        self.lock = threading.RLock()
        self.commands = []
        self.headers = []
        self.mode = 'normal'
        self.offset = 0.
        self.invalid = None
        self.emit = True
        self.cache = {}
        self.state = {'protocol_version': 1, 'server_id': 'test-boot', 'last_sequence': None,
                      'updated_stamp': time.time(), 'task_id': None, 'phase': 'IDLE',
                      'last_command': None, 'execution_updated_stamp': time.time(),
                      'startup_checked': True, 'runtime_phase': 'IDLE', 'cancellation_pending': False,
                      'datum': 'WGS84', 'physical_reference': 'base_link_horizontal'}
        self.worker_stalled = False
        self.publisher = rospy.Publisher(namespace + '/state', String, queue_size=1, latch=True)
        self.subscriber = rospy.Subscriber(namespace + '/command', Command, self.command, queue_size=10)
        self.timer = rospy.Timer(rospy.Duration(.02), self.tick)

    def tick(self, _):
        with self.lock:
            if self.emit:
                self.state['updated_stamp'] = rospy.Time.now().to_sec() + self.offset
                if not self.worker_stalled:
                    self.state['execution_updated_stamp'] = rospy.Time.now().to_sec()
                payload = self.invalid if self.invalid is not None else json.dumps(self.state)
                self.publisher.publish(String(data=payload))

    def command(self, msg):
        with self.lock:
            self.commands.append(copy.deepcopy(msg))
            self.headers.append(dict(msg._connection_header or {}))
            if self.mode == 'drop':
                return
            if self.mode == 'restart':
                self.state['server_id'] = 'new-test-boot'
                self.state['last_sequence'] = None
                self.state['last_command'] = None
                return
            receipt = {key: getattr(msg, key) for key in
                       ('request_id', 'sequence', 'task_id', 'operation')}
            duplicate = msg.request_id in self.cache
            if duplicate:
                receipt = copy.deepcopy(self.cache[msg.request_id])
            else:
                receipt.update(accepted=self.mode != 'reject', reason='test rejection' if self.mode == 'reject' else '')
                self.cache[msg.request_id] = copy.deepcopy(receipt)
                self.state['last_sequence'] = msg.sequence
                if receipt['accepted']:
                    self.state['task_id'] = msg.task_id
                    self.state['phase'] = {'SET_TASK': 'READY', 'REPLACE_VIA_POINTS': 'READY',
                                           'PAUSE': 'PAUSED', 'RESUME': 'EXECUTING', 'CANCEL': 'CANCELED'}[msg.operation]
            receipt['duplicate'] = duplicate
            if self.mode == 'mismatch':
                receipt['request_id'] = 'unrelated-request'
            self.state['last_command'] = receipt

    def close(self):
        self.timer.shutdown()
        self.subscriber.unregister()
        self.publisher.unregister()


class ClientRosTests(unittest.TestCase):
    def setUp(self):
        self.namespace = '/test_ag/' + self._testMethodName
        self.server = FakeReceiver(self.namespace)
        self.client = self.make_client()
        self.directory = tempfile.TemporaryDirectory(prefix='ag-client-test-')

    def make_client(self, **kwargs):
        options = dict(command_topic=self.namespace + '/command', state_topic=self.namespace + '/state',
                       state_timeout=1., state_max_age=.3, connect_timeout=2., receipt_timeout=.5)
        options.update(kwargs)
        return AirGroundClient(**options)

    def tearDown(self):
        self.client.close()
        self.server.close()
        self.directory.cleanup()

    def test_five_operations_and_wire_payload(self):
        goal = point(30., 120., 25., .8)
        receipt = self.client.set_task(goal, [point(30.0001, 120.0001)])
        self.assertTrue(receipt['accepted'])
        task = receipt['task_id']
        self.client.replace_via_points([])
        self.client.pause()
        self.client.resume()
        self.client.cancel()
        commands = self.server.commands
        self.assertEqual([m.operation for m in commands],
                         ['SET_TASK', 'REPLACE_VIA_POINTS', 'PAUSE', 'RESUME', 'CANCEL'])
        self.assertEqual([m.sequence for m in commands], [1, 2, 3, 4, 5])
        self.assertTrue(all(m.task_id == task for m in commands))
        self.assertEqual(commands[0].final_goal.latitude, goal['latitude'])
        self.assertTrue(commands[0].final_goal.has_altitude)
        self.assertTrue(commands[0].final_goal.has_yaw)
        self.assertFalse(commands[0].via_points[0].has_altitude)
        self.assertFalse(commands[0].via_points[0].has_yaw)
        for msg in commands[1:]:
            self.assertEqual(msg.final_goal.latitude, 0.)
            self.assertEqual(msg.final_goal.longitude, 0.)
            self.assertFalse(msg.final_goal.has_altitude)
            self.assertFalse(msg.final_goal.has_yaw)
        self.assertTrue(all(header.get('latching') == '0' for header in self.server.headers))

    def test_exact_md5_and_serialization(self):
        self.assertEqual(Command._md5sum, 'b574bb9bb1240c0d2ae262d6c3feb2bf')
        request = build_request(self.client.status(), 'SET_TASK', point(1., 2., None, .5), [point(3., 4.)])
        msg = to_message(request)
        buffer = io.BytesIO()
        msg.serialize(buffer)
        restored = Command().deserialize(buffer.getvalue())
        self.assertEqual(msg, restored)

    def test_saved_replay_is_exact_and_duplicate(self):
        path = os.path.join(self.directory.name, 'request.json')
        original = self.client.set_task(point(1., 2.), save_path=path)
        request = load_request(path)
        replay = self.client.replay(request)
        self.assertEqual(original['request_id'], replay['request_id'])
        self.assertTrue(replay['duplicate'])
        self.assertEqual(self.server.commands[0], self.server.commands[1])
        self.assertEqual(os.stat(path).st_mode & 0o777, 0o600)

    def test_save_never_overwrites_or_publishes(self):
        path = os.path.join(self.directory.name, 'existing.json')
        with open(path, 'w') as stream:
            stream.write('keep')
        with self.assertRaises(FileExistsError):
            self.client.set_task(point(1., 2.), save_path=path)
        self.assertEqual(self.server.commands, [])
        with open(path) as stream:
            self.assertEqual(stream.read(), 'keep')

    def test_commands_are_nonlatched_and_not_replayed_on_connect(self):
        self.client.set_task(point(1., 2.))
        self.assertEqual(self.server.headers[0].get('latching'), '0')
        # rospy shares one transport per node/topic; disconnect the first
        # subscriber before checking a genuinely new connection.
        self.server.subscriber.unregister()
        until(lambda: self.client._publisher.get_num_connections() == 0)
        late = []
        subscriber = rospy.Subscriber(self.namespace + '/command', Command, late.append, queue_size=1)
        try:
            until(lambda: self.client._publisher.get_num_connections() == 1)
            time.sleep(.1)
            self.assertEqual(late, [])
        finally:
            subscriber.unregister()

    def test_old_boot_replay_is_never_published(self):
        request = build_request(self.client.status(), 'SET_TASK', point(1., 2.))
        request['server_id'] = 'old-boot'
        with self.assertRaises(ClientError) as caught:
            self.client.replay(request)
        self.assertFalse(caught.exception.published)
        self.assertEqual(self.server.commands, [])

    def test_future_and_stale_ros_stamps_are_rejected(self):
        for offset in (30., -30.):
            with self.server.lock:
                self.server.offset = offset
            # Wait for the fake server's deliberately bad sample to arrive.
            until(lambda: self.client._state is not None and
                  abs(self.client._state['updated_stamp'] - time.time()) > 10)
            started = time.monotonic()
            with self.assertRaises(ClientError):
                self.client.status()
            self.assertLess(time.monotonic() - started, 1.5)
        self.assertEqual(self.server.commands, [])

    def test_invalid_json_invalidates_previous_good_state(self):
        self.client.status()
        with self.server.lock:
            self.server.invalid = '{"protocol_version":NaN}'
        until(lambda: self.client._state is None)
        with self.assertRaises(ClientError):
            self.client.status()
        self.assertEqual(self.server.commands, [])

    def test_local_receive_age_expires_when_publication_stops(self):
        self.client.status()
        with self.server.lock:
            self.server.emit = False
        time.sleep(.4)
        with self.assertRaises(ClientError):
            self.client.status()

    def test_no_subscriber_does_not_publish(self):
        self.server.subscriber.unregister()
        with self.make_client(command_topic=self.namespace + '/absent', connect_timeout=.15) as client:
            with self.assertRaises(ClientError) as caught:
                client.set_task(point(1., 2.))
            self.assertFalse(caught.exception.published)
        self.assertEqual(self.server.commands, [])

    def test_receipt_timeout_is_unknown_and_never_retries_or_cancels(self):
        self.server.mode = 'drop'
        with self.assertRaises(OutcomeUnknown) as caught:
            self.client.set_task(point(1., 2.))
        self.assertTrue(caught.exception.published)
        self.assertEqual(caught.exception.request['request_id'], self.server.commands[0].request_id)
        time.sleep(.1)
        self.assertEqual([m.operation for m in self.server.commands], ['SET_TASK'])

    def test_receipt_must_match_request(self):
        self.server.mode = 'mismatch'
        with self.assertRaises(OutcomeUnknown):
            self.client.set_task(point(1., 2.))
        self.assertEqual(len(self.server.commands), 1)

    def test_explicit_rejection_is_distinct_from_timeout(self):
        self.server.mode = 'reject'
        with self.assertRaises(CommandRejected) as caught:
            self.client.set_task(point(1., 2.))
        self.assertFalse(caught.exception.receipt['accepted'])
        self.assertEqual(str(caught.exception), 'test rejection')

    def test_receiver_restart_after_publish_is_unknown(self):
        self.server.mode = 'restart'
        with self.assertRaises(OutcomeUnknown) as caught:
            self.client.set_task(point(1., 2.))
        self.assertIn('restarted', str(caught.exception))
        self.assertEqual(len(self.server.commands), 1)

    def test_concurrent_local_clients_cannot_send(self):
        self.server.mode = 'drop'
        result = []

        def send():
            try:
                self.client.set_task(point(1., 2.))
            except OutcomeUnknown as exc:
                result.append(exc)

        thread = threading.Thread(target=send)
        thread.start()
        until(lambda: len(self.server.commands) == 1)
        try:
            with self.make_client() as other:
                with self.assertRaises(ClientError) as caught:
                    other.set_task(point(3., 4.))
                self.assertFalse(caught.exception.published)
        finally:
            thread.join(3.)
        self.assertFalse(thread.is_alive())
        self.assertEqual(len(result), 1)
        self.assertEqual(len(self.server.commands), 1)

    def run_cli(self, *arguments):
        command = [sys.executable, '-m', 'ag_client.cli'] + list(arguments)
        command += ['--state-topic', self.namespace + '/state',
                    '--command-topic', self.namespace + '/command', '--receipt-timeout', '.5',
                    '--state-timeout', '2', '--connect-timeout', '2']
        return subprocess.run(command, stdout=subprocess.PIPE, stderr=subprocess.PIPE,
                              text=True, timeout=10.)

    def test_cli_status_set_replay_and_exit_codes(self):
        status = self.run_cli('status')
        self.assertEqual(status.returncode, 0, status.stderr)
        self.assertEqual(json.loads(status.stdout)['phase'], 'IDLE')
        path = os.path.join(self.directory.name, 'cli-request.json')
        sent = self.run_cli('set', '--latitude', '1', '--longitude', '2', '--save-request', path)
        self.assertEqual(sent.returncode, 0, sent.stderr)
        self.assertTrue(json.loads(sent.stdout)['accepted'])
        replay = self.run_cli('replay', path)
        self.assertEqual(replay.returncode, 0, replay.stderr)
        self.assertTrue(json.loads(replay.stdout)['duplicate'])
        self.server.mode = 'reject'
        rejected = self.run_cli('pause')
        self.assertEqual(rejected.returncode, 1, rejected.stderr)
        self.server.mode = 'drop'
        unknown = self.run_cli('pause')
        self.assertEqual(unknown.returncode, 2, unknown.stderr)
        self.assertIn('"published": true', unknown.stderr)
        invalid = self.run_cli('set', '--latitude', '91', '--longitude', '2')
        self.assertEqual(invalid.returncode, 2)

    def test_cli_missing_master_exits_within_timeout(self):
        # Reserve a loopback port without listening so no ROS master is reachable.
        with socket.socket() as reserved:
            reserved.bind(('127.0.0.1', 0))
            environment = os.environ.copy()
            environment['ROS_MASTER_URI'] = 'http://127.0.0.1:' + str(reserved.getsockname()[1])
            environment['ROS_IP'] = '127.0.0.1'
            environment.pop('ROS_HOSTNAME', None)
            started = time.monotonic()
            result = subprocess.run([sys.executable, '-m', 'ag_client.cli', 'status',
                                     '--state-timeout', '.3'], env=environment,
                                    stdout=subprocess.PIPE, stderr=subprocess.PIPE,
                                    text=True, timeout=5.)
        self.assertEqual(result.returncode, 2, result.stderr)
        self.assertLess(time.monotonic() - started, 4.)
        self.assertIn('timed out', result.stderr)

    def test_stale_worker_blocks_routes_but_allows_stop_commands(self):
        self.client.set_task(point(1., 2.))
        with self.server.lock:
            self.server.worker_stalled = True
            self.server.state['execution_updated_stamp'] = rospy.Time.now().to_sec() - 10.
        until(lambda: not self.client.health()['execution_healthy'])
        for operation, kwargs in [('SET_TASK', {'final_goal': point(3., 4.)}),
                                  ('REPLACE_VIA_POINTS', {'via_points': []}), ('RESUME', {})]:
            with self.assertRaises(ClientError) as caught:
                self.client.command(operation, **kwargs)
            self.assertFalse(caught.exception.published)
        self.client.pause()
        self.client.cancel()
        self.assertEqual([msg.operation for msg in self.server.commands], ['SET_TASK', 'PAUSE', 'CANCEL'])

    def test_startup_and_cancel_barrier_block_new_routes(self):
        for key, value in [('startup_checked', False), ('cancellation_pending', True)]:
            with self.server.lock:
                self.server.state.update(startup_checked=True, cancellation_pending=False)
                self.server.state[key] = value
            until(lambda: self.client.status().get(key) == value)
            with self.assertRaises(ClientError):
                self.client.set_task(point(1., 2.))
        self.assertEqual(self.server.commands, [])

    def test_replay_ignores_unchanged_old_transport_error(self):
        request = build_request(self.client.status(), 'SET_TASK', point(1., 2.))
        self.server.mode = 'drop'
        with self.server.lock:
            self.server.state['transport_error'] = dict(request_id=request['request_id'],
                                                       reason='old queue full', stamp=time.time()-10.)
        until(lambda: self.client.status().get('transport_error') is not None)
        started = time.monotonic()
        with self.assertRaises(OutcomeUnknown) as caught:
            self.client.replay(request)
        self.assertIn('Receipt timeout', str(caught.exception))
        self.assertGreaterEqual(time.monotonic()-started, .45)
        self.assertEqual(len(self.server.commands), 1)

    def test_new_transport_error_is_reported_after_publish(self):
        self.server.mode = 'drop'

        def error_after_command():
            until(lambda: bool(self.server.commands))
            with self.server.lock:
                self.server.state['transport_error'] = dict(request_id=self.server.commands[0].request_id,
                                                           reason='new queue full', stamp=time.time())

        worker = threading.Thread(target=error_after_command)
        worker.start()
        try:
            with self.assertRaises(OutcomeUnknown) as caught:
                self.client.set_task(point(1., 2.))
            self.assertIn('new queue full', str(caught.exception))
        finally:
            worker.join(3.)

    def test_cli_health_and_task_status_are_readonly(self):
        health = self.run_cli('health')
        self.assertEqual(health.returncode, 0, health.stderr)
        self.assertTrue(json.loads(health.stdout)['execution_healthy'])
        report = self.run_cli('task-status', 'missing-task')
        self.assertEqual(report.returncode, 0, report.stderr)
        self.assertEqual(json.loads(report.stdout)['status'], 'UNKNOWN')
        self.assertEqual(self.server.commands, [])


if __name__ == '__main__':
    rospy.init_node('ag_client_ros_test', anonymous=True)
    rostest.rosrun('ag_client', 'ag_client_ros', ClientRosTests)
