"""One-shot, non-latched commands with fresh, correlated receipts."""
import copy
import fcntl
import hashlib
import math
import os
import tempfile
import threading
import time
from contextlib import contextmanager

import rospy
from std_msgs.msg import String
from slicer_air_ground_msgs.msg import Command, GeoPoint

from .protocol import (ProtocolError, loads_json, validate_state, validate_request,
                       build_request, save_request)


class ClientError(RuntimeError):
    """Local failure before publication; no command was sent."""
    published = False


class OutcomeUnknown(ClientError):
    """Publication occurred, but no definitive receipt was obtained."""
    published = True

    def __init__(self, message, request):
        super().__init__(message)
        self.request = copy.deepcopy(request)


class CommandRejected(ClientError):
    published = True

    def __init__(self, receipt, request):
        super().__init__(receipt.get('reason', 'Command rejected'))
        self.receipt = copy.deepcopy(receipt)
        self.request = copy.deepcopy(request)


def to_message(request):
    request = validate_request(request)

    def convert(value):
        msg = GeoPoint()
        if value is not None:
            msg.latitude, msg.longitude = value['latitude'], value['longitude']
            for field, flag in [('altitude_ellipsoid_m', 'has_altitude'), ('yaw', 'has_yaw')]:
                if value[field] is not None:
                    setattr(msg, flag, True)
                    setattr(msg, field, value[field])
        return msg

    msg = Command()
    for field in ('protocol_version', 'server_id', 'sequence', 'request_id', 'task_id', 'operation'):
        setattr(msg, field, request[field])
    msg.final_goal = convert(request['final_goal'])
    msg.via_points = [convert(p) for p in request['via_points']]
    return msg


class AirGroundClient:
    """Call after rospy.init_node(). One logical command source across the fleet.

    Timeout limits use monotonic wall time, even when ROS /clock stops.
    This client never initializes flight, navigation, or control nodes.
    """

    def __init__(self, command_topic='/air_ground/command', state_topic='/air_ground/state',
                 state_timeout=5., state_max_age=2., connect_timeout=3., receipt_timeout=3.):
        for name, value in locals().copy().items():
            if name.endswith('timeout') or name == 'state_max_age':
                if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value) or value <= 0:
                    raise ValueError(name + ' must be finite and positive')
        self.state_timeout, self.state_max_age = state_timeout, state_max_age
        self.connect_timeout, self.receipt_timeout = connect_timeout, receipt_timeout
        self._condition = threading.Condition()
        self._sending = threading.Lock()
        self._state = None
        self._received = 0.
        self._error = 'No state received'
        self._closed = False
        self._publisher = None
        self._command_topic = rospy.resolve_name(command_topic)
        self._state_topic = rospy.resolve_name(state_topic)
        identity = os.environ.get('ROS_MASTER_URI', '') + '\n' + self._command_topic
        self._lock_path = os.path.join(tempfile.gettempdir(), 'ag_client-' + str(os.getuid()) + '-' +
                                      hashlib.sha256(identity.encode()).hexdigest() + '.lock')
        self._subscriber = rospy.Subscriber(self._state_topic, String, self._callback, queue_size=1)

    def _callback(self, message):
        with self._condition:
            try:
                self._state = validate_state(loads_json(message.data))
                self._received = time.monotonic()
                self._error = ''
            except (ValueError, TypeError, KeyError) as exc:
                self._state = None
                self._error = 'Invalid state: ' + str(exc)
            self._condition.notify_all()

    def _fresh(self):
        if self._state is None:
            return False
        age = rospy.Time.now().to_sec() - self._state['updated_stamp']
        return 0 <= age <= self.state_max_age and time.monotonic() - self._received <= self.state_max_age

    def _check_open(self):
        if self._closed or rospy.is_shutdown():
            raise ClientError('Client closed or ROS shut down')

    def status(self):
        deadline = time.monotonic() + self.state_timeout
        with self._condition:
            while True:
                self._check_open()
                if self._fresh():
                    return copy.deepcopy(self._state)
                left = deadline - time.monotonic()
                if left <= 0:
                    raise ClientError('No fresh state: ' + (self._error or 'stale/future ROS timestamp; check clocks'))
                self._condition.wait(min(left, .05))

    @contextmanager
    def _serial(self):
        if not self._sending.acquire(False):
            raise ClientError('Another command is in progress in this client')
        fd = None
        try:
            fd = os.open(self._lock_path, os.O_CREAT | os.O_RDWR | os.O_NOFOLLOW, 0o600)
            try:
                fcntl.flock(fd, fcntl.LOCK_EX | fcntl.LOCK_NB)
            except BlockingIOError as exc:
                raise ClientError('Another local client is sending on this master/topic') from exc
            yield
        finally:
            if fd is not None:
                os.close(fd)
            self._sending.release()

    def command(self, operation, final_goal=None, via_points=None, save_path=None):
        """Create one request from fresh state and publish once. Returns its receipt."""
        with self._serial():
            state = self.status()
            request = build_request(state, operation, final_goal=final_goal, via_points=via_points)
            return self._publish(request, save_path, replay=False)

    def replay(self, request):
        """Replay an unmodified saved request; no new IDs, automatic retry or cancel."""
        with self._serial():
            return self._publish(validate_request(request), None, replay=True)

    def _publish(self, request, save_path, replay=False):
        message = to_message(request)
        self._check_open()
        if self._publisher is None:
            self._publisher = rospy.Publisher(self._command_topic, Command, queue_size=1, latch=False)
        deadline = time.monotonic() + self.connect_timeout
        while self._publisher.get_num_connections() == 0:
            self._check_open()
            if time.monotonic() >= deadline:
                raise ClientError('No command subscriber; request was not published')
            with self._condition:
                self._condition.wait(.02)
        # Recheck boot and watermark after establishing the transport connection.
        state = self.status()
        if state['server_id'] != request['server_id']:
            raise ClientError('Receiver restarted; saved request belongs to another boot')
        if not replay and (state['last_sequence'] or 0) >= request['sequence']:
            raise ClientError('Sequence changed while connecting; check other command senders')
        # Replays may legitimately have old sequence values; receiver owns deduplication.
        if save_path is not None:
            save_request(request, save_path)
        with self._condition:
            self._check_open()
            baseline = self._received
            try:
                self._publisher.publish(message)
            except Exception as exc:
                raise OutcomeUnknown('Publish failed; delivery cannot be ruled out: ' + str(exc), request) from exc
            deadline = time.monotonic() + self.receipt_timeout
            while True:
                if self._closed or rospy.is_shutdown():
                    raise OutcomeUnknown('Stopped after publishing; check vehicle state', request)
                if self._received > baseline and self._fresh():
                    state = self._state
                    if state['server_id'] != request['server_id']:
                        raise OutcomeUnknown('Receiver restarted after publication', request)
                    receipt = state.get('last_command')
                    if receipt and all(receipt.get(key) == request[key] for key in
                                       ('request_id', 'sequence', 'task_id', 'operation')):
                        if type(receipt.get('accepted')) is bool:
                            if not receipt['accepted']:
                                raise CommandRejected(receipt, request)
                            return copy.deepcopy(receipt)
                    error = state.get('transport_error')
                    if isinstance(error, dict) and error.get('request_id') == request['request_id']:
                        raise OutcomeUnknown('Receiver transport error: ' + str(error.get('reason')), request)
                left = deadline - time.monotonic()
                if left <= 0:
                    raise OutcomeUnknown('Receipt timeout; command may already be executing. Check status, then replay the saved request if needed', request)
                self._condition.wait(min(left, .05))

    def set_task(self, final_goal, via_points=None, save_path=None):
        return self.command('SET_TASK', final_goal, via_points, save_path)

    def replace_via_points(self, via_points, save_path=None):
        return self.command('REPLACE_VIA_POINTS', via_points=via_points, save_path=save_path)

    def pause(self, save_path=None):
        return self.command('PAUSE', save_path=save_path)

    def resume(self, save_path=None):
        return self.command('RESUME', save_path=save_path)

    def cancel(self, save_path=None):
        return self.command('CANCEL', save_path=save_path)

    def close(self):
        with self._condition:
            self._closed = True
            self._condition.notify_all()
        self._subscriber.unregister()
        if self._publisher is not None:
            self._publisher.unregister()

    def __enter__(self):
        return self

    def __exit__(self, *_):
        self.close()
