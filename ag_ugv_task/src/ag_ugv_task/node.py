"""Bridge V2 route edits to the UGV's existing single-Action executor."""
import importlib
import json
from pathlib import Path
import queue
import sys
import time

import roslib.packages
import rospkg
import rospy
from std_msgs.msg import String

from ag_uav_client.v2 import COMMAND, STATE, decode, number, snapshot, vehicle_position

from .executor import TaskExecutor
from .navigation import RosNavigation
from .route import Route


def load_vehicle_modules():
    """Find only the installed car geographic modules, not its task interface."""
    share = Path(rospkg.RosPack().get_path('ugv_navigation'))
    candidates = [share / 'scripts']
    candidates.extend(Path(path).resolve().parent for path in
                      (roslib.packages.find_resource('ugv_navigation', 'geographic.py')))
    for directory in candidates:
        if all((directory / (name + '.py')).is_file() for name in (
                'geographic', 'geographic_goal_core', 'geographic_core')):
            sys.path.insert(0, str(directory))
            return tuple(importlib.import_module(name) for name in
                         ('geographic', 'geographic_goal_core', 'geographic_core'))
    raise RuntimeError('Source the original UGV navigation workspace with its geographic modules installed')


class GroundNode:
    def __init__(self, backend, executor_type, position, executor_options=None):
        self.backend, self.position = backend, position
        self.route = Route()
        self.executor = executor_type(self.route, backend, **(executor_options or {}))
        self.last_command = None
        self.inbox = queue.Queue(maxsize=64)
        self.publisher = rospy.Publisher(STATE, String, queue_size=1, latch=True)
        self.subscriber = rospy.Subscriber(COMMAND, String, self.receive, queue_size=64)
        self.last_signature = None
        self.published = float('-inf')

    def receive(self, message):
        try:
            self.inbox.put_nowait((message.data, rospy.Time.now().to_sec()))
        except queue.Full:
            rospy.logwarn_throttle(2, 'V2 task command queue is full')

    def process(self, raw, received_at):
        command = {}
        try:
            command = decode(raw)
            result = self.executor.command(command)
        except ValueError as exc:
            result = dict(accepted=False, reason=str(exc))
        self.last_command = {
            key: command.get(key) if isinstance(command.get(key), str) else None
            for key in ('task_id', 'op')
        }
        seq, stamp = command.get('seq'), command.get('sent_at')
        self.last_command.update(seq=seq if type(seq) is int else None,
                                 sent_at=stamp if number(stamp) else None,
                                 received_at=received_at, accepted=result['accepted'],
                                 reason=result.get('reason') or None)
        self.publish_if_changed()

    def publish_if_changed(self):
        exec_ = self.executor
        active = exec_.active[0]['public'] if exec_.active else None
        phase = self.route.phase
        reason = self.route.reason or None
        if phase in ('CANCELING', 'CANCELED'):
            phase = {'CANCELING': 'STOPPING', 'CANCELED': 'STOPPED'}[phase]
        elif phase == 'READY' and active:
            phase = 'SWITCHING'
        if self.route.phase == 'READY' and not active:
            status = self.backend.active_server_goal()
            if status is not False:
                reason = 'WAITING_NAVIGATION' if status is None else 'NAVIGATION_BUSY'

        summary = dict(task_id=self.route.task_id, phase=phase, reason=reason,
                       active=active, mvps=self.route.mvps, target=self.route.target,
                       last_command=self.last_command)
        signature = json.dumps(summary, sort_keys=True)
        now = time.monotonic()
        if signature != self.last_signature or now - self.published >= 1.0:
            data = snapshot('ugv', rospy.Time.now().to_sec(), self.position(), **summary)
            self.publisher.publish(String(data=json.dumps(data, ensure_ascii=False, allow_nan=False)))
            self.last_signature, self.published = signature, now

    def run(self):
        try:
            while not rospy.is_shutdown():
                for _ in range(16):
                    try:
                        raw, stamp = self.inbox.get_nowait()
                    except queue.Empty:
                        break
                    self.process(raw, stamp)
                # Don't touch another goal when starting alongside an existing navigator.
                if (self.route.phase != 'READY' or self.executor.active is not None
                        or self.backend.active_server_goal() is False):
                    self.executor.tick()
                self.publish_if_changed()
                time.sleep(.05)
        finally:
            self.executor.shutdown()


def main():
    rospy.init_node('ag_ugv_task')
    geographic_client, geographic, coordinates = load_vehicle_modules()
    backend = RosNavigation(rospy, rospy.get_param('~action', '/ugv/move_base'),
                            geographic_client.GeographicClient, geographic)

    def current_position():
        try:
            status = backend.status()
            obs = status['latest_observation']
            stamp = obs['stamp']
            age = rospy.Time.now().to_sec() - stamp
            if (obs['fresh_rtk_usable'] is not True or obs['gga_quality'] != 4
                    or not 0 <= age <= backend.geo.fix_age):
                return vehicle_position()
            x, y = obs['base_xy']
            origin = geographic.origin_record(status)
            lat, lon, _ = coordinates.enu_to_geodetic([x, y, 0.], origin)
            if number(lat) and number(lon):
                return vehicle_position(lat, lon, stamp)
        except (TypeError, KeyError, ValueError, ArithmeticError):
            pass
        return vehicle_position()

    options = {key: rospy.get_param('~' + key, default) for key, default in (
        ('via_pass_radius_m', .25), ('segment_timeout_s', 120.),
        ('cancel_timeout_s', 3.), ('stop_timeout_s', 8.))}
    GroundNode(backend, TaskExecutor, current_position, options).run()
