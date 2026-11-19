"""Adapter for the installed Slicer Noetic geographic navigation and Mux.

The Action and hold behavior is carried over from the supplied UGV package.
"""
import json
import subprocess
import time
from types import SimpleNamespace


class RosNavigation:
    """Reuse the field-proven converter, observation windows and Mux CLI."""
    def __init__(self, ros, action, client_type, geometry):
        from std_msgs.msg import String
        from actionlib_msgs.msg import GoalStatusArray
        self.ros = ros
        self.geometry = geometry
        self.geo = client_type(SimpleNamespace(action=action, command='status', timeout=120.))
        self.mux_sample = None
        self.mux_sub = ros.Subscriber('/cup_nav/manual_state', String, self._mux, queue_size=1)
        self.last_action_state = None
        self.status_sample = None
        self.action_status_sub = ros.Subscriber(action+'/status', GoalStatusArray, self._action_status, queue_size=1)

    def _action_status(self, message):
        if (getattr(message, '_connection_header', {}) or {}).get('callerid') != self.geo.args.action:
            return
        age = (self.ros.Time.now()-message.header.stamp).to_sec()
        if 0 <= age <= .5:
            self.status_sample = time.monotonic(), list(message.status_list)

    def _mux(self, message):
        header = getattr(message, '_connection_header', {}) or {}
        if header.get('callerid') != '/slicer_velocity_mux' or header.get('latching', '0') != '0':
            return
        try:
            value = json.loads(message.data)
            if any(type(value[k]) is not bool for k in ('manual_hold', 'emergency_stop_latched', 'teleop_fresh')):
                return
            if type(value['takeover_generation']) is not int or not isinstance(value['mux_instance_id'], str):
                return
            self.mux_sample = time.monotonic(), value
        except (KeyError, ValueError, TypeError):
            return

    def mux_state(self):
        sample = self.mux_sample
        if sample is None or not 0 <= time.monotonic()-sample[0] <= .5:
            raise ValueError('existing Mux state unavailable/stale')
        return sample[1]

    def status(self, generation=None):
        status, received, _ = self.geo.observation()
        problem = self.geometry.status_problem(status, received, time.monotonic(), self.ros.Time.now().to_sec(),
                                 generation=generation, max_age=self.geo.status_age)
        if problem:
            raise ValueError(problem)
        return status

    def health(self, target=None):
        try:
            self.status(target['generation'] if target else None)
            return None
        except ValueError as error:
            return str(error)

    def _convert(self, point, status, pose):
        height = point.get('altitude_ellipsoid_m')
        if height is None:
            height = self.geometry.origin_record(status)['altitude_ellipsoid_m']
        args = SimpleNamespace(latitude=point['latitude'], longitude=point['longitude'],
                               altitude_ellipsoid=height, yaw=point.get('yaw'), target_file=None,
                               east=None, north=None, up=0., origin_file=None, session_enu=False)
        target = self.geometry.resolve_target(args, status, pose)
        target.update(generation=status['generation'], stamp=self.ros.Time.now().to_sec(),
                      requested_wgs84=dict(point),
                      height_policy=('explicit ellipsoid height; 2D navigation' if point.get('altitude_ellipsoid_m')
                                     is not None else 'task-origin ellipsoid height; 2D Up=0; not measured base height'))
        return target

    def validate_route(self, final, vias):
        self.geo.frame_check('map')
        status, pose = self.status(), self.geo.current_pose('map')
        for point in vias+[final]:
            self._convert(point, status, pose)

    def target(self, point):
        target = self._convert(point, self.status(), self.geo.current_pose('map'))
        self.ensure_idle()  # A known pre-dispatch failure must not create a phantom owned Action.
        return target

    def position(self):
        return self.geo.current_pose('map')[:2]

    def active_server_goal(self):
        sample = self.status_sample
        if sample is None or not 0 <= time.monotonic()-sample[0] <= .5:
            return None
        return any(row.status in (0, 1, 6, 7) for row in sample[1])

    def ensure_idle(self):
        if not self.geo.client.wait_for_server(self.ros.Duration(.2)):
            raise ValueError('existing move_base Action unavailable')
        # Reuse the existing control client's idle-status criterion. Do not
        # preempt a competing legacy goal merely by submitting our first goal.
        deadline = time.monotonic()+1.
        while True:
            if self.active_server_goal() is False:
                break
            if time.monotonic() >= deadline:
                raise ValueError('fresh idle move_base status unavailable; stop other goal senders')
            time.sleep(.02)

    def start(self, target):
        self.geo.client.send_goal(self.geo.MoveBaseGoal(
            target_pose=self.geo.pose(target['map_xyz'], target['yaw'], target['frame'])))

    def poll(self):
        return dict(state=self.geo.client.get_state(), text=self.geo.client.get_goal_status_text(),
                    observed_stamp=self.ros.Time.now().to_sec())

    def cancel(self):
        self.geo.client.cancel_goal()

    def _control(self, operation, expected_hold=None):
        command = ['rosrun', 'ugv_slicer_control', 'control.py', operation]
        if expected_hold is not None:
            command += ['--expected-mux-instance', str(expected_hold[0]),
                        '--expected-generation', str(expected_hold[1])]
        process = subprocess.run(command,
                                 text=True, capture_output=True, timeout=12.)
        for line in reversed(process.stdout.splitlines()):
            try:
                result = json.loads(line)
                if isinstance(result, dict) and 'manual_hold' in result:
                    # control.py prints its confirmed Mux ACK before separately
                    # waiting for cancel-all. Keep that hold ownership even if
                    # the subsequent Action cancel ACK times out. The executor
                    # still waits for its owned Action; this is not a stop ACK.
                    if process.returncode == 0 or (operation == 'hold' and result.get('manual_hold')
                                                   and result.get('last_intent_accepted')):
                        return dict(result, action_cancel_note=process.stderr.strip())
            except ValueError:
                pass
        raise ValueError('existing Mux '+operation+' ACK unavailable: '+process.stderr.strip())

    def hold(self):
        return self._control('hold')

    def release(self, expected_hold):
        return self._control('release', expected_hold)

    def stopped(self, since):
        if self.active_server_goal() is not False:
            return dict(confirmed=False, reason='fresh global Action termination ACK unavailable')
        _, _, rows = self.geo.observation()
        return self.geometry.quiet_odom_tail([r for r in rows if r['received'] >= since], time.monotonic(),
                               self.ros.Time.now().to_sec(), window=self.geo.stop_window)

    def review(self, target, action, since):
        status, received, _ = self.geo.observation()
        return self.geometry.arrival_review(target, status, received, time.monotonic(), self.ros.Time.now().to_sec(),
                              target['generation'], action['state'], self.stopped(since),
                              self.geo.tolerance, self.geo.fix_age, self.geo.status_age)
