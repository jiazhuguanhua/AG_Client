#!/usr/bin/env python3
"""Stamp local route intents and publish UAV telemetry on the V2 link."""
import json
import threading
import time

import rospy
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import String

from ag_uav_client.v2 import (
    COMMAND, INTENT, UAV_STATE, coordinates, decode, snapshot, validate, vehicle_position,
)


class UavGateway:
    def __init__(self):
        self.lock = threading.Lock()
        self.sequence = 0
        self.last_command = None
        self.fix = None
        self.flight_phase = None
        self.command = rospy.Publisher(COMMAND, String, queue_size=32, latch=False)
        self.state = rospy.Publisher(UAV_STATE, String, queue_size=1, latch=True)
        self.inputs = [
            rospy.Subscriber(INTENT, String, self.send, queue_size=32),
            rospy.Subscriber(rospy.get_param('~fix_topic', '/uav/fix'), NavSatFix, self.on_fix, queue_size=1),
            rospy.Subscriber(rospy.get_param('~phase_topic', '/uav/phase'), String, self.on_phase, queue_size=1),
        ]
        self.timer = rospy.Timer(rospy.Duration(1), self.publish_state)

    def on_fix(self, fix):
        position = vehicle_position()
        if fix.status.status >= 0 and not fix.header.stamp.is_zero():
            try:
                location = coordinates(dict(lat=fix.latitude, lon=fix.longitude))
                position = vehicle_position(location['lat'], location['lon'], fix.header.stamp.to_sec())
            except ValueError:
                pass
        with self.lock:
            self.fix = position

    def on_phase(self, message):
        with self.lock:
            self.flight_phase = (message.data.strip() or None, time.monotonic())

    def send(self, message):
        try:
            command = validate(decode(message.data), wire=False)
            with self.lock:
                if not self.command.get_num_connections():
                    rospy.logwarn('V2 intent not sent: no subscriber on %s', COMMAND)
                    return
                self.sequence = max(self.sequence + 1, time.time_ns() // 1000)
                command.update(version=2, seq=self.sequence, sent_at=rospy.Time.now().to_sec())
                self.command.publish(String(data=json.dumps(validate(command), allow_nan=False)))
                self.last_command = {key: command[key] for key in ('task_id', 'op', 'seq', 'sent_at')}
                self.last_command.update(accepted=None, received_at=None, reason=None)
                self._publish_state()
        except ValueError as exc:
            rospy.logwarn('Invalid V2 intent: %s', exc)

    def _publish_state(self):
        now = rospy.Time.now().to_sec()
        position = self.fix
        if not position or not position['valid'] or not 0 <= now - position['measured_at'] <= 2.0:
            position = vehicle_position()
        phase = self.flight_phase
        phase = phase[0] if phase and time.monotonic() - phase[1] <= 2.0 else None
        last = self.last_command
        data = snapshot('uav', now, position, phase=phase, task_id=last['task_id'] if last else None,
                        last_command=last)
        self.state.publish(String(data=json.dumps(data, ensure_ascii=False, allow_nan=False)))

    def publish_state(self, _event):
        with self.lock:
            self._publish_state()


if __name__ == '__main__':
    rospy.init_node('ag_uav_gateway')
    gateway = UavGateway()
    rospy.spin()
