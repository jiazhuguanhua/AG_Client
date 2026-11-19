"""Shared ROS startup and V2 local intent publication."""
import json
import signal
import time

from .v2 import INTENT, validate


class _InitializationTimeout(BaseException):
    # rospy catches Exception during registration.
    pass


def init_ros(rospy, timeout=5., name='ag_client'):
    def expired(*_):
        raise _InitializationTimeout()

    old = signal.signal(signal.SIGALRM, expired)
    signal.setitimer(signal.ITIMER_REAL, timeout)
    try:
        rospy.init_node(name, anonymous=True)
    except _InitializationTimeout:
        raise TimeoutError('ROS initialization timed out; check ROS_MASTER_URI and ROS_IP') from None
    finally:
        signal.setitimer(signal.ITIMER_REAL, 0)
        signal.signal(signal.SIGALRM, old)


def publish_intent(rospy, publisher, intent):
    """Submit to the local gateway. Return does not imply UGV acceptance."""
    payload = json.dumps(validate(intent, wire=False), allow_nan=False)
    deadline = time.monotonic() + 3.
    while not publisher.get_num_connections():
        if rospy.is_shutdown() or time.monotonic() >= deadline:
            raise TimeoutError(f'{INTENT} 没有连接到 UAV gateway，请先启动 uav.launch')
        time.sleep(.05)
    from std_msgs.msg import String
    publisher.publish(String(data=payload))
