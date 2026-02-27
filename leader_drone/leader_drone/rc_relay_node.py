"""
RC relay node — reads RC input from MAVROS2 and, when the mode switch
indicates "follower control", sends RC_CHANNELS_OVERRIDE to the
follower's CubeOrange+ via mavlink-router → RFD9000 radio.

Channel mapping (configurable via YAML):
  CH1-4: AETR sticks (Aileron, Elevator, Throttle, Rudder)
  CH7:   Leader/Follower selector switch
         < threshold = controlling leader (default)
         > threshold = controlling follower (RC override active)
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from mavros_msgs.msg import RCIn
from pymavlink import mavutil

SENSOR_QOS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE,
    history=HistoryPolicy.KEEP_LAST,
    depth=5,
)


class RCRelayNode(Node):
    def __init__(self):
        super().__init__('rc_relay_node')

        self.declare_parameter('follower_sysid', 2)
        self.declare_parameter('follower_compid', 1)
        self.declare_parameter('switch_channel', 7)       # 1-indexed
        self.declare_parameter('switch_threshold', 1500)
        self.declare_parameter('mavlink_udp_port', 14551)
        self.declare_parameter('override_rate_hz', 10.0)

        self.follower_sysid = self.get_parameter('follower_sysid').value
        self.follower_compid = self.get_parameter('follower_compid').value
        self.switch_ch = self.get_parameter('switch_channel').value - 1  # 0-indexed
        self.switch_threshold = self.get_parameter('switch_threshold').value
        port = self.get_parameter('mavlink_udp_port').value
        rate = self.get_parameter('override_rate_hz').value

        # pymavlink — send overrides to mavlink-router
        self.mav = mavutil.mavlink_connection(
            f'udpout:127.0.0.1:{port}',
            source_system=254,
            source_component=99,
        )

        self.latest_channels = [0] * 18
        self.follower_active = False
        self.was_active = False

        self.create_subscription(RCIn, 'mavros/rc/in', self._rc_cb, SENSOR_QOS)
        self.create_timer(1.0 / rate, self._send_override)

        self.get_logger().info(
            f'RCRelay started — switch CH{self.switch_ch + 1}, '
            f'target SYSID={self.follower_sysid}')

    def _rc_cb(self, msg: RCIn):
        self.latest_channels = list(msg.channels)
        if len(self.latest_channels) > self.switch_ch:
            self.follower_active = (
                self.latest_channels[self.switch_ch] > self.switch_threshold
            )

    def _send_override(self):
        if not self.follower_active:
            if self.was_active:
                # Release override — send all zeros once
                self.mav.mav.rc_channels_override_send(
                    self.follower_sysid, self.follower_compid,
                    0, 0, 0, 0, 0, 0, 0, 0,
                )
                self.was_active = False
                self.get_logger().info('RC override released — back to leader')
            return

        if not self.was_active:
            self.get_logger().info('RC override active — controlling follower')
            self.was_active = True

        # Pass through CH1-4 (AETR sticks), CHAN_NOCHANGE for the rest
        NOCHANGE = 65535
        ch = [NOCHANGE] * 8
        for i in range(min(4, len(self.latest_channels))):
            ch[i] = self.latest_channels[i]

        self.mav.mav.rc_channels_override_send(
            self.follower_sysid, self.follower_compid,
            ch[0], ch[1], ch[2], ch[3], ch[4], ch[5], ch[6], ch[7],
        )


def main(args=None):
    rclpy.init(args=args)
    node = RCRelayNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
