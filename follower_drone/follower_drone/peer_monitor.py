"""
Peer monitor — reads raw MAVLink from mavlink-router to extract
remote drone telemetry arriving over the RFD9000 radio mesh.

On follower: filters for SYSID=1 (leader).
Uses pymavlink directly because MAVROS2 ignores foreign SYSIDs.
"""

import time
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import TwistStamped
from std_msgs.msg import Float64
from pymavlink import mavutil

SENSOR_QOS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE,
    history=HistoryPolicy.KEEP_LAST,
    depth=5,
)


class PeerMonitor(Node):
    def __init__(self):
        super().__init__('peer_monitor')

        self.declare_parameter('peer_sysid', 1)
        self.declare_parameter('mavlink_udp_port', 14551)
        self.declare_parameter('timeout_s', 5.0)

        self.peer_sysid = self.get_parameter('peer_sysid').value
        port = self.get_parameter('mavlink_udp_port').value
        self.timeout_s = self.get_parameter('timeout_s').value

        # pymavlink UDP — listen for data pushed by mavlink-router
        self.mav = mavutil.mavlink_connection(
            f'udpin:0.0.0.0:{port}',
            source_system=254,
        )

        self.pub_position = self.create_publisher(NavSatFix, 'peer/global_position', SENSOR_QOS)
        self.pub_velocity = self.create_publisher(TwistStamped, 'peer/velocity', SENSOR_QOS)
        self.pub_heartbeat_age = self.create_publisher(Float64, 'peer/heartbeat_age', SENSOR_QOS)

        self.last_peer_time = 0.0

        # Drain MAVLink buffer at 50Hz
        self.create_timer(0.02, self._tick)
        self.get_logger().info(
            f'PeerMonitor started — watching SYSID={self.peer_sysid} on UDP:{port}')

    def _tick(self):
        last_pos = None

        while True:
            msg = self.mav.recv_match(blocking=False)
            if msg is None:
                break
            if msg.get_srcSystem() != self.peer_sysid:
                continue
            msg_type = msg.get_type()
            if msg_type == 'GLOBAL_POSITION_INT':
                last_pos = msg
                self.last_peer_time = time.time()
            elif msg_type == 'HEARTBEAT':
                self.last_peer_time = time.time()

        if last_pos is not None:
            now_stamp = self.get_clock().now().to_msg()

            fix = NavSatFix()
            fix.header.stamp = now_stamp
            fix.header.frame_id = 'earth'
            fix.latitude = last_pos.lat / 1e7
            fix.longitude = last_pos.lon / 1e7
            fix.altitude = last_pos.relative_alt / 1000.0
            self.pub_position.publish(fix)

            vel = TwistStamped()
            vel.header.stamp = now_stamp
            vel.twist.linear.x = last_pos.vx / 100.0
            vel.twist.linear.y = last_pos.vy / 100.0
            vel.twist.linear.z = last_pos.vz / 100.0
            self.pub_velocity.publish(vel)

        # Heartbeat age
        age = Float64()
        age.data = (time.time() - self.last_peer_time) if self.last_peer_time > 0 else -1.0
        self.pub_heartbeat_age.publish(age)


def main(args=None):
    rclpy.init(args=args)
    node = PeerMonitor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
