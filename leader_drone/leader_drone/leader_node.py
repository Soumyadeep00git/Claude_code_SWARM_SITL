"""
Leader node — monitors own state and applies collision avoidance overlay
when the follower is too close. Otherwise, the leader flies under
Mission Planner control (MAVROS2 passes through MP commands directly).

Uses compute_escape() for simple radial push-away during PROX_AVOID.
"""

import time
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import TwistStamped
from std_msgs.msg import String

from follower_drone.guidance import GuidanceState, compute_escape

SENSOR_QOS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE,
    history=HistoryPolicy.KEEP_LAST,
    depth=5,
)


class LeaderNode(Node):
    def __init__(self):
        super().__init__('leader_node')

        self.declare_parameter('control_rate_hz', 10.0)
        self.declare_parameter('safety_dist_m', 5.0)
        self.declare_parameter('escape_speed', 3.0)
        self.declare_parameter('max_accel', 4.0)

        rate = self.get_parameter('control_rate_hz').value
        self.safety_dist = self.get_parameter('safety_dist_m').value
        self.escape_speed = self.get_parameter('escape_speed').value
        self.max_accel = self.get_parameter('max_accel').value

        self.escape_state = GuidanceState()

        # Own state
        self.my_lat = 0.0
        self.my_lon = 0.0
        self.my_alt = 0.0
        self.my_vn = 0.0
        self.my_ve = 0.0

        # Peer state
        self.peer_lat = 0.0
        self.peer_lon = 0.0
        self.peer_alt = 0.0
        self.peer_vn = 0.0
        self.peer_ve = 0.0

        # Failsafe state
        self.failsafe_state = 'PREFLIGHT'

        # Subscribers — own telemetry from MAVROS2
        self.create_subscription(
            NavSatFix, 'mavros/global_position/global',
            self._own_gps_cb, SENSOR_QOS)
        self.create_subscription(
            TwistStamped, 'mavros/local_position/velocity_body',
            self._own_vel_cb, SENSOR_QOS)

        # Subscribers — peer from peer_monitor
        self.create_subscription(
            NavSatFix, 'peer/global_position',
            self._peer_gps_cb, SENSOR_QOS)
        self.create_subscription(
            TwistStamped, 'peer/velocity',
            self._peer_vel_cb, SENSOR_QOS)

        # Subscriber — failsafe state
        self.create_subscription(String, 'failsafe/state', self._failsafe_cb, 10)

        # Publishers
        self.pub_cmd_vel = self.create_publisher(
            TwistStamped, 'mavros/setpoint_velocity/cmd_vel', 10)
        self.pub_status = self.create_publisher(
            String, 'escape_status', 10)

        self.create_timer(1.0 / rate, self._control_tick)
        self.get_logger().info('LeaderNode started (escape safety overlay)')

    # ── Callbacks ───────────────────────────────────────────

    def _own_gps_cb(self, msg: NavSatFix):
        self.my_lat = msg.latitude
        self.my_lon = msg.longitude
        self.my_alt = msg.altitude

    def _own_vel_cb(self, msg: TwistStamped):
        self.my_vn = msg.twist.linear.x
        self.my_ve = msg.twist.linear.y

    def _peer_gps_cb(self, msg: NavSatFix):
        self.peer_lat = msg.latitude
        self.peer_lon = msg.longitude
        self.peer_alt = msg.altitude

    def _peer_vel_cb(self, msg: TwistStamped):
        self.peer_vn = msg.twist.linear.x
        self.peer_ve = msg.twist.linear.y

    def _failsafe_cb(self, msg: String):
        self.failsafe_state = msg.data

    # ── Control loop ────────────────────────────────────────

    def _control_tick(self):
        if self.failsafe_state != 'PROX_AVOID':
            return
        if self.my_lat == 0.0 and self.my_lon == 0.0:
            return

        result = compute_escape(
            my_lat=self.my_lat, my_lon=self.my_lon, my_alt=self.my_alt,
            peer_lat=self.peer_lat, peer_lon=self.peer_lon,
            peer_alt=self.peer_alt,
            my_vn=self.my_vn, my_ve=self.my_ve,
            peer_vn=self.peer_vn, peer_ve=self.peer_ve,
            safety_dist=self.safety_dist,
            escape_speed=self.escape_speed,
            max_accel=self.max_accel,
            state=self.escape_state,
        )

        cmd = TwistStamped()
        cmd.header.stamp = self.get_clock().now().to_msg()
        cmd.twist.linear.x = result['vn']
        cmd.twist.linear.y = result['ve']
        cmd.twist.linear.z = result['vd']
        self.pub_cmd_vel.publish(cmd)

        if result['emergency']:
            status = String()
            status.data = f"dist={result['peer_dist']:.1f} flags={result['flags']}"
            self.pub_status.publish(status)
            self.get_logger().warn(f"ESCAPE EMERGENCY: {result['flags']}")


def main(args=None):
    rclpy.init(args=args)
    node = LeaderNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
