"""
Follow controller — the core follower ROS2 node.

Maintains a fixed NED offset from the leader using the 3-mode
sigmoid-blended guidance (Tracking, Catch-up, Evasion).

Only publishes velocity commands when failsafe state is FOLLOWING.
"""

import math
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import TwistStamped, Vector3Stamped
from std_msgs.msg import String

from follower_drone.geo_utils import gps_to_ned, ned_to_gps
from follower_drone.guidance import GuidanceConfig, GuidanceState, compute_guidance
from follower_drone.command_smoother import CommandSmoother

METERS_PER_DEG_LAT = 111320.0

SENSOR_QOS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE,
    history=HistoryPolicy.KEEP_LAST,
    depth=5,
)


class FollowController(Node):
    def __init__(self):
        super().__init__('follow_controller')

        # ── Follow geometry ───────────────────────────────────
        self.declare_parameter('offset_north_m', -5.0)
        self.declare_parameter('offset_east_m', 3.0)
        self.declare_parameter('offset_down_m', 0.0)
        self.declare_parameter('control_rate_hz', 10.0)

        # ── Guidance params ───────────────────────────────────
        self.declare_parameter('guidance.kp', 0.7)
        self.declare_parameter('guidance.kd', 0.4)
        self.declare_parameter('guidance.ff_gain', 0.8)
        self.declare_parameter('guidance.catchup_dist_m', 8.0)
        self.declare_parameter('guidance.max_catchup_speed', 4.0)
        self.declare_parameter('guidance.safety_dist_m', 1.0)
        self.declare_parameter('guidance.escape_speed', 3.0)
        self.declare_parameter('guidance.max_speed', 3.0)
        self.declare_parameter('guidance.max_vertical_speed', 1.5)
        self.declare_parameter('guidance.max_accel', 4.0)
        self.declare_parameter('guidance.min_altitude_m', 3.0)
        self.declare_parameter('guidance.critical_altitude_m', 1.5)
        self.declare_parameter('guidance.deadzone_m', 0.5)

        # ── Load params ───────────────────────────────────────
        self.offset_n = self.get_parameter('offset_north_m').value
        self.offset_e = self.get_parameter('offset_east_m').value
        self.offset_d = self.get_parameter('offset_down_m').value
        rate = self.get_parameter('control_rate_hz').value

        self.cfg = GuidanceConfig(
            kp=self.get_parameter('guidance.kp').value,
            kd=self.get_parameter('guidance.kd').value,
            ff_gain=self.get_parameter('guidance.ff_gain').value,
            catchup_dist_m=self.get_parameter('guidance.catchup_dist_m').value,
            max_catchup_speed=self.get_parameter('guidance.max_catchup_speed').value,
            safety_dist_m=self.get_parameter('guidance.safety_dist_m').value,
            escape_speed=self.get_parameter('guidance.escape_speed').value,
            max_speed=self.get_parameter('guidance.max_speed').value,
            max_vertical_speed=self.get_parameter('guidance.max_vertical_speed').value,
            max_accel=self.get_parameter('guidance.max_accel').value,
            min_altitude_m=self.get_parameter('guidance.min_altitude_m').value,
            critical_altitude_m=self.get_parameter('guidance.critical_altitude_m').value,
            deadzone_m=self.get_parameter('guidance.deadzone_m').value,
        )

        self.guidance_state = GuidanceState()
        self.cmd_smoother = CommandSmoother(dt=1.0 / rate)

        # ── Own state ─────────────────────────────────────────
        self.my_lat = 0.0
        self.my_lon = 0.0
        self.my_alt = 0.0
        self.my_vn = 0.0
        self.my_ve = 0.0
        self.my_vd = 0.0

        # ── Leader state ──────────────────────────────────────
        self.leader_lat = 0.0
        self.leader_lon = 0.0
        self.leader_alt = 0.0
        self.leader_vn = 0.0
        self.leader_ve = 0.0

        # ── Failsafe ──────────────────────────────────────────
        self.failsafe_state = 'PREFLIGHT'

        # ── Subscribers — own telemetry ───────────────────────
        self.create_subscription(
            NavSatFix, 'mavros/global_position/global',
            self._own_gps_cb, SENSOR_QOS)
        self.create_subscription(
            TwistStamped, 'mavros/local_position/velocity_body',
            self._own_vel_cb, SENSOR_QOS)

        # ── Subscribers — leader from peer_monitor ────────────
        self.create_subscription(
            NavSatFix, 'peer/global_position',
            self._leader_gps_cb, SENSOR_QOS)
        self.create_subscription(
            TwistStamped, 'peer/velocity',
            self._leader_vel_cb, SENSOR_QOS)

        # ── Subscriber — failsafe ─────────────────────────────
        self.create_subscription(String, 'failsafe/state', self._failsafe_cb, 10)

        # ── Publishers ────────────────────────────────────────
        self.pub_cmd_vel = self.create_publisher(
            TwistStamped, 'mavros/setpoint_velocity/cmd_vel', 10)
        self.pub_offset_error = self.create_publisher(
            Vector3Stamped, 'follow/offset_error', 10)
        self.pub_status = self.create_publisher(
            String, 'follow/guidance_status', 10)

        self.create_timer(1.0 / rate, self._control_tick)
        self.get_logger().info(
            f'FollowController started (3-mode guidance) — '
            f'offset=({self.offset_n}, {self.offset_e}, {self.offset_d})m')

    # ── Callbacks ───────────────────────────────────────────

    def _own_gps_cb(self, msg: NavSatFix):
        self.my_lat = msg.latitude
        self.my_lon = msg.longitude
        self.my_alt = msg.altitude

    def _own_vel_cb(self, msg: TwistStamped):
        self.my_vn = msg.twist.linear.x
        self.my_ve = msg.twist.linear.y
        self.my_vd = msg.twist.linear.z

    def _leader_gps_cb(self, msg: NavSatFix):
        self.leader_lat = msg.latitude
        self.leader_lon = msg.longitude
        self.leader_alt = msg.altitude

    def _leader_vel_cb(self, msg: TwistStamped):
        self.leader_vn = msg.twist.linear.x
        self.leader_ve = msg.twist.linear.y

    def _failsafe_cb(self, msg: String):
        self.failsafe_state = msg.data

    # ── Control loop ────────────────────────────────────────

    def _control_tick(self):
        if self.failsafe_state != 'FOLLOWING':
            return
        if self.my_lat == 0.0 and self.my_lon == 0.0:
            return
        if self.leader_lat == 0.0 and self.leader_lon == 0.0:
            return

        # ── Compute target: leader + NED offset ─────────────
        target_lat, target_lon = ned_to_gps(
            self.offset_n, self.offset_e,
            self.leader_lat, self.leader_lon)
        target_alt = self.leader_alt - self.offset_d

        # ── Feedforward: shift target ahead ──────────────────
        ff = self.cfg.ff_gain
        if ff > 0.01 and abs(self.leader_lat) > 1e-6:
            cos_lat = math.cos(math.radians(self.leader_lat))
            target_lat += ff * self.leader_vn * 0.1 / METERS_PER_DEG_LAT
            target_lon += (ff * self.leader_ve * 0.1
                           / (METERS_PER_DEG_LAT * max(cos_lat, 1e-6)))

        # ── Publish offset error for diagnostics ─────────────
        err_n, err_e = gps_to_ned(target_lat, target_lon, self.my_lat, self.my_lon)
        err_msg = Vector3Stamped()
        err_msg.header.stamp = self.get_clock().now().to_msg()
        err_msg.vector.x = err_n
        err_msg.vector.y = err_e
        err_msg.vector.z = -(target_alt - self.my_alt)
        self.pub_offset_error.publish(err_msg)

        # ── 3-mode guidance ──────────────────────────────────
        result = compute_guidance(
            my_lat=self.my_lat, my_lon=self.my_lon, my_alt=self.my_alt,
            my_vn=self.my_vn, my_ve=self.my_ve, my_vd=self.my_vd,
            peer_lat=self.leader_lat, peer_lon=self.leader_lon,
            peer_alt=self.leader_alt,
            peer_vn=self.leader_vn, peer_ve=self.leader_ve,
            goal_lat=target_lat, goal_lon=target_lon, goal_alt=target_alt,
            cfg=self.cfg,
            state=self.guidance_state,
        )

        # ── Smooth & publish velocity command ────────────────
        sm_vn, sm_ve, sm_vd = self.cmd_smoother.filter(
            result['vn'], result['ve'], result['vd'])

        cmd = TwistStamped()
        cmd.header.stamp = self.get_clock().now().to_msg()
        cmd.twist.linear.x = sm_vn
        cmd.twist.linear.y = sm_ve
        cmd.twist.linear.z = sm_vd
        self.pub_cmd_vel.publish(cmd)

        # ── Publish status for diagnostics ───────────────────
        if result['mode'] != 'TRACKING' or result['emergency']:
            status = String()
            status.data = (
                f"mode={result['mode']} "
                f"dist={result['peer_dist']:.1f} "
                f"w_e={result['w_evasion']:.2f} "
                f"w_c={result['w_catchup']:.2f} "
                f"emerg={result['emergency']}")
            self.pub_status.publish(status)

            if result['emergency']:
                self.get_logger().warn(f"GUIDANCE EMERGENCY: {result['flags']}")


def main(args=None):
    rclpy.init(args=args)
    node = FollowController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
