"""
Follower failsafe state machine.

States:
  PREFLIGHT    — waiting for GPS fix + GUIDED mode + armed
  FOLLOWING    — leader position known, follow controller active
  PROX_AVOID   — too close to leader, active repulsion
  LEADER_LOST  — leader heartbeat stale, phased response:
                 0-10s: LOITER, 10-30s: maintain loiter, 30s+: RTL
  RC_OVERRIDE  — RC override arriving from leader, yield control
  LOW_BATTERY  — battery below threshold, RTL/LAND
"""

import time
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from sensor_msgs.msg import NavSatFix, BatteryState
from geometry_msgs.msg import TwistStamped
from std_msgs.msg import String, Float64
from mavros_msgs.msg import State as MavrosState, RCIn
from mavros_msgs.srv import SetMode, CommandBool

from follower_drone.geo_utils import gps_distance_2d
from follower_drone.guidance import GuidanceState, compute_escape

SENSOR_QOS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE,
    history=HistoryPolicy.KEEP_LAST,
    depth=5,
)


class FollowerFailsafe(Node):
    def __init__(self):
        super().__init__('failsafe_node')

        self.declare_parameter('hard_radius_m', 1.0)
        self.declare_parameter('soft_radius_m', 2.0)
        self.declare_parameter('clear_multiplier', 1.5)
        self.declare_parameter('escape_speed', 3.0)
        self.declare_parameter('max_avoidance_speed', 3.0)
        self.declare_parameter('leader_timeout_s', 5.0)
        self.declare_parameter('loiter_phase_s', 10.0)
        self.declare_parameter('rtl_phase_s', 30.0)
        self.declare_parameter('low_battery_pct', 20.0)
        self.declare_parameter('tick_rate_hz', 10.0)

        self.hard_radius = self.get_parameter('hard_radius_m').value
        self.soft_radius = self.get_parameter('soft_radius_m').value
        self.clear_mult = self.get_parameter('clear_multiplier').value
        self.escape_speed = self.get_parameter('escape_speed').value
        self.max_avoid_speed = self.get_parameter('max_avoidance_speed').value
        self.leader_timeout = self.get_parameter('leader_timeout_s').value
        self.loiter_phase = self.get_parameter('loiter_phase_s').value
        self.rtl_phase = self.get_parameter('rtl_phase_s').value
        self.low_batt = self.get_parameter('low_battery_pct').value
        rate = self.get_parameter('tick_rate_hz').value

        self.escape_state = GuidanceState()

        self.state = 'PREFLIGHT'
        self.leader_lost_time = 0.0
        self.rc_override_active = False

        # Telemetry
        self.my_lat = 0.0
        self.my_lon = 0.0
        self.my_alt = 0.0
        self.my_vn = 0.0
        self.my_ve = 0.0
        self.leader_lat = 0.0
        self.leader_lon = 0.0
        self.leader_alt = 0.0
        self.leader_vn = 0.0
        self.leader_ve = 0.0
        self.battery_pct = -1.0
        self.armed = False
        self.mode = ''
        self.peer_age = -1.0
        self.last_rc_override_time = 0.0

        # Subscribers
        self.create_subscription(
            NavSatFix, 'mavros/global_position/global',
            self._own_gps_cb, SENSOR_QOS)
        self.create_subscription(
            TwistStamped, 'mavros/local_position/velocity_body',
            self._own_vel_cb, SENSOR_QOS)
        self.create_subscription(
            NavSatFix, 'peer/global_position',
            self._leader_gps_cb, SENSOR_QOS)
        self.create_subscription(
            TwistStamped, 'peer/velocity',
            self._leader_vel_cb, SENSOR_QOS)
        self.create_subscription(
            Float64, 'peer/heartbeat_age',
            self._peer_age_cb, SENSOR_QOS)
        self.create_subscription(
            BatteryState, 'mavros/battery',
            self._battery_cb, SENSOR_QOS)
        self.create_subscription(
            MavrosState, 'mavros/state',
            self._state_cb, SENSOR_QOS)
        self.create_subscription(
            RCIn, 'mavros/rc/in',
            self._rc_cb, SENSOR_QOS)

        # Publishers
        self.pub_state = self.create_publisher(String, 'failsafe/state', 10)
        self.pub_avoid_vel = self.create_publisher(
            TwistStamped, 'mavros/setpoint_velocity/cmd_vel', 10)

        # Service clients for mode changes
        self.set_mode_client = self.create_client(SetMode, 'mavros/set_mode')

        self.create_timer(1.0 / rate, self._tick)
        self.get_logger().info('FollowerFailsafe started')

    # ── Callbacks ───────────────────────────────────────────

    def _own_gps_cb(self, msg: NavSatFix):
        self.my_lat = msg.latitude
        self.my_lon = msg.longitude
        self.my_alt = msg.altitude

    def _own_vel_cb(self, msg: TwistStamped):
        self.my_vn = msg.twist.linear.x
        self.my_ve = msg.twist.linear.y

    def _leader_gps_cb(self, msg: NavSatFix):
        self.leader_lat = msg.latitude
        self.leader_lon = msg.longitude
        self.leader_alt = msg.altitude

    def _leader_vel_cb(self, msg: TwistStamped):
        self.leader_vn = msg.twist.linear.x
        self.leader_ve = msg.twist.linear.y

    def _peer_age_cb(self, msg: Float64):
        self.peer_age = msg.data

    def _battery_cb(self, msg: BatteryState):
        self.battery_pct = msg.percentage * 100.0

    def _state_cb(self, msg: MavrosState):
        self.armed = msg.armed
        self.mode = msg.mode

    def _rc_cb(self, msg: RCIn):
        has_override = any(ch > 0 and ch != 65535 for ch in msg.channels[:4])
        if has_override:
            self.last_rc_override_time = time.time()

    # ── State machine ──────────────────────────────────────

    def _tick(self):
        now = time.time()
        self.rc_override_active = (now - self.last_rc_override_time) < 1.5

        if self.state == 'PREFLIGHT':
            has_gps = (self.my_lat != 0.0 or self.my_lon != 0.0)
            leader_known = (self.peer_age > 0 and self.peer_age < self.leader_timeout)
            if has_gps and self.armed and leader_known:
                self.state = 'FOLLOWING'
                self.get_logger().info('Failsafe: PREFLIGHT -> FOLLOWING')

        elif self.state == 'FOLLOWING':
            if self.rc_override_active:
                self.state = 'RC_OVERRIDE'
                self.get_logger().info('Failsafe: FOLLOWING -> RC_OVERRIDE')
            elif self._check_proximity():
                self.state = 'PROX_AVOID'
                self.get_logger().warn('Failsafe: FOLLOWING -> PROX_AVOID')
            elif self.peer_age > self.leader_timeout:
                self.state = 'LEADER_LOST'
                self.leader_lost_time = now
                self.get_logger().warn('Failsafe: FOLLOWING -> LEADER_LOST')
                self._request_mode('LOITER')
            elif 0 < self.battery_pct < self.low_batt:
                self.state = 'LOW_BATTERY'
                self.get_logger().error('Failsafe: FOLLOWING -> LOW_BATTERY')
                self._request_mode('RTL')

        elif self.state == 'PROX_AVOID':
            self._publish_avoidance()
            if not self._check_proximity():
                self.state = 'FOLLOWING'
                self.get_logger().info('Failsafe: PROX_AVOID -> FOLLOWING (clear)')

        elif self.state == 'LEADER_LOST':
            elapsed = now - self.leader_lost_time
            if 0 < self.peer_age < self.leader_timeout:
                self.state = 'FOLLOWING'
                self.get_logger().info('Failsafe: LEADER_LOST -> FOLLOWING (restored)')
                self._request_mode('GUIDED')
            elif elapsed > self.rtl_phase:
                self.get_logger().error(
                    f'Failsafe: LEADER_LOST for {elapsed:.0f}s -> RTL')
                self._request_mode('RTL')

        elif self.state == 'RC_OVERRIDE':
            if not self.rc_override_active:
                self.state = 'FOLLOWING'
                self.get_logger().info('Failsafe: RC_OVERRIDE -> FOLLOWING')
                self._request_mode('GUIDED')

        elif self.state == 'LOW_BATTERY':
            pass  # Terminal

        msg = String()
        msg.data = self.state
        self.pub_state.publish(msg)

    def _check_proximity(self) -> bool:
        if (self.leader_lat == 0.0 and self.leader_lon == 0.0):
            return False
        if (self.my_lat == 0.0 and self.my_lon == 0.0):
            return False

        dist = gps_distance_2d(
            self.my_lat, self.my_lon, self.leader_lat, self.leader_lon)
        clear_dist = self.hard_radius * self.clear_mult

        if self.state == 'PROX_AVOID':
            return dist < clear_dist
        return dist < self.hard_radius

    def _publish_avoidance(self):
        result = compute_escape(
            my_lat=self.my_lat, my_lon=self.my_lon, my_alt=self.my_alt,
            peer_lat=self.leader_lat, peer_lon=self.leader_lon,
            peer_alt=self.leader_alt,
            my_vn=self.my_vn, my_ve=self.my_ve,
            peer_vn=self.leader_vn, peer_ve=self.leader_ve,
            safety_dist=self.hard_radius,
            escape_speed=self.escape_speed,
            state=self.escape_state,
        )

        cmd = TwistStamped()
        cmd.header.stamp = self.get_clock().now().to_msg()
        cmd.twist.linear.x = result['vn']
        cmd.twist.linear.y = result['ve']
        cmd.twist.linear.z = result['vd']
        self.pub_avoid_vel.publish(cmd)

        if result['emergency']:
            self.get_logger().warn(f"Failsafe ESCAPE EMERGENCY: {result['flags']}")

    def _request_mode(self, mode_name: str):
        if not self.set_mode_client.service_is_ready():
            self.get_logger().warn(f'SetMode service not ready, cannot switch to {mode_name}')
            return
        req = SetMode.Request()
        req.custom_mode = mode_name
        future = self.set_mode_client.call_async(req)
        self.get_logger().info(f'Requesting mode: {mode_name}')


def main(args=None):
    rclpy.init(args=args)
    node = FollowerFailsafe()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
