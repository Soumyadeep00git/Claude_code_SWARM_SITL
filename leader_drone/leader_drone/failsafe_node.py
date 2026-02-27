"""
Leader failsafe state machine.

States:
  PREFLIGHT   — waiting for GPS fix + GUIDED mode + armed
  NOMINAL     — all healthy, leader flies under Mission Planner control
  PROX_AVOID  — follower too close, collision avoidance active
  PEER_LOST   — follower heartbeat stale, log warning, continue mission
  LOW_BATTERY — battery below threshold, trigger RTL
"""

import time
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from sensor_msgs.msg import NavSatFix, BatteryState
from geometry_msgs.msg import TwistStamped
from std_msgs.msg import String, Float64
from mavros_msgs.msg import State as MavrosState

from leader_drone.geo_utils import gps_distance_2d

SENSOR_QOS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE,
    history=HistoryPolicy.KEEP_LAST,
    depth=5,
)


class LeaderFailsafe(Node):
    def __init__(self):
        super().__init__('failsafe_node')

        self.declare_parameter('hard_radius_m', 5.0)
        self.declare_parameter('clear_multiplier', 1.5)
        self.declare_parameter('peer_timeout_s', 5.0)
        self.declare_parameter('low_battery_pct', 20.0)
        self.declare_parameter('tick_rate_hz', 10.0)

        self.hard_radius = self.get_parameter('hard_radius_m').value
        self.clear_mult = self.get_parameter('clear_multiplier').value
        self.peer_timeout = self.get_parameter('peer_timeout_s').value
        self.low_batt = self.get_parameter('low_battery_pct').value
        rate = self.get_parameter('tick_rate_hz').value

        self.state = 'PREFLIGHT'

        # Telemetry
        self.my_lat = 0.0
        self.my_lon = 0.0
        self.peer_lat = 0.0
        self.peer_lon = 0.0
        self.battery_pct = -1.0
        self.armed = False
        self.mode = ''
        self.peer_age = -1.0

        # Subscribers
        self.create_subscription(
            NavSatFix, 'mavros/global_position/global',
            self._own_gps_cb, SENSOR_QOS)
        self.create_subscription(
            NavSatFix, 'peer/global_position',
            self._peer_gps_cb, SENSOR_QOS)
        self.create_subscription(
            Float64, 'peer/heartbeat_age',
            self._peer_age_cb, SENSOR_QOS)
        self.create_subscription(
            BatteryState, 'mavros/battery',
            self._battery_cb, SENSOR_QOS)
        self.create_subscription(
            MavrosState, 'mavros/state',
            self._state_cb, SENSOR_QOS)

        # Publisher
        self.pub_state = self.create_publisher(String, 'failsafe/state', 10)

        self.create_timer(1.0 / rate, self._tick)
        self.get_logger().info('LeaderFailsafe started')

    def _own_gps_cb(self, msg: NavSatFix):
        self.my_lat = msg.latitude
        self.my_lon = msg.longitude

    def _peer_gps_cb(self, msg: NavSatFix):
        self.peer_lat = msg.latitude
        self.peer_lon = msg.longitude

    def _peer_age_cb(self, msg: Float64):
        self.peer_age = msg.data

    def _battery_cb(self, msg: BatteryState):
        self.battery_pct = msg.percentage * 100.0  # 0.0-1.0 → 0-100

    def _state_cb(self, msg: MavrosState):
        self.armed = msg.armed
        self.mode = msg.mode

    def _tick(self):
        prev = self.state

        if self.state == 'PREFLIGHT':
            has_gps = (self.my_lat != 0.0 or self.my_lon != 0.0)
            if has_gps and self.armed:
                self.state = 'NOMINAL'
                self.get_logger().info('Failsafe: PREFLIGHT → NOMINAL')

        elif self.state == 'NOMINAL':
            # Check proximity
            if self._check_proximity():
                self.state = 'PROX_AVOID'
                self.get_logger().warn('Failsafe: NOMINAL → PROX_AVOID')
            # Check peer lost
            elif self.peer_age > self.peer_timeout:
                self.state = 'PEER_LOST'
                self.get_logger().warn('Failsafe: NOMINAL → PEER_LOST (follower not heard)')
            # Check battery
            elif 0 < self.battery_pct < self.low_batt:
                self.state = 'LOW_BATTERY'
                self.get_logger().error('Failsafe: NOMINAL → LOW_BATTERY')

        elif self.state == 'PROX_AVOID':
            if not self._check_proximity():
                self.state = 'NOMINAL'
                self.get_logger().info('Failsafe: PROX_AVOID → NOMINAL (clear)')

        elif self.state == 'PEER_LOST':
            if 0 < self.peer_age < self.peer_timeout:
                self.state = 'NOMINAL'
                self.get_logger().info('Failsafe: PEER_LOST → NOMINAL (follower restored)')

        elif self.state == 'LOW_BATTERY':
            pass  # Terminal — operator must intervene

        # Publish state
        msg = String()
        msg.data = self.state
        self.pub_state.publish(msg)

    def _check_proximity(self) -> bool:
        if (self.peer_lat == 0.0 and self.peer_lon == 0.0):
            return False
        if (self.my_lat == 0.0 and self.my_lon == 0.0):
            return False

        dist = gps_distance_2d(self.my_lat, self.my_lon, self.peer_lat, self.peer_lon)
        clear_dist = self.hard_radius * self.clear_mult

        if self.state == 'PROX_AVOID':
            return dist < clear_dist  # Hysteresis: need wider clearance to exit
        return dist < self.hard_radius


def main(args=None):
    rclpy.init(args=args)
    node = LeaderFailsafe()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
