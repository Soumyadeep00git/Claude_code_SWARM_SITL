"""
Telemetry bridge — reads MAVLink from both drones via RFD9000 USB,
publishes as ROS2 topics, and logs to CSV files.

This runs on the GCS laptop alongside Mission Planner.
Mission Planner connects to the same RFD9000 via its own serial port,
or this node can share the port via MAVProxy.
"""

import os
import time
import csv
from datetime import datetime
from pathlib import Path

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import String
from pymavlink import mavutil


class TelemetryBridge(Node):
    def __init__(self):
        super().__init__('telemetry_bridge')

        self.declare_parameter('leader_sysid', 1)
        self.declare_parameter('follower_sysid', 2)
        self.declare_parameter('mavlink_port', '/dev/ttyUSB0')
        self.declare_parameter('mavlink_baud', 57600)
        self.declare_parameter('log_rate_hz', 2.0)
        self.declare_parameter('log_directory', '~/flight_logs')

        self.leader_sysid = self.get_parameter('leader_sysid').value
        self.follower_sysid = self.get_parameter('follower_sysid').value
        port = self.get_parameter('mavlink_port').value
        baud = self.get_parameter('mavlink_baud').value
        log_rate = self.get_parameter('log_rate_hz').value
        log_dir = os.path.expanduser(self.get_parameter('log_directory').value)

        # MAVLink connection to RFD9000 USB
        self.mav = mavutil.mavlink_connection(port, baud=baud, source_system=255)
        self.get_logger().info(f'Connected to RFD9000 on {port} @ {baud}')

        # Publishers
        self.pub_leader_pos = self.create_publisher(NavSatFix, 'leader/position', 10)
        self.pub_follower_pos = self.create_publisher(NavSatFix, 'follower/position', 10)
        self.pub_status = self.create_publisher(String, 'gcs/status', 10)

        # Logging
        Path(log_dir).mkdir(parents=True, exist_ok=True)
        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        log_path = os.path.join(log_dir, f'flight_{timestamp}.csv')
        self.log_file = open(log_path, 'w', newline='')
        self.csv_writer = csv.writer(self.log_file)
        self.csv_writer.writerow([
            'time', 'sysid', 'lat', 'lon', 'alt', 'vx', 'vy', 'vz', 'heading'])
        self.get_logger().info(f'Logging to {log_path}')

        # State
        self.leader_state = {}
        self.follower_state = {}
        self.last_log_time = 0.0

        # Timers
        self.create_timer(0.02, self._read_mavlink)  # 50Hz drain
        self.create_timer(1.0 / log_rate, self._log_tick)

    def _read_mavlink(self):
        while True:
            msg = self.mav.recv_match(blocking=False)
            if msg is None:
                break
            sysid = msg.get_srcSystem()
            msg_type = msg.get_type()

            if msg_type == 'GLOBAL_POSITION_INT':
                state = {
                    'lat': msg.lat / 1e7,
                    'lon': msg.lon / 1e7,
                    'alt': msg.relative_alt / 1000.0,
                    'vx': msg.vx / 100.0,
                    'vy': msg.vy / 100.0,
                    'vz': msg.vz / 100.0,
                    'heading': msg.hdg / 100.0,
                }

                fix = NavSatFix()
                fix.header.stamp = self.get_clock().now().to_msg()
                fix.latitude = state['lat']
                fix.longitude = state['lon']
                fix.altitude = state['alt']

                if sysid == self.leader_sysid:
                    self.leader_state = state
                    self.pub_leader_pos.publish(fix)
                elif sysid == self.follower_sysid:
                    self.follower_state = state
                    self.pub_follower_pos.publish(fix)

    def _log_tick(self):
        now = time.time()
        for sysid, state in [(self.leader_sysid, self.leader_state),
                              (self.follower_sysid, self.follower_state)]:
            if state:
                self.csv_writer.writerow([
                    f'{now:.3f}', sysid,
                    f'{state["lat"]:.7f}', f'{state["lon"]:.7f}',
                    f'{state["alt"]:.2f}',
                    f'{state["vx"]:.2f}', f'{state["vy"]:.2f}', f'{state["vz"]:.2f}',
                    f'{state["heading"]:.1f}',
                ])
        self.log_file.flush()

    def destroy_node(self):
        self.log_file.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = TelemetryBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
