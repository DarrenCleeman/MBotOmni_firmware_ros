#!/usr/bin/env python3
"""
MBot Omni Demo Drive
Demonstrates ROS2 velocity control of the MBot Omni:
  1. Drive forward in a straight line (3 seconds)
  2. Stop (2 seconds)
  3. Drive in a circle (5 seconds)
  4. Stop

Usage:
  # Make sure micro-ros-agent is running first:
  #   ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/mbot_microros
  #
  # Then run this script:
  #   python3 mbot_demo_drive.py
  #   -- or --
  #   ros2 run mbot_omni_firmware mbot_demo_drive  (if installed as a package)
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import time


class MBotDemoDriver(Node):
    def __init__(self):
        super().__init__('mbot_demo_driver')
        self.publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        self.get_logger().info('MBot Omni Demo Driver started')

    def send_cmd(self, vx: float, vy: float, wz: float):
        """Publish a Twist command."""
        msg = Twist()
        msg.linear.x = vx
        msg.linear.y = vy
        msg.angular.z = wz
        self.publisher.publish(msg)

    def stop(self):
        """Send zero velocity."""
        self.send_cmd(0.0, 0.0, 0.0)

    def run_demo(self):
        # Give the connection a moment to establish
        time.sleep(1.0)

        # --- Phase 1: Drive forward ---
        self.get_logger().info('Phase 1: Driving forward...')
        duration = 3.0  # seconds
        rate = 20  # Hz
        for _ in range(int(duration * rate)):
            self.send_cmd(0.2, 0.0, 0.0)  # 0.2 m/s forward
            time.sleep(1.0 / rate)

        # --- Phase 2: Stop ---
        self.get_logger().info('Phase 2: Stopping...')
        self.stop()
        time.sleep(2.0)

        # --- Phase 3: Drive in a circle ---
        self.get_logger().info('Phase 3: Driving in a circle...')
        duration = 5.0
        for _ in range(int(duration * rate)):
            self.send_cmd(0.15, 0.0, 0.8)  # forward + rotate
            time.sleep(1.0 / rate)

        # --- Done: Stop ---
        self.get_logger().info('Demo complete. Stopping.')
        self.stop()
        time.sleep(0.5)


def main(args=None):
    rclpy.init(args=args)
    node = MBotDemoDriver()
    try:
        node.run_demo()
    except KeyboardInterrupt:
        node.get_logger().info('Interrupted, stopping motors.')
        node.stop()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
