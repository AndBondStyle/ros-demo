#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64MultiArray


class DiffDriveController(Node):
    def __init__(self):
        super().__init__('diff_drive_controller')

        self.declare_parameter('wheel_radius', 0.1)
        self.declare_parameter('track_width', 0.35)

        self.wheel_radius = self.get_parameter('wheel_radius').value
        self.track_width = self.get_parameter('track_width').value

        self.publisher_ = self.create_publisher(
            Float64MultiArray,
            '/velocity_controller/commands',
            10
        )
        self.subscription = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10
        )

    def cmd_vel_callback(self, msg: Twist):
        self.get_logger().info(f"{msg}")
        v = msg.linear.x
        omega = msg.angular.z

        left_speed = (v - omega * self.track_width / 2.0) / self.wheel_radius
        right_speed = (v + omega * self.track_width / 2.0) / self.wheel_radius

        wheel_speeds = Float64MultiArray()
        wheel_speeds.data = [left_speed, right_speed]

        self.publisher_.publish(wheel_speeds)

        self.get_logger().info(
            f'v={v:.3f}, omega={omega:.3f} -> left={left_speed:.3f}, right={right_speed:.3f} rad/s'
        )


def main(args=None):
    rclpy.init(args=args)
    node = DiffDriveController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down diff_drive_controller')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
