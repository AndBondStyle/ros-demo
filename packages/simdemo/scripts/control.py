#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Quaternion, TransformStamped
from std_msgs.msg import Float64MultiArray
from control_msgs.msg import DynamicJointState
from nav_msgs.msg import Odometry
from tf_transformations import quaternion_from_euler
import numpy as np
import tf2_ros


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
        self.joint_state_sub = self.create_subscription(
            DynamicJointState,
            "/dynamic_joint_states",
            self.join_states_callback,
            10
        )
        self.odom_pub = self.create_publisher(
            Odometry,
            "/wheel/odom",
            10
        )

        self.tfb = tf2_ros.TransformBroadcaster(self)

        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.last_time = None

    def cmd_vel_callback(self, msg: Twist):
        self.get_logger().info(f"{msg}")
        v = msg.linear.x  # m/s
        w = msg.angular.z  # rad/s

        left_speed = (v - w * self.track_width / 2.0) / self.wheel_radius  # rad/s
        right_speed = (v + w * self.track_width / 2.0) / self.wheel_radius  # rad/s
        wheel_speeds = Float64MultiArray()
        wheel_speeds.data = [left_speed, right_speed]

        self.publisher_.publish(wheel_speeds)
        # self.get_logger().info(
        #     f'FORWARD: v={v:.3f}, w={w:.3f} -> left={left_speed:.3f}, right={right_speed:.3f} rad/s'
        # )

    def join_states_callback(self, msg: DynamicJointState):
        left_speed = msg.interface_values[0].values[1]
        right_speed = msg.interface_values[1].values[1]
        left_speed = left_speed + np.random.normal(0.0, 0.1)
        right_speed = right_speed + np.random.normal(0.0, 0.1)

        v = (left_speed + right_speed) * self.wheel_radius / 2
        w = (right_speed - left_speed) * self.wheel_radius / self.track_width

        current_time = msg.header.stamp
        if self.last_time is None:
            self.last_time = current_time
            return

        current_sec = current_time.sec + current_time.nanosec * 1e-9
        last_sec = self.last_time.sec + self.last_time.nanosec * 1e-9
        dt = current_sec - last_sec
        if dt <= 0.0:
            self.x = 0.0
            self.y = 0.0
            self.theta = 0.0
            self.last_time = current_time
            return
        
        self.last_time = current_time

        delta_theta = w * dt
        delta_trans = v * dt
        self.theta += delta_theta
        if delta_trans != 0.0:
            self.x += delta_trans * np.cos(self.theta - delta_theta / 2)
            self.y += delta_trans * np.sin(self.theta - delta_theta / 2)
        self.theta = np.arctan2(np.sin(self.theta), np.cos(self.theta))
        
        odom = Odometry()
        odom.header.stamp = current_time
        odom.header.frame_id = "wheel_odom"
        odom.child_frame_id = "base_link"

        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0

        q = quaternion_from_euler(0.0, 0.0, self.theta)
        odom.pose.pose.orientation = Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])

        odom.twist.twist.linear.x = v
        odom.twist.twist.angular.z = w
        self.odom_pub.publish(odom)

        tf = TransformStamped()
        tf.header.stamp = current_time
        tf.header.frame_id = "wheel_odom"
        tf.child_frame_id = "base_link"

        tf.transform.translation.x = self.x
        tf.transform.translation.y = self.y
        tf.transform.translation.z = 0.0

        tf.transform.rotation.x = q[0]
        tf.transform.rotation.y = q[1]
        tf.transform.rotation.z = q[2]
        tf.transform.rotation.w = q[3]
        self.tfb.sendTransform(tf)

        # self.get_logger().info(
        #     f'BACKWARD: left={left_speed:.3f}, right={right_speed:.3f} rad/s -> v={v:.3f}, w={w:.3f}'
        # )


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
