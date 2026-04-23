#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from tf2_ros import TransformBroadcaster, TransformListener, Buffer
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped, TransformStamped, Transform, Quaternion
import tf_transformations
import tf2_geometry_msgs
from dataclasses import dataclass
from typing import Any


@dataclass
class OdomSource:
    odom_sub: Any = None
    path_pub: Any = None
    transform: TransformStamped = None
    last_msg: Odometry = None
    path: Path = None


MAX_PATH_ITEMS = 300


class OdomVizNode(Node):
    def __init__(self):
        super().__init__('odom_viz_node')

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.tf_broadcaster = TransformBroadcaster(self)

        self.sim_odom_sub = self.create_subscription(
            Odometry,
            "/sim/odom",
            self.sim_odom_callback,
            10
        )

        self.timer = self.create_timer(1 / 10, self.timer_callback)
        self.last_sim_odom = None

        self.sources = [
            self.make_source("/icp/odom"),
            self.make_source("/wheel/odom"),
            self.make_source("/ekf/odom"),
        ]

    def make_source(self, topic: str):
        source = OdomSource()

        def msg_callback(msg: Odometry):
            if self.last_sim_odom is None: return
            if source.transform is None:
                source.transform = self.compute_transform(msg)
            source.last_msg = msg

        source.odom_sub = self.create_subscription(Odometry, topic, msg_callback, 10)
        source.path_pub = self.create_publisher(Path, f"{topic}/path", 10)
        source.path = Path()
        source.path.header.frame_id = "sim_odom"
        return source

    def quat_to_arr(self, q: Quaternion):
        return [q.x, q.y, q.z, q.w]

    def sim_odom_callback(self, msg: Odometry):
        self.last_sim_odom = msg
        t = TransformStamped()
        t.header.stamp = msg.header.stamp
        t.header.frame_id = "sim_odom"
        t.child_frame_id = "base_link"
        t.transform.translation.x = msg.pose.pose.position.x
        t.transform.translation.y = msg.pose.pose.position.y
        t.transform.translation.z = msg.pose.pose.position.z
        t.transform.rotation.x = msg.pose.pose.orientation.x
        t.transform.rotation.y = msg.pose.pose.orientation.y
        t.transform.rotation.z = msg.pose.pose.orientation.z
        t.transform.rotation.w = msg.pose.pose.orientation.w
        # self.tf_broadcaster.sendTransform(t)

    def compute_transform(self, msg: Odometry):
        t = TransformStamped()
        q = self.quat_to_arr(self.last_sim_odom.pose.pose.orientation)
        q2 = self.quat_to_arr(msg.pose.pose.orientation)
        
        q_inv = tf_transformations.quaternion_inverse(q)
        q2_inv = tf_transformations.quaternion_inverse(q2)
        q_diff = tf_transformations.quaternion_multiply(q, q2_inv)
        t.transform.rotation.x = q_diff[0]
        t.transform.rotation.y = q_diff[1]
        t.transform.rotation.z = q_diff[2]
        t.transform.rotation.w = q_diff[3]

        point = msg.pose.pose.position
        point = [point.x, point.y, point.z, 0.0]
        q_p = tf_transformations.quaternion_multiply(q_diff, point)
        q_conj = tf_transformations.quaternion_conjugate(q_diff)
        point = tf_transformations.quaternion_multiply(q_p, q_conj)[:3]
        t.transform.translation.x = -point[0] + self.last_sim_odom.pose.pose.position.x
        t.transform.translation.y = -point[1] + self.last_sim_odom.pose.pose.position.y
        t.transform.translation.z = -point[2] + self.last_sim_odom.pose.pose.position.z

        return t

    def timer_callback(self):
        now = self.get_clock().now().to_msg()

        for source in self.sources:
            if source.last_msg is None: continue
            odom_pose = source.last_msg.pose.pose
            odom_pose = tf2_geometry_msgs.do_transform_pose(odom_pose, source.transform)

            pose = PoseStamped()
            pose.header.stamp = now
            pose.header.frame_id = "sim_odom"
            pose.pose.position.x = odom_pose.position.x
            pose.pose.position.y = odom_pose.position.y
            pose.pose.position.z = odom_pose.position.z
            pose.pose.orientation = odom_pose.orientation
            source.path.poses.append(pose)

            if len(source.path.poses) > MAX_PATH_ITEMS:
                source.path.poses = source.path.poses[-MAX_PATH_ITEMS:]
            source.path_pub.publish(source.path)


def main(args=None):
    try:
        rclpy.init(args=args)
        node = OdomVizNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
