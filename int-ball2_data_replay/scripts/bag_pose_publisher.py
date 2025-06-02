#!/usr/bin/python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from ib2_msgs.msg import Navigation
from scipy.spatial.transform import Rotation as R
import numpy as np


class NavigationToWorldTwist(Node):

    def __init__(self):
        super().__init__('navigation_to_world_twist')

        # Subscriber
        self.subscription = self.create_subscription(
            Navigation,
            '/sensor_fusion/navigation',
            self.ros_bag_pose_callback,
            10)

        # Publisher
        self.twist_pub = self.create_publisher(Twist, '/transform_twist', 10)

        # Timer
        self.timer = self.create_timer(0.1, self.publish_twist)

        # Pose of fixed docking station in world frame
        self.fixed_translation = np.array([10.88492, -3.53022, 4.07888])
        self.fixed_rotation_deg = [180.0, 0.32, -90.0]
        
        self.delta_threshold = 0.05

        self.prev_position = None
        self.prev_euler = None
        self.first_msg = True
        self.msg_received = False

    def ros_bag_pose_callback(self, msg):
        pos = np.array([
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
            msg.pose.pose.position.z
        ])
        quat = [
            msg.pose.pose.orientation.x,
            msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z,
            msg.pose.pose.orientation.w
        ]

        T_robot = np.eye(4)
        T_robot[:3, :3] = R.from_quat(quat).as_matrix()
        T_robot[:3, 3] = pos

        T_fixed = np.eye(4)
        T_fixed[:3, :3] = R.from_euler('xyz', self.fixed_rotation_deg, degrees=True).as_matrix()
        T_fixed[:3, 3] = self.fixed_translation

        # Transform robot pose to world frame
        T_world = T_fixed @ T_robot
        world_pos = T_world[:3, 3]

        rot_matrix = T_world[:3, :3]
        euler = R.from_matrix(rot_matrix).as_euler('xyz', degrees=True)

        twist_msg = Twist()

        # Filtering noise
        if self.first_msg or np.linalg.norm(world_pos - self.prev_position) > self.delta_threshold:
            twist_msg.linear.x, twist_msg.linear.y, twist_msg.linear.z = world_pos
            self.prev_position = world_pos
        else:
            twist_msg.linear.x, twist_msg.linear.y, twist_msg.linear.z = self.prev_position

        if self.first_msg or np.linalg.norm(euler - self.prev_euler) > self.delta_threshold:
            twist_msg.angular.x, twist_msg.angular.y, twist_msg.angular.z = euler
            self.prev_euler = euler
        else:
            twist_msg.angular.x, twist_msg.angular.y, twist_msg.angular.z = self.prev_euler

        self.twist_pub.publish(twist_msg)
        self.msg_received = True
        self.first_msg = False

    def publish_twist(self):
        if not self.msg_received:
            twist_msg = Twist()
            twist_msg.linear.x = self.fixed_translation[0]
            twist_msg.linear.y = self.fixed_translation[1]
            twist_msg.linear.z = self.fixed_translation[2]

            twist_msg.angular.x = self.fixed_rotation_deg[0]
            twist_msg.angular.y = self.fixed_rotation_deg[1]
            twist_msg.angular.z = self.fixed_rotation_deg[2]

            self.get_logger().warn('No Navigation msg received — publishing default docking pose')
            self.twist_pub.publish(twist_msg)
        else:
            self.msg_received = False

def main(args=None):
    rclpy.init(args=args)
    node = NavigationToWorldTwist()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
