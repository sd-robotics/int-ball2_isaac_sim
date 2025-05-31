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

        self.subscription = self.create_subscription(
            Navigation,
            '/sensor_fusion/navigation',
            self.listener_callback,
            10)

        self.twist_pub = self.create_publisher(Twist, '/transform_twist', 10)

        self.msg_received = False
        self.timer = self.create_timer(1.0, self.check_no_msg)

        # Fixed transform: robot to world
        self.fixed_translation = np.array([10.88492, -3.53022, 4.07888])
        self.fixed_rotation_deg = [180.0, 0.32, -90.0]
        self.threshold = 0.015

    def apply_threshold(self, value):
        """
        Avoids small values to be published as zero.
        """
        return 0.0 if abs(value) < self.threshold else value

    def listener_callback(self, msg):
        """
        Callback function to convert pose from robot's message to World's frame.
        """
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

        # Robot pose as transformation matrix
        T_robot = np.eye(4)
        T_robot[:3, :3] = R.from_quat(quat).as_matrix()
        T_robot[:3, 3] = pos

        # Fixed transform: robot → world
        T_fixed = np.eye(4)
        T_fixed[:3, :3] = R.from_euler('xyz', self.fixed_rotation_deg, degrees=True).as_matrix()
        T_fixed[:3, 3] = self.fixed_translation

        # Combined transform: world = T_fixed * robot
        T_world = T_fixed @ T_robot
        world_pos = T_world[:3, 3]
        world_rot = R.from_matrix(T_world[:3, :3]).as_euler('xyz', degrees=True)

        twist_msg = Twist()
        twist_msg.linear.x = self.apply_threshold(world_pos[0])
        twist_msg.linear.y = self.apply_threshold(world_pos[1])
        twist_msg.linear.z = self.apply_threshold(world_pos[2])

        twist_msg.angular.x = self.apply_threshold(world_rot[0])
        twist_msg.angular.y = self.apply_threshold(world_rot[1])
        twist_msg.angular.z = self.apply_threshold(world_rot[2])

        self.twist_pub.publish(twist_msg)
        self.msg_received = True

    def check_no_msg(self):
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
