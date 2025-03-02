#!/usr/bin/python3

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from geometry_msgs.msg import Twist, Wrench
from nav_msgs.msg import Odometry

class FeedbackController(Node):
    def __init__(self):
        super().__init__('feedback_controller')

        # Callback groups for concurrency
        self.cmd_vel_cb_group = MutuallyExclusiveCallbackGroup()
        self.odom_cb_group = MutuallyExclusiveCallbackGroup()

        # Subscriber to target velocity (cmd_vel)
        self.cmd_vel_sub = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10,
            callback_group=self.cmd_vel_cb_group
        )

        # Subscriber to odometry (current state)
        self.odom_sub = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10,
            callback_group=self.odom_cb_group
        )

        # Publisher for force & torque control
        self.force_pub = self.create_publisher(Wrench, '/ctl/wrench', 10)

        # Control gains
        self.Kp = 1.0  # Proportional gain
        self.Kd = 1.0   # Damping gain

        # Target and current velocities
        self.target_vel = Twist()
        self.current_vel = Twist()

    def cmd_vel_callback(self, msg):
        """ Receive target velocity from joystick input """
        self.target_vel = msg

    def odom_callback(self, msg):
        """ Receive current velocity from odometry and compute control force """
        # Extract linear and angular velocity from odometry
        self.current_vel.linear = msg.twist.twist.linear
        self.current_vel.angular = msg.twist.twist.angular

        # Compute force & torque using PD control
        force_x = self.Kp * (self.target_vel.linear.x - self.current_vel.linear.x) - self.Kd * self.current_vel.linear.x
        force_y = self.Kp * (self.target_vel.linear.y - self.current_vel.linear.y) - self.Kd * self.current_vel.linear.y
        force_z = self.Kp * (self.target_vel.linear.z - self.current_vel.linear.z) - self.Kd * self.current_vel.linear.z

        torque_x = self.Kp * (self.target_vel.angular.x - self.current_vel.angular.x) - self.Kd * self.current_vel.angular.x
        torque_y = self.Kp * (self.target_vel.angular.y - self.current_vel.angular.y) - self.Kd * self.current_vel.angular.y
        torque_z = self.Kp * (self.target_vel.angular.z - self.current_vel.angular.z) - self.Kd * self.current_vel.angular.z

        # Publish force and torque as Wrench message
        wrench_msg = Wrench()
        wrench_msg.force.x = force_y
        wrench_msg.force.y = force_x
        wrench_msg.force.z = force_z
        wrench_msg.torque.x = torque_x
        wrench_msg.torque.y = torque_y
        wrench_msg.torque.z = torque_z

        self.force_pub.publish(wrench_msg)
        self.get_logger().info(f"Force:{force_x}, {force_y}, {force_z}  Torque: {torque_x}, {torque_y}, {torque_z}")

def main(args=None):
    rclpy.init(args=args)
    node = FeedbackController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
