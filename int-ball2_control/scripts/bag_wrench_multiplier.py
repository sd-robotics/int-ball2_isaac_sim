#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from geometry_msgs.msg import WrenchStamped

class WrenchScaler(Node):

    def __init__(self):
        super().__init__('wrench_scaler')

        self.callback_group = ReentrantCallbackGroup()

        self.subscription = self.create_subscription(
            WrenchStamped,
            '/bag/ctl/wrench',
            self.listener_callback,
            10,
            callback_group=self.callback_group)

        self.publisher = self.create_publisher(WrenchStamped, '/ctl/wrench', 10)

        self.timer = self.create_timer(
            1.0,
            self.check_no_msg,
            callback_group=self.callback_group)

        self.force_multiplier = 1.0
        self.torque_multiplier = 1.0
        self.threshold = 0.015
        self.msg_received = False

    def low_pass(self, value, threshold):
        return 0.0 if abs(value) < threshold else value

    def listener_callback(self, msg):
        scaled_msg = WrenchStamped()
        scaled_msg.header = msg.header

        # Scale and apply threshold to forces
        scaled_msg.wrench.force.x = msg.wrench.force.x * self.force_multiplier * 7
        scaled_msg.wrench.force.y = self.low_pass(msg.wrench.force.y, 0.03) * 5
        scaled_msg.wrench.force.z = (msg.wrench.force.z) * 10

        # Scale and apply threshold to torques
        # scaled_msg.wrench.torque.x = self.low_pass(msg.wrench.torque.x) * self.torque_multiplier
        # scaled_msg.wrench.torque.y = self.low_pass(msg.wrench.torque.y) * self.torque_multiplier
        # scaled_msg.wrench.torque.z = self.low_pass(msg.wrench.torque.z) * self.torque_multiplier
        scaled_msg.wrench.torque.x = msg.wrench.torque.x * self.torque_multiplier
        scaled_msg.wrench.torque.y = msg.wrench.torque.y * self.torque_multiplier
        scaled_msg.wrench.torque.z = msg.wrench.torque.z * self.torque_multiplier * 20

        self.publisher.publish(scaled_msg)
        self.msg_received = True

    def check_no_msg(self):
        if not self.msg_received:
            zero_msg = WrenchStamped()
            zero_msg.header.stamp = self.get_clock().now().to_msg()
            zero_msg.header.frame_id = "body"
            self.get_logger().warn('No wrench msg received — publishing zero wrench')
            self.publisher.publish(zero_msg)

        self.msg_received = False

def main(args=None):
    rclpy.init(args=args)
    node = WrenchScaler()

    from rclpy.executors import MultiThreadedExecutor
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    executor.spin()

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
