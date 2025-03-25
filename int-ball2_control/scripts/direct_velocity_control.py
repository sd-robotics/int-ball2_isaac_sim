#!/usr/bin/python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy

class IntBall2VelocityController(Node):
    def __init__(self):
        super().__init__('ib2_vel_ctrl')

        # Parameters for scaling
        self.MAX_LIN_VEL = 0.5  # m/sec
        self.MAX_ANG_VEL = 20.0 # deg/sec

        # Subscriptions
        self.subscription = self.create_subscription(
            Joy,
            'joy',  # Joystick messages
            self.joy_callback,
            10)

        # Publishers
        self.twist_publisher = self.create_publisher(Twist, '/cmd_vel', 10)

        self.get_logger().info("Int-Ball2 Velocity Controller Node Initialized")
        
    def joy_callback(self, joy_msg):
        """
        Callback for handling joystick inputs and mapping them to Twist messages.
        """
        twist = Twist()

        # Left stick controls linear velocity in X and Y
        twist.linear.x = msg.axes[1] * self.MAX_LIN_VEL   # Left stick X
        twist.linear.y = msg.axes[0] * self.MAX_LIN_VEL   # Left stick Y

       # Right stick controls angular velocity in X and Y
        twist.angular.x = msg.axes[4] * self.MAX_ANG_VEL  # Right stick X
        twist.angular.y = msg.axes[3] * self.MAX_ANG_VEL  # Right stick Y

        # L2 (Axis 2) and R2 (Axis 5) control force in Z
        L2 = (1 - msg.axes[2]) / 2  # Convert from [-1, 1] to [0, 1]
        R2 = (1 - msg.axes[5]) / 2  # Convert from [-1, 1] to [0, 1]

        if msg.buttons[1]:
            twist.linear.z  = (R2 - L2) * self.MAX_LIN_VEL # R2 increases, L2 decreases
        elif msg.buttons[0]:
            twist.angular.z = (R2 - L2) * self.MAX_ANG_VEL # R2 increases, L2 decreases

        # Publish the twist message
        self.twist_publisher.publish(twist)
        self.get_logger().info(
            f"Published Twist: Linear({twist.linear.x}, {twist.linear.y}, {twist.linear.z}) | "
            f"Angular({twist.angular.x}, {twist.angular.y}, {twist.angular.z})"
        )

def main(args=None):
    rclpy.init(args=args)
    node = IntBall2VelocityController()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Node interrupted, shutting down.")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
