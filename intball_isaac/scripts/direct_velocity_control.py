#!/usr/bin/python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy


MAX_LIN_VEL = 0.5 # m/sec
MAX_ANG_VEL = 20.0 # deg/sec
class IntBallVelocityController(Node):
    def __init__(self):
        super().__init__('intball_velocity_controller_node')

        # Subscriptions
        self.subscription = self.create_subscription(
            Joy,
            '/joy',  # Joystick messages
            self.joy_callback,
            10)
        self.subscription  # Prevent unused variable warning

        # Publishers
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)

        # Initialize state
        self.z_mode = False  # Flag for z-velocity mode (triangle pressed)

        # Log initialization
        self.get_logger().info("IntBall Velocity Controller Node Initialized")
        
    def joy_callback(self, joy_msg):
        """
        Callback for handling joystick inputs and mapping them to Twist messages.
        """
        twist = Twist()

        # Axes and Buttons Mapping
        left_stick_x = -joy_msg.axes[0]  # Left stick horizontal (x)
        left_stick_y = joy_msg.axes[1]  # Left stick vertical (y)
        right_stick_x = -joy_msg.axes[3]  # Right stick horizontal (angular x)
        right_stick_y = -joy_msg.axes[4]  # Right stick vertical (angular y)
        L2 = joy_msg.axes[2]  # L2 trigger (rotation left)
        R2 = joy_msg.axes[5]  # R2 trigger (rotation right)
        triangle_button = joy_msg.buttons[2]  # Triangle button

        # Z-velocity mode toggle (Triangle button)
        if triangle_button:
            self.z_mode = True
            self.get_logger().info("Z-mode activated: Use left stick to control z velocity.")
        else:
            self.z_mode = False

        # Map joystick inputs to Twist
        if self.z_mode:
            # Left stick controls z velocity (up/down)
            twist.linear.z = left_stick_y * MAX_LIN_VEL  # Scale for z-axis control
        else:
            # Default mode: Control linear x/y and angular x/y
            twist.linear.x = left_stick_x * MAX_LIN_VEL  # Scale linear x
            twist.linear.y = left_stick_y * MAX_LIN_VEL  # Scale linear y
            twist.angular.x = right_stick_y * MAX_ANG_VEL  # Scale angular x
            twist.angular.y = right_stick_x * MAX_ANG_VEL  # Scale angular y
            twist.angular.z = (R2-L2) * MAX_ANG_VEL  # Positive angular z

        # Publish Twist message
        self.publisher.publish(twist)

        # Log Twist output (for debugging)
        self.get_logger().info(
            f"Published Twist: Linear({twist.linear.x}, {twist.linear.y}, {twist.linear.z}) | "
            f"Angular({twist.angular.x}, {twist.angular.y}, {twist.angular.z})"
        )


def main(args=None):
    rclpy.init(args=args)
    node = IntBallVelocityController()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Node interrupted, shutting down.")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
