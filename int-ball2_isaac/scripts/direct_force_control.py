#!/usr/bin/python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Wrench

class IntBallForceController(Node):
    def __init__(self):
        super().__init__("intball_force_controller_node")

        # Parameters for scaling
        self.force_scale = 1.0   # Adjust force scaling
        self.torque_scale = 1.0   # Adjust torque scaling
        
        # Subscriber to the Joy topic
        self.subscription = self.create_subscription(
            Joy,
            "joy",
            self.joy_callback,
            10
        )

        # Publisher for force/torque
        self.wrench_publisher = self.create_publisher(Wrench, "/ctl/wrench", 10)

        self.get_logger().info("IntBall Force Controller Node Initialized")

    def joy_callback(self, msg):
        """
        Callback for handling joystick inputs and mapping them to Wrench messages.
        """
        wrench = Wrench()

        # Left stick controls force in X and Y
        wrench.force.x = msg.axes[0] * self.force_scale  # Left stick X
        wrench.force.y = -msg.axes[1] * self.force_scale  # Left stick Y

        # Right stick controls rotation (torque) in X-Y
        wrench.torque.x = msg.axes[4] * self.torque_scale  # Right stick Y (tilt forward/back)
        wrench.torque.y = msg.axes[3] * self.torque_scale  # Right stick X (tilt left/right)

        # L2 (Axis 2) and R2 (Axis 5) control force in Z
        L2 = (1 - msg.axes[2]) / 2  # Convert from [-1,1] to [0,1]
        R2 = (1 - msg.axes[5]) / 2  # Convert from [-1,1] to [0,1]
        
        if msg.buttons[12]:
            wrench.torque.z = (L2 - R2) * self.torque_scale  # R1 positive, L1 negative
        else:
            wrench.force.z = (R2 - L2) * self.force_scale  # R2 increases, L2 decreases

        # Publish the wrench message
        self.wrench_publisher.publish(wrench)
        self.get_logger().info(
            f"Published wrench: Force({wrench.force.x}, {wrench.force.y}, {wrench.force.z}) | "
            f"Torque({wrench.torque.x}, {wrench.torque.y}, {wrench.torque.z})"
        )

def main(args=None):
    rclpy.init(args=args)
    node = IntBallForceController()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Node interrupted, shutting down.")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
