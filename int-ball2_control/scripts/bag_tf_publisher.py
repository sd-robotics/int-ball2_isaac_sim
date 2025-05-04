#!/usr/bin/python3

import rclpy
from rclpy.node import Node
from tf2_msgs.msg import TFMessage
from geometry_msgs.msg import Twist
from scipy.spatial.transform import Rotation as R

class TransformToTwist(Node):

    def __init__(self):
        super().__init__('transform_to_twist')

        self.subscription = self.create_subscription(
            TFMessage,
            '/bag_tf',
            self.listener_callback,
            10)

        self.twist_pub = self.create_publisher(Twist, '/transform_twist', 10)
        self.target_child_frame = "body"
        
        self.msg_received = False
        self.timer = self.create_timer(1.0, self.check_no_msg)

        # Set your initial/default values here
        self.default_translation = [10.88492, -3.53022, 4.07888]
        self.default_orientation_deg = [180.0, 0.32, -90.0]

    def listener_callback(self, msg):
        for transform in msg.transforms:
            if transform.child_frame_id != self.target_child_frame:
                continue

            trans = transform.transform.translation
            rot = transform.transform.rotation

            r = R.from_quat([rot.x, rot.y, rot.z, rot.w])
            roll, pitch, yaw = r.as_euler('xyz', degrees=True)

            twist_msg = Twist()
            twist_msg.linear.x = trans.x
            twist_msg.linear.y = trans.y
            twist_msg.linear.z = trans.z

            twist_msg.angular.x = roll
            twist_msg.angular.y = pitch
            twist_msg.angular.z = yaw

            self.twist_pub.publish(twist_msg)
            self.msg_received = True

    def check_no_msg(self):
        if not self.msg_received:
            # Publish initial/default Twist
            twist_msg = Twist()
            twist_msg.linear.x = self.default_translation[0]
            twist_msg.linear.y = self.default_translation[1]
            twist_msg.linear.z = self.default_translation[2]

            twist_msg.angular.x = self.default_orientation_deg[0]
            twist_msg.angular.y = self.default_orientation_deg[1]
            twist_msg.angular.z = self.default_orientation_deg[2]

            self.get_logger().warn('No /bag_tf received, publishing default transform')
            self.twist_pub.publish(twist_msg)
        else:
            # Reset the flag for the next check
            self.msg_received = False

def main(args=None):
    rclpy.init(args=args)
    node = TransformToTwist()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
