#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
import cv2

class DepthImageConverter(Node):
    """Node for conveting depth camera encoding from 32FC1 to 16UC1 for foxglove studio limitation

    Args:
        Node (_type_):
    """
    def __init__(self):
        super().__init__('depth_image_converter')
        
        # Initialize a CvBridge for converting between ROS and OpenCV images
        self.bridge = CvBridge()
        
        # Subscriber to the depth image topic
        self.subscription = self.create_subscription(
            Image,
            '/front_cam/depth',  # Your topic name
            self.depth_image_callback,
            10)
        
        # Publisher for the converted image
        self.publisher_ = self.create_publisher(
            Image,
            '/converted_depth_img',  # Topic for publishing the converted image
            10)
        
    def depth_image_callback(self, msg):
        try:
            # Convert the depth image to a numpy array (32FC1)
            depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='32FC1')
            
            # Convert the 32FC1 image to 16UC1 or mono16 (depends on your preference)
            # For example, we scale the 32FC1 image to 16-bit unsigned integer (16UC1)
            # Scaling might be required depending on the depth range.
            depth_image_scaled = np.clip(depth_image * 1000.0, 0, 65535).astype(np.uint16)  # Scale to millimeters (for example)

            # Convert back to ROS Image with 16UC1 encoding
            converted_msg = self.bridge.cv2_to_imgmsg(depth_image_scaled, encoding='16UC1')
            converted_msg.header = msg.header  # Preserve the original header
            
            # Publish the converted image
            self.publisher_.publish(converted_msg)
            
            # self.get_logger().info("Depth image converted and published")

        except CvBridgeError as e:
            self.get_logger().error(f"Failed to convert depth image: {str(e)}")


def main(args=None):
    rclpy.init(args=args)
    node = DepthImageConverter()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()