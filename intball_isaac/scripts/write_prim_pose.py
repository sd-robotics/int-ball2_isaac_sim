#!/usr/bin/python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import pandas as pd
import numpy as np
from scipy.spatial.transform import Rotation as R
import time

class CSVPathFollower(Node):
    def __init__(self, csv_file):
        super().__init__('csv_path_follower')

        # Create publisher for /cmd_vel
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)

        # Load CSV file
        self.path_df = pd.read_csv(csv_file)

        # Ensure correct data types
        self.path_df = self.path_df.astype(float)

        # Compute velocity commands
        self.velocities = self.compute_velocities()

        # Timer for publishing velocities
        self.index = 0
        self.timer = self.create_timer(1.0, self.publish_next_command)

    def compute_velocities(self):
        """
        Compute velocity (linear & angular) based on pose differences in CSV.
        """
        velocities = []

        for i in range(len(self.path_df) - 1):
            # Extract consecutive timestamps
            t1, t2 = self.path_df.iloc[i]["Timestep"], self.path_df.iloc[i + 1]["Timestep"]
            dt = t2 - t1  # Time difference

            # Extract position (x, y, z)
            pos1 = np.array([self.path_df.iloc[i][["x", "y", "z"]]])
            pos2 = np.array([self.path_df.iloc[i + 1][["x", "y", "z"]]])

            # Compute linear velocity
            linear_vel = (pos2 - pos1) / dt

            # Extract quaternions
            quat1 = R.from_quat(self.path_df.iloc[i][["qx", "qy", "qz", "qw"]])
            quat2 = R.from_quat(self.path_df.iloc[i + 1][["qx", "qy", "qz", "qw"]])

            # Compute angular velocity (change in orientation)
            delta_rotation = quat2 * quat1.inv()
            angular_vel = delta_rotation.as_rotvec() / dt  # Convert to angular velocity

            velocities.append((linear_vel.flatten(), angular_vel.flatten(), dt))

        return velocities

    def publish_next_command(self):
        """
        Publish the next velocity command in sequence.
        """
        if self.index >= len(self.velocities):
            self.get_logger().info("✅ Completed Path!")
            return

        linear_vel, angular_vel, duration = self.velocities[self.index]

        # Create Twist message
        twist = Twist()
        twist.linear.x, twist.linear.y, twist.linear.z = linear_vel
        twist.angular.x, twist.angular.y, twist.angular.z = angular_vel

        # Publish velocity command
        self.publisher.publish(twist)
        self.get_logger().info(f"📡 Published cmd_vel: {linear_vel}, Angular: {angular_vel}")

        # Wait before next command
        time.sleep(duration)
        self.index += 1  # Move to next command

def main(args=None):
    rclpy.init(args=args)

    csv_file_path = "~/portrs_ws/src/intball_isaac/src/pose_orientation_data.csv"
    node = CSVPathFollower(csv_file_path)

    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
