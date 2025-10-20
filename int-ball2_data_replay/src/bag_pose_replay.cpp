#include "int-ball2_data_replay/bag_pose_replay.hpp"

#include <rclcpp_components/register_node_macro.hpp>

namespace bag_pose_replay
{

BagPoseReplay::BagPoseReplay(const rclcpp::NodeOptions & options)
: Node("bag_pose_replay_node", options),
  fixed_translation_({10.88492, -3.53022, 4.07888}),
  fixed_rotation_deg_({180.0, 0.32, -90.0}),
  msg_received_(false)
{
  // Subscriber
  nav_sub_ = this->create_subscription<ib2_interfaces::msg::Navigation>(
    "/sensor_fusion/navigation", 10,
    std::bind(&BagPoseReplay::RosBagPoseCallback, this, std::placeholders::_1));

  // Publisher
  twist_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/transform_twist", 10);

  // Timer
  timer_ = this->create_wall_timer(
    std::chrono::duration<double>(0.1),
    std::bind(&BagPoseReplay::PublishTwist, this));
}

BagPoseReplay::~BagPoseReplay()
{
  RCLCPP_INFO(this->get_logger(), "BagPoseReplay node is shutting down.");
}

void BagPoseReplay::RosBagPoseCallback(const ib2_interfaces::msg::Navigation::SharedPtr msg)
{
  Eigen::Vector3d pos(
    msg->pose.pose.position.x,
    msg->pose.pose.position.y,
    msg->pose.pose.position.z);

  Eigen::Quaterniond q(
    msg->pose.pose.orientation.w,
    msg->pose.pose.orientation.x,
    msg->pose.pose.orientation.y,
    msg->pose.pose.orientation.z);

  // Robot-frame transform
  Eigen::Matrix4d T_robot = Eigen::Matrix4d::Identity();
  T_robot.block<3, 3>(0, 0) = q.toRotationMatrix();
  T_robot.block<3, 1>(0, 3) = pos;

  // Fixed transform
  Eigen::Matrix4d T_fixed = Eigen::Matrix4d::Identity();
  Eigen::Matrix3d R_fixed =
    (Eigen::AngleAxisd(fixed_rotation_deg_[2] * M_PI / 180.0, Eigen::Vector3d::UnitZ()) *
     Eigen::AngleAxisd(fixed_rotation_deg_[1] * M_PI / 180.0, Eigen::Vector3d::UnitY()) *
     Eigen::AngleAxisd(fixed_rotation_deg_[0] * M_PI / 180.0, Eigen::Vector3d::UnitX()))
      .toRotationMatrix();

  T_fixed.block<3, 3>(0, 0) = R_fixed;
  T_fixed.block<3, 1>(0, 3) = Eigen::Vector3d(fixed_translation_[0], fixed_translation_[1], fixed_translation_[2]);

  // Transform robot pose to world frame
  Eigen::Matrix4d T_world = T_fixed * T_robot;
  Eigen::Vector3d world_pos = T_world.block<3, 1>(0, 3);

  // Convert to tf2 Quaternion for consistent RPY extraction
  Eigen::Matrix3d R_world = T_world.block<3, 3>(0, 0);
  tf2::Matrix3x3 tf_rot(
    R_world(0, 0), R_world(0, 1), R_world(0, 2),
    R_world(1, 0), R_world(1, 1), R_world(1, 2),
    R_world(2, 0), R_world(2, 1), R_world(2, 2)
  );

  double roll, pitch, yaw;
  tf_rot.getRPY(roll, pitch, yaw);  // matches ROS 'xyz' (roll, pitch, yaw)

  Eigen::Vector3d euler(roll, pitch, yaw);
  euler *= 180.0 / M_PI;

  // Filtering noise
  geometry_msgs::msg::Twist twist;

  if (first_msg_ || (world_pos - prev_position_).norm() > delta_threshold_) {
    twist.linear.x = world_pos.x();
    twist.linear.y = world_pos.y();
    twist.linear.z = world_pos.z();
    prev_position_ = world_pos;
  } else {
    twist.linear.x = prev_position_.x();
    twist.linear.y = prev_position_.y();
    twist.linear.z = prev_position_.z();
  }

  if (first_msg_ || (euler - prev_euler_).norm() > delta_threshold_) {
    twist.angular.x = euler.x();
    twist.angular.y = euler.y();
    twist.angular.z = euler.z();
    prev_euler_ = euler;
  } else {
    twist.angular.x = prev_euler_.x();
    twist.angular.y = prev_euler_.y();
    twist.angular.z = prev_euler_.z();
  }

  twist_pub_->publish(twist);
  msg_received_ = true;
  first_msg_ = false;
}

void BagPoseReplay::PublishTwist()
{
  if (!msg_received_) {
    geometry_msgs::msg::Twist twist;
    twist.linear.x = fixed_translation_[0];
    twist.linear.y = fixed_translation_[1];
    twist.linear.z = fixed_translation_[2];

    twist.angular.x = fixed_rotation_deg_[0];
    twist.angular.y = fixed_rotation_deg_[1];
    twist.angular.z = fixed_rotation_deg_[2];

    RCLCPP_WARN(this->get_logger(), "No Navigation msg received — publishing default twist");
    twist_pub_->publish(twist);
  } else {
    msg_received_ = false;
  }
}

}  // namespace bag_pose_replay

RCLCPP_COMPONENTS_REGISTER_NODE(bag_pose_replay::BagPoseReplay)
