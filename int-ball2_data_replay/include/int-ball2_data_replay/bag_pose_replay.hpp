#ifndef IB2_DATA_REPLAY__BAG_POSE_REPLAY_HPP_
#define IB2_DATA_REPLAY__BAG_POSE_REPLAY_HPP_

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <array>

#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include "ib2_interfaces/msg/navigation.hpp"
#include "int-ball2_data_replay/visibility_control.hpp"


namespace bag_pose_replay
{

class BagPoseReplay : public rclcpp::Node 
{
public:
  IB2_DATA_REPLAY_PUBLIC
  explicit BagPoseReplay(const rclcpp::NodeOptions & options);

  virtual ~BagPoseReplay();

private:
  void RosBagPoseCallback(const ib2_interfaces::msg::Navigation::SharedPtr msg);
  void PublishTwist();

  // Subscriber
  rclcpp::Subscription<ib2_interfaces::msg::Navigation>::SharedPtr nav_sub_;

  // Publisher
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr twist_pub_;

  // Timer
  rclcpp::TimerBase::SharedPtr timer_;

  std::array<double, 3> fixed_translation_;
  std::array<double, 3> fixed_rotation_deg_;
  Eigen::Vector3d prev_position_;
  Eigen::Vector3d prev_euler_;
  bool first_msg_ = true;
  double delta_threshold_ = 0.05;
  bool msg_received_;
};

}  // namespace bag_pose_replay

#endif  // IB2_DATA_REPLAY__BAG_POSE_REPLAY_HPP_