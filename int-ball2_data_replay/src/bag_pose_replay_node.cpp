#include "rclcpp/rclcpp.hpp"
#include "int-ball2_data_replay/bag_pose_replay.hpp"

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<bag_pose_replay::BagPoseReplay>(rclcpp::NodeOptions());
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
