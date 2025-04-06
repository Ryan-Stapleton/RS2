#include <rclcpp/rclcpp.hpp>
#include "trackit_robot_state/joint_limit_aggregator.h" // Use .h extension

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<JointLimitAggregator>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
