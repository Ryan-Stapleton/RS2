#include <rclcpp/rclcpp.hpp>
#include "trackit_robot_state/static_joint_limit_publisher.h"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<StaticJointLimitPublisher>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
