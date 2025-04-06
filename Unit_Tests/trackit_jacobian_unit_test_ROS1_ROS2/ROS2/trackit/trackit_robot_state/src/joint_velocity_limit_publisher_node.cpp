#include <rclcpp/rclcpp.hpp>
#include "trackit_robot_state/joint_velocity_limit_publisher.h" // Use .h extension

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<JointVelocityLimitPublisher>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
