#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>
#include <trackit_msgs/msg/joint_limits_stamped.h>
#include "trackit_robot_state/static_joint_limit_publisher.h"  // Use .hpp for ROS2

static trackit_msgs::msg::JointLimitsStamped last_received_msg;
static bool msg_received = false;

void staticOutputCallback(const trackit_msgs::msg::JointLimitsStamped::SharedPtr msg) {
  last_received_msg = *msg;
  msg_received = true;
}

void spinForDuration(double seconds) {
  auto start = rclcpp::Clock().now();
  while ((rclcpp::Clock().now() - start).seconds() < seconds) {
    rclcpp::spin_some(rclcpp::Node::make_shared("dummy_spinning_node"));
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
}

class StaticJointLimitPublisherTest : public ::testing::Test {
protected:
  rclcpp::Node::SharedPtr test_node_;
  rclcpp::Subscription<trackit_msgs::msg::JointLimitsStamped>::SharedPtr sub_;
  std::shared_ptr<StaticJointLimitPublisher> staticPublisher_;

  virtual void SetUp() {
    test_node_ = rclcpp::Node::make_shared("test_static_joint_limit_publisher_node");

    // Minimal URDF with one joint "joint1"
    std::string urdf =
      "<robot name=\"test_robot\">"
      "  <link name=\"base_link\"/>"
      "  <link name=\"joint1_link\"/>"
      "  <joint name=\"joint1\" type=\"revolute\">"
      "    <parent link=\"base_link\"/>"
      "    <child link=\"joint1_link\"/>"
      "    <origin xyz=\"0 0 0\"/>"
      "    <limit lower=\"-2.0\" upper=\"2.0\" effort=\"10.0\" velocity=\"1.0\"/>"
      "  </joint>"
      "</robot>";
    
    // Create subscription to the topic that the publisher will publish on
    sub_ = test_node_->create_subscription<trackit_msgs::msg::JointLimitsStamped>(
      "joint_limits", 10, staticOutputCallback);

    // Allow time for subscription setup
    rclcpp::sleep_for(std::chrono::milliseconds(200));

    // Create the publisher (it inherits from Node, so no node handle needed)
    staticPublisher_ = std::make_shared<StaticJointLimitPublisher>();

    // Set up parameters on the publisher node
    staticPublisher_->declare_parameter("robot_description", urdf);
    staticPublisher_->declare_parameter("control_frame", "base_link");
    staticPublisher_->declare_parameter("tool_frame", "joint1_link");
    staticPublisher_->declare_parameter("rate", 100.0);
    
    // Allow time for initialization
    rclcpp::sleep_for(std::chrono::milliseconds(1000));
  }
};

TEST_F(StaticJointLimitPublisherTest, PublishesStaticJointLimitsCorrectly) {
  msg_received = false;
  last_received_msg = trackit_msgs::msg::JointLimitsStamped();

  // Spin to allow at least one timer callback to publish
  spinForDuration(1.0);

  EXPECT_TRUE(msg_received) << "StaticJointLimitPublisher did not publish a message within the expected time";

  ASSERT_EQ(last_received_msg.joint_names.size(), 1);
  ASSERT_EQ(last_received_msg.position_upper_limits.data.size(), 1);
  ASSERT_EQ(last_received_msg.position_lower_limits.data.size(), 1);
  ASSERT_EQ(last_received_msg.velocity_upper_limits.data.size(), 1);
  ASSERT_EQ(last_received_msg.velocity_lower_limits.data.size(), 1);
  ASSERT_EQ(last_received_msg.effort_limits.data.size(), 1);

  // Expected values from the URDF
  EXPECT_NEAR(last_received_msg.position_upper_limits.data[0], 2.0, 1e-2);
  EXPECT_NEAR(last_received_msg.position_lower_limits.data[0], -2.0, 1e-2);
  EXPECT_NEAR(last_received_msg.velocity_upper_limits.data[0], 1.0, 1e-2);
  EXPECT_NEAR(last_received_msg.velocity_lower_limits.data[0], -1.0, 1e-2);
  EXPECT_NEAR(last_received_msg.effort_limits.data[0], 10.0, 1e-2);

  EXPECT_EQ(last_received_msg.header.frame_id, "base_link");
}

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  testing::InitGoogleTest(&argc, argv);
  auto result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
