#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.h>
#include <trackit_msgs/msg/joint_limits_stamped.h>
#include "trackit_robot_state/joint_velocity_limit_publisher.h"

// Global variables to capture the published joint limits message
static trackit_msgs::msg::JointLimitsStamped last_received_msg;
static bool msg_received = false;

// Callback to capture the output from the joint velocity limit publisher
void velocityOutputCallback(const trackit_msgs::msg::JointLimitsStamped::SharedPtr msg) {
  last_received_msg = *msg;
  msg_received = true;
}

// Helper function to spin for a specified duration (in seconds)
void spinForDuration(rclcpp::Node::SharedPtr node, double seconds) {
  auto start = node->now();
  rclcpp::Rate rate(100); // 100 Hz
  while ((node->now() - start).seconds() < seconds) {
    rclcpp::spin_some(node);
    rate.sleep();
  }
}

class JointVelocityLimitPublisherTest : public ::testing::Test {
protected:
  rclcpp::Node::SharedPtr node_;
  rclcpp::Subscription<trackit_msgs::msg::JointLimitsStamped>::SharedPtr sub_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr pub_;
  std::shared_ptr<JointVelocityLimitPublisher> velocityPublisher_;

  void SetUp() override {
    node_ = rclcpp::Node::make_shared("test_joint_velocity_limit_publisher");
    
    // Minimal URDF with one joint "joint1"
    std::string urdf = 
      "<robot name=\"test_robot\">"
        "<link name=\"base_link\"/>"
        "<link name=\"joint1_link\"/>"
        "<joint name=\"joint1\" type=\"revolute\">"
          "<parent link=\"base_link\"/>"
          "<child link=\"joint1_link\"/>"
          "<origin xyz=\"0 0 0\"/>"
          "<limit lower=\"-2.0\" upper=\"2.0\" effort=\"10.0\" velocity=\"1.0\"/>"
        "</joint>"
      "</robot>";
    
    // Declare and set parameters
    node_->declare_parameter("robot_description", urdf);
    node_->declare_parameter("control_frame", "base_link");
    node_->declare_parameter("tool_frame", "joint1_link");
    node_->declare_parameter("time_to_pos_limit", 5.0);
    node_->declare_parameter("position_limit_buffer", 0.1);
    
    node_->set_parameter(rclcpp::Parameter("robot_description", urdf));
    node_->set_parameter(rclcpp::Parameter("control_frame", "base_link"));
    node_->set_parameter(rclcpp::Parameter("tool_frame", "joint1_link"));
    node_->set_parameter(rclcpp::Parameter("time_to_pos_limit", 5.0));
    node_->set_parameter(rclcpp::Parameter("position_limit_buffer", 0.1));

    // Create subscriber for velocity limits output on topic "joint_limits"
    sub_ = node_->create_subscription<trackit_msgs::msg::JointLimitsStamped>(
              "joint_limits", 10, velocityOutputCallback);
    
    // Create publisher for joint state messages on topic "joint_states"
    pub_ = node_->create_publisher<sensor_msgs::msg::JointState>("joint_states", 10);
    
    // Allow time for parameters to propagate and for connections to establish
    spinForDuration(node_, 0.5);
    
    // Instantiate the JointVelocityLimitPublisher AFTER parameters are set
    velocityPublisher_ = std::make_shared<JointVelocityLimitPublisher>();
    velocityPublisher_->declare_parameter("robot_description", urdf);
    velocityPublisher_->declare_parameter("control_frame", "base_link");
    velocityPublisher_->declare_parameter("tool_frame", "joint1_link");
    velocityPublisher_->declare_parameter("time_to_pos_limit", 1.0);
    velocityPublisher_->declare_parameter("position_limit_buffer", 0.1);
    velocityPublisher_->declare_parameter("rate", 100.0);
    // Allow additional time for node initialization and timer callbacks
    spinForDuration(node_, 1.0);
  }
};

TEST_F(JointVelocityLimitPublisherTest, ComputesCorrectVelocityLimitsNormalCase) {
  // Reset flag and output
  msg_received = false;
  last_received_msg = trackit_msgs::msg::JointLimitsStamped();
  
  // In our test URDF, effective joint limits for joint1 are:
  // Upper: 2.0 - 0.1 = 1.9, Lower: -2.0 + 0.1 = -1.9.
  // For joint1 at position = 1.0:
  // Expected velocity_upper = (1.9 - 1.0) / 5.0 = 0.18.
  // Expected velocity_lower = (-1.9 - 1.0) / 5.0 = -0.58.
  auto js = std::make_shared<sensor_msgs::msg::JointState>();
  js->header.stamp = node_->now();
  js->name.push_back("joint1");
  js->position.push_back(1.0);

  pub_->publish(*js);
  spinForDuration(node_, 0.5); // allow processing
  
  EXPECT_TRUE(msg_received) << "Did not receive a velocity limit message.";
  ASSERT_EQ(last_received_msg.joint_names.size(), 1);
  ASSERT_EQ(last_received_msg.velocity_upper_limits.data.size(), 1);
  ASSERT_EQ(last_received_msg.velocity_lower_limits.data.size(), 1);

  EXPECT_NEAR(last_received_msg.velocity_upper_limits.data[0], 0.18, 1e-2);
  EXPECT_NEAR(last_received_msg.velocity_lower_limits.data[0], -0.58, 1e-2);
}

TEST_F(JointVelocityLimitPublisherTest, ClampsVelocityLimitsEdgeCase) {
  // Reset flag and output
  msg_received = false;
  last_received_msg = trackit_msgs::msg::JointLimitsStamped();
  
  auto js = std::make_shared<sensor_msgs::msg::JointState>();
  js->header.stamp = node_->now();
  js->name.push_back("joint1");
  
  // Test upper limit edge case: joint1 at position 2.0,
  // expected velocity_upper = (1.9 - 2.0) / 5.0 = -0.1, but clamped to 0.0.
  js->position.push_back(2.0);
  pub_->publish(*js);
  spinForDuration(node_, 0.5);
  EXPECT_TRUE(msg_received) << "Upper limit case: No message received.";
  ASSERT_EQ(last_received_msg.velocity_upper_limits.data.size(), 1);
  EXPECT_NEAR(last_received_msg.velocity_upper_limits.data[0], 0.0, 1e-2);
  
  // Test lower limit edge case: joint1 at position -2.0,
  // expected velocity_lower = (-1.9 - (-2.0)) / 5.0 = 0.02, but clamped to 0.0.
  msg_received = false;
  last_received_msg = trackit_msgs::msg::JointLimitsStamped();
  js->position[0] = -2.0;
  pub_->publish(*js);
  spinForDuration(node_, 0.5);
  EXPECT_TRUE(msg_received) << "Lower limit case: No message received.";
  ASSERT_EQ(last_received_msg.velocity_lower_limits.data.size(), 1);
  EXPECT_NEAR(last_received_msg.velocity_lower_limits.data[0], 0.0, 1e-2);
}

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
