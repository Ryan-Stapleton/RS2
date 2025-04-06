#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>
#include <trackit_msgs/msg/joint_limits_stamped.h>
#include <std_msgs/msg/float64_multi_array.h>
#include "trackit_robot_state/joint_limit_aggregator.h"


// Global variables to capture output from the aggregator
static trackit_msgs::msg::JointLimitsStamped last_received_msg;
static bool msg_received = false;

// Debug callback for the aggregator output topic
void outputCallback(const trackit_msgs::msg::JointLimitsStamped::SharedPtr msg) {
  last_received_msg = *msg;
  msg_received = true;
  RCLCPP_INFO(rclcpp::get_logger("test"), "Received aggregated message:");
  RCLCPP_INFO(rclcpp::get_logger("test"), "  Joint count: %zu", msg->joint_names.size());
  if (!msg->position_upper_limits.data.empty())
    RCLCPP_INFO(rclcpp::get_logger("test"), "  position_upper: %f", msg->position_upper_limits.data[0]);
  if (!msg->position_lower_limits.data.empty())
    RCLCPP_INFO(rclcpp::get_logger("test"), "  position_lower: %f", msg->position_lower_limits.data[0]);
  if (!msg->velocity_upper_limits.data.empty())
    RCLCPP_INFO(rclcpp::get_logger("test"), "  velocity_upper: %f", msg->velocity_upper_limits.data[0]);
  if (!msg->velocity_lower_limits.data.empty())
    RCLCPP_INFO(rclcpp::get_logger("test"), "  velocity_lower: %f", msg->velocity_lower_limits.data[0]);
  if (!msg->effort_limits.data.empty())
    RCLCPP_INFO(rclcpp::get_logger("test"), "  effort: %f", msg->effort_limits.data[0]);
}

// Helper function to spin for a specified duration (in seconds)
template <typename NodeT>
void spinForDuration(std::shared_ptr<NodeT> node, double seconds) {
  auto start = node->now();
  rclcpp::Rate rate(100); // 100 Hz
  while ((node->now() - start).seconds() < seconds) {
    rclcpp::spin_some(node);
    rate.sleep();
  }
}

class JointLimitAggregatorTest : public ::testing::Test {
protected:
  rclcpp::Node::SharedPtr node_;
  rclcpp::Subscription<trackit_msgs::msg::JointLimitsStamped>::SharedPtr sub_;
  rclcpp::Publisher<trackit_msgs::msg::JointLimitsStamped>::SharedPtr pub_;
  std::shared_ptr<JointLimitAggregator> aggregator_;

  void SetUp() override {
    node_ = rclcpp::Node::make_shared("test_joint_limit_aggregator");
    RCLCPP_INFO(node_->get_logger(), "Setting up JointLimitAggregatorTest");

    // Minimal URDF with one revolute joint "joint1"
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
    node_->declare_parameter("msg_timeout", 1.0);
    node_->declare_parameter("rate", 100.0);

    // Subscribe to output topic ("joint_limits_out")
    sub_ = node_->create_subscription<trackit_msgs::msg::JointLimitsStamped>(
              "joint_limits_out", 10, outputCallback);
    RCLCPP_INFO(node_->get_logger(), "Subscribed to joint_limits_out.");

    // Create publisher for input messages ("joint_limits_in")
    pub_ = node_->create_publisher<trackit_msgs::msg::JointLimitsStamped>("joint_limits_in", 10);
    RCLCPP_INFO(node_->get_logger(), "Advertised joint_limits_in.");

    // Allow time for parameter propagation and connections
    spinForDuration(node_, 1.0);
    RCLCPP_INFO(node_->get_logger(), "Spun for 1.0 seconds to allow connections.");

    // Create the aggregator without passing a node (it inherits from Node)
    aggregator_ = std::make_shared<JointLimitAggregator>();
    
    // Set the parameters on the aggregator node
    aggregator_->declare_parameter("robot_description", urdf);
    aggregator_->declare_parameter("control_frame", "base_link");
    aggregator_->declare_parameter("tool_frame", "joint1_link");
    aggregator_->declare_parameter("msg_timeout", 1.0);
    aggregator_->declare_parameter("rate", 100.0);
    
    RCLCPP_INFO(node_->get_logger(), "Aggregator node instantiated.");

    // Allow time for aggregator initialization and timer callbacks
    // We need to spin both nodes
    spinForDuration(node_, 0.5);
    spinForDuration(aggregator_, 0.5);
    RCLCPP_INFO(node_->get_logger(), "Completed test setup; beginning tests.");
  }
};

TEST_F(JointLimitAggregatorTest, AggregatesAndSelectsMostRestrictive) {
  RCLCPP_INFO(rclcpp::get_logger("test"), "Running AggregatesAndSelectsMostRestrictive test");
  msg_received = false;
  last_received_msg = trackit_msgs::msg::JointLimitsStamped();

  // Create first message (msg1) with URDF default limits:
  // position: [2.0, -2.0], velocity: [1.0, -1.0], effort: 10.0
  auto msg1 = std::make_shared<trackit_msgs::msg::JointLimitsStamped>();
  msg1->header.stamp = node_->now();
  msg1->joint_names.push_back("joint1");
  std_msgs::msg::Float64MultiArray arr;
  arr.data.push_back(2.0);
  msg1->position_upper_limits = arr;
  arr.data.clear();
  arr.data.push_back(-2.0);
  msg1->position_lower_limits = arr;
  arr.data.clear();
  arr.data.push_back(1.0);
  msg1->velocity_upper_limits = arr;
  arr.data.clear();
  arr.data.push_back(-1.0);
  msg1->velocity_lower_limits = arr;
  arr.data.clear();
  arr.data.push_back(10.0);
  msg1->effort_limits = arr;

  RCLCPP_INFO(rclcpp::get_logger("test"), "Publishing first message (msg1) with URDF values.");
  pub_->publish(*msg1);
  spinForDuration(node_, 0.2);

  // Create second message (msg2) with more restrictive limits:
  // position: [1.5, -1.0], velocity: [0.8, -0.8], effort: 8.0
  auto msg2 = std::make_shared<trackit_msgs::msg::JointLimitsStamped>();
  msg2->header.stamp = node_->now();
  msg2->joint_names.push_back("joint1");
  arr.data.clear();
  arr.data.push_back(1.5);
  msg2->position_upper_limits = arr;
  arr.data.clear();
  arr.data.push_back(-1.0);
  msg2->position_lower_limits = arr;
  arr.data.clear();
  arr.data.push_back(0.8);
  msg2->velocity_upper_limits = arr;
  arr.data.clear();
  arr.data.push_back(-0.8);
  msg2->velocity_lower_limits = arr;
  arr.data.clear();
  arr.data.push_back(8.0);
  msg2->effort_limits = arr;

  RCLCPP_INFO(rclcpp::get_logger("test"), "Publishing second message (msg2) with more restrictive values.");
  pub_->publish(*msg2);
  spinForDuration(node_, 0.1);

  spinForDuration(node_, 0.3);

  EXPECT_TRUE(msg_received) << "Aggregator did not publish an aggregated message.";

  // Expected aggregated values should be the more restrictive ones from msg2:
  // position_upper: 1.5, position_lower: -1.0, velocity_upper: 0.8, velocity_lower: -0.8, effort: 8.0
  ASSERT_EQ(last_received_msg.joint_names.size(), 1);
  ASSERT_EQ(last_received_msg.position_upper_limits.data.size(), 1);
  EXPECT_NEAR(last_received_msg.position_upper_limits.data[0], 1.5, 1e-2);
  ASSERT_EQ(last_received_msg.position_lower_limits.data.size(), 1);
  EXPECT_NEAR(last_received_msg.position_lower_limits.data[0], -1.0, 1e-2);
  ASSERT_EQ(last_received_msg.velocity_upper_limits.data.size(), 1);
  EXPECT_NEAR(last_received_msg.velocity_upper_limits.data[0], 0.8, 1e-2);
  ASSERT_EQ(last_received_msg.velocity_lower_limits.data.size(), 1);
  EXPECT_NEAR(last_received_msg.velocity_lower_limits.data[0], -0.8, 1e-2);
  ASSERT_EQ(last_received_msg.effort_limits.data.size(), 1);
  EXPECT_NEAR(last_received_msg.effort_limits.data[0], 8.0, 1e-2);
}

TEST_F(JointLimitAggregatorTest, HandlesStaleMessagesAndUsesURDFValues) {
  RCLCPP_INFO(rclcpp::get_logger("test"), "Running HandlesStaleMessagesAndUsesURDFValues test");
  msg_received = false;
  last_received_msg = trackit_msgs::msg::JointLimitsStamped();

  // Create a stale message (timestamp set to 0)
  auto stale_msg = std::make_shared<trackit_msgs::msg::JointLimitsStamped>();
  stale_msg->header.stamp = rclcpp::Time(0, 0, RCL_ROS_TIME);
  stale_msg->joint_names.push_back("joint1");
  std_msgs::msg::Float64MultiArray arr;
  arr.data.push_back(1.0);  // should be ignored
  stale_msg->position_upper_limits = arr;
  arr.data.clear();
  arr.data.push_back(-0.5);
  stale_msg->position_lower_limits = arr;
  arr.data.clear();
  arr.data.push_back(0.5);
  stale_msg->velocity_upper_limits = arr;
  arr.data.clear();
  arr.data.push_back(-0.5);
  stale_msg->velocity_lower_limits = arr;
  arr.data.clear();
  arr.data.push_back(5.0);
  stale_msg->effort_limits = arr;

  RCLCPP_INFO(rclcpp::get_logger("test"), "Publishing stale message.");
  pub_->publish(*stale_msg);
  spinForDuration(node_, 0.1);

  spinForDuration(node_, 0.3);

  EXPECT_TRUE(msg_received) << "Aggregator did not fall back to URDF values on stale message.";

  // Expected URDF values: position: [2.0, -2.0], velocity: [1.0, -1.0], effort: 10.0
  ASSERT_EQ(last_received_msg.joint_names.size(), 1);
  ASSERT_EQ(last_received_msg.position_upper_limits.data.size(), 1);
  EXPECT_NEAR(last_received_msg.position_upper_limits.data[0], 2.0, 1e-2);
  ASSERT_EQ(last_received_msg.position_lower_limits.data.size(), 1);
  EXPECT_NEAR(last_received_msg.position_lower_limits.data[0], -2.0, 1e-2);
  ASSERT_EQ(last_received_msg.velocity_upper_limits.data.size(), 1);
  EXPECT_NEAR(last_received_msg.velocity_upper_limits.data[0], 1.0, 1e-2);
  ASSERT_EQ(last_received_msg.velocity_lower_limits.data.size(), 1);
  EXPECT_NEAR(last_received_msg.velocity_lower_limits.data[0], -1.0, 1e-2);
  ASSERT_EQ(last_received_msg.effort_limits.data.size(), 1);
  EXPECT_NEAR(last_received_msg.effort_limits.data[0], 10.0, 1e-2);
}

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
