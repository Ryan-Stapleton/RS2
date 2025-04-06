#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <trackit_msgs/msg/jacobian_stamped.hpp>
#include <fstream>
#include <string>

bool jacobian_received = false;

/**
 * @file test_trackit_jacobian.cpp
 *
 * This file contains the implementation of the TrackItJacobian class, which uses the kdl library
 * for calculating and publishing the Jacobian matrix for a robot.
 * The Jacobian matrix relates the end-effector's velocity to the joint velocities of the robot.
 *
 * @param robot_description: urdf of the robot in the ros parameter server
 *
 * Subscribers:
 *     jacobian: the jacobian of the robot for a given joint state
 *
 * Publishers:
 *     joint_states: the joint_state of the robot
 */

// Callback to check if Jacobian message is received
void jacobianCallback(const trackit_msgs::msg::JacobianStamped::SharedPtr msg) {
    jacobian_received = true;
    
    // Print the Jacobian data when received
    RCLCPP_INFO_STREAM(rclcpp::get_logger("test"), "[TEST] Jacobian message received. Data size: " << msg->jacobian.data.size());
    RCLCPP_INFO_STREAM(rclcpp::get_logger("test"), "[TEST] Jacobian Data: ");
    for (size_t i = 0; i < msg->jacobian.data.size(); ++i) {
        RCLCPP_INFO_STREAM(rclcpp::get_logger("test"), "  " << msg->jacobian.data[i]);
    }

    EXPECT_GT(msg->jacobian.data.size(), 0);  // Ensure Jacobian data is non-empty
}

// Test case: Check if Jacobian is published
TEST(JacobianPublisherTest, JacobianComputation) {
    rclcpp::Node::SharedPtr node = rclcpp::Node::make_shared("jacobian_publisher_test");
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_pub = node->create_publisher<sensor_msgs::msg::JointState>("joint_states", 10);
    rclcpp::Subscription<trackit_msgs::msg::JacobianStamped>::SharedPtr jacobian_sub = node->create_subscription<trackit_msgs::msg::JacobianStamped>(
        "jacobian", 10, jacobianCallback);
    
    // std::ifstream urdf_file("/home/ryan/ros2_ws/src/trackit/trackit_jacobian/urdf/ur3e.urdf");
    // std::string robot_desc((std::istreambuf_iterator<char>(urdf_file)), std::istreambuf_iterator<char>());
    // node->declare_parameter("robot_description", robot_desc);
    // std::string robot_desc;

    // Check if robot_description is available
    // if (node->get_parameter("robot_description", robot_desc)) {
    //     RCLCPP_INFO_STREAM(rclcpp::get_logger("test"), "[TEST] robot_description param found and is " << robot_desc.length() << " characters long.");
    // } else {
    //     RCLCPP_ERROR(rclcpp::get_logger("test"), "[TEST] robot_description param NOT found!");
    // }

    rclcpp::sleep_for(std::chrono::seconds(2));  // Allow connections to establish

    // Publish test joint state
    sensor_msgs::msg::JointState joint_msg;
    joint_msg.name = {
        "shoulder_pan_joint",
        "shoulder_lift_joint",
        "elbow_joint",
        "wrist_1_joint",
        "wrist_2_joint",
        "wrist_3_joint"
    };
    joint_msg.position = {0.0, -1.57, 1.57, 0.0, -1.57, 0.0};
    joint_msg.velocity = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    joint_msg.effort = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    joint_msg.header.stamp = node->now();

    joint_pub->publish(joint_msg);

    // Spin the node until the Jacobian message is received
    while (rclcpp::ok() && !jacobian_received) {
        rclcpp::spin_some(node);  // Process any incoming messages
        rclcpp::sleep_for(std::chrono::milliseconds(100));  // Allow some time for the callback to trigger
    }

    EXPECT_TRUE(jacobian_received);
}

// Main function for running tests
int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}