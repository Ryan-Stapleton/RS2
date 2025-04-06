#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <trackit_msgs/msg/joint_velocity_stamped.hpp>
#include <trackit_msgs/msg/jacobian_stamped.hpp>
#include <trackit_msgs/msg/joint_limits_stamped.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

class TrackItIDKTest : public ::testing::Test {
protected:
    rclcpp::Node::SharedPtr node;
    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr twist_pub;
    rclcpp::Publisher<trackit_msgs::msg::JacobianStamped>::SharedPtr jacobian_pub;
    rclcpp::Publisher<trackit_msgs::msg::JointLimitsStamped>::SharedPtr limits_pub;
    rclcpp::Subscription<trackit_msgs::msg::JointVelocityStamped>::SharedPtr joint_velocity_sub;
    trackit_msgs::msg::JointVelocityStamped::SharedPtr received_joint_velocity;

    void SetUp() override {
        rclcpp::init(0, nullptr);
        node = rclcpp::Node::make_shared("test_trackit_idk");

        // Initialize ROS publishers
        twist_pub = node->create_publisher<geometry_msgs::msg::TwistStamped>("twist_in", 10);
        jacobian_pub = node->create_publisher<trackit_msgs::msg::JacobianStamped>("jacobian_inverse", 10);
        limits_pub = node->create_publisher<trackit_msgs::msg::JointLimitsStamped>("joint_limits", 10);

        // Initialize ROS subscriber
        joint_velocity_sub = node->create_subscription<trackit_msgs::msg::JointVelocityStamped>(
            "joint_velocity_out", 10, std::bind(&TrackItIDKTest::jointVelocityCallback, this, std::placeholders::_1)
        );

        // Wait for connections (ROS2 doesn't have ros::Duration in the same way as ROS1)
        rclcpp::sleep_for(std::chrono::seconds(1));
    }

    void jointVelocityCallback(const trackit_msgs::msg::JointVelocityStamped::SharedPtr msg) {
        received_joint_velocity = msg;
        RCLCPP_INFO(node->get_logger(), "Received joint velocity message");
    }

    // Helper method to spin the node until the message is received
    void spinUntilReceived() {
        rclcpp::Rate rate(10);
        while (!received_joint_velocity) {
            rclcpp::spin_some(node);
            rate.sleep();
        }
    }
};

TEST_F(TrackItIDKTest, JointVelocityComputation) {
    // Publish mock Jacobian
    trackit_msgs::msg::JacobianStamped jacobian_msg;
    jacobian_msg.joint_names = {"joint1", "joint2", "joint3"};
    jacobian_msg.jacobian.layout.dim.resize(2);
    jacobian_msg.jacobian.layout.dim[0].size = 3;
    jacobian_msg.jacobian.layout.dim[1].size = 6;
    jacobian_msg.jacobian.data = { 1.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                                   0.0, 1.0, 0.0, 0.0, 0.0, 0.0,
                                   0.0, 0.0, 1.0, 0.0, 0.0, 0.0 };
    RCLCPP_INFO(node->get_logger(), "Publishing jacobian message");
    jacobian_pub->publish(jacobian_msg);
    
    // Publish mock joint limits
    trackit_msgs::msg::JointLimitsStamped limits_msg;
    limits_msg.joint_names = {"joint1", "joint2", "joint3"};
    limits_msg.velocity_upper_limits.data = {1.0, 1.0, 1.0};
    limits_msg.velocity_lower_limits.data = {-1.0, -1.0, -1.0};
    RCLCPP_INFO(node->get_logger(), "Publishing joint limits message");
    limits_pub->publish(limits_msg);
    
    // Publish a twist message
    geometry_msgs::msg::TwistStamped twist_msg;
    twist_msg.header.frame_id = "base";
    twist_msg.twist.linear.x = 0.5;
    twist_msg.twist.linear.y = 0.5;
    twist_msg.twist.linear.z = 0.5;
    twist_msg.twist.angular.x = 0.1;
    twist_msg.twist.angular.y = 0.0;
    twist_msg.twist.angular.z = 0.0;
    RCLCPP_INFO(node->get_logger(), "Publishing twist message");
    twist_pub->publish(twist_msg);

    // Wait for message to be received
    spinUntilReceived();

    ASSERT_NE(received_joint_velocity, nullptr);
    EXPECT_EQ(received_joint_velocity->joint_names.size(), 3);
    EXPECT_NEAR(received_joint_velocity->joint_velocity.data[0], 0.5, 0.01);
    EXPECT_NEAR(received_joint_velocity->joint_velocity.data[1], 0.0, 0.01);
    EXPECT_NEAR(received_joint_velocity->joint_velocity.data[2], 0.0, 0.01);
}

// Run all tests
int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}