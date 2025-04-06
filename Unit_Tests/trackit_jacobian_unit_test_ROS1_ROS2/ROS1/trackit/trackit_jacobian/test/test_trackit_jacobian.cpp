#include <gtest/gtest.h>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <sensor_msgs/JointState.h>
#include <trackit_msgs/JacobianStamped.h>

bool jacobian_received = false;

// Callback to check if Jacobian message is received
void jacobianCallback(const trackit_msgs::JacobianStamped::ConstPtr& msg) {
    jacobian_received = true;
    
    // Print the Jacobian data when received
    ROS_INFO_STREAM("[TEST] Jacobian message received. Data size: " << msg->jacobian.data.size());
    ROS_INFO_STREAM("[TEST] Jacobian Data: ");
    for (size_t i = 0; i < msg->jacobian.data.size(); ++i) {
        ROS_INFO_STREAM("  " << msg->jacobian.data[i]);
    }

    EXPECT_GT(msg->jacobian.data.size(), 0);  // Ensure Jacobian data is non-empty
}

// Test case: Check if Jacobian is published
TEST(JacobianPublisherTest, JacobianComputation) {
    ros::NodeHandle nh;
    ros::Publisher joint_pub = nh.advertise<sensor_msgs::JointState>("joint_states", 10);
    ros::Subscriber jacobian_sub = nh.subscribe("jacobian", 10, jacobianCallback);

    // Check if robot_description is available
    std::string robot_desc;
    if (nh.getParam("/robot_description", robot_desc)) {
        ROS_INFO_STREAM("[TEST] robot_description param found and is " << robot_desc.length() << " characters long.");
    } else {
        ROS_ERROR("[TEST] robot_description param NOT found!");
    }

    ros::Duration(2.0).sleep();  // Allow connections to establish

    // Publish test joint state
    sensor_msgs::JointState joint_msg;
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
    joint_msg.header.stamp = ros::Time::now();

    joint_pub.publish(joint_msg);
    ros::spinOnce();

    ros::Duration(2.0).sleep();  // Give time for processing

    EXPECT_TRUE(jacobian_received);
}

// Main function for running tests
int main(int argc, char **argv) {
    ros::init(argc, argv, "jacobian_publisher_test");
    testing::InitGoogleTest(&argc, argv);
    ros::AsyncSpinner spinner(1);
    spinner.start();
    return RUN_ALL_TESTS();
}