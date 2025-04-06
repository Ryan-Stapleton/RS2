#include <gtest/gtest.h>
#include <ros/ros.h>
#include <geometry_msgs/TwistStamped.h>
#include <trackit_msgs/JointVelocityStamped.h>
#include <trackit_msgs/JacobianStamped.h>
#include <trackit_msgs/JointLimitsStamped.h>
#include <std_msgs/Float64MultiArray.h>

class TrackItIDKTest : public ::testing::Test {
protected:
    ros::NodeHandle nh;
    ros::Publisher twist_pub, jacobian_pub, limits_pub;
    ros::Subscriber joint_velocity_sub;
    trackit_msgs::JointVelocityStamped::ConstPtr received_joint_velocity;

    void SetUp() override {
        // Initialize ROS publishers
        twist_pub = nh.advertise<geometry_msgs::TwistStamped>("twist_in", 10);
        jacobian_pub = nh.advertise<trackit_msgs::JacobianStamped>("jacobian_inverse", 10);
        limits_pub = nh.advertise<trackit_msgs::JointLimitsStamped>("joint_limits", 10);

        // Initialize ROS subscriber
        joint_velocity_sub = nh.subscribe("joint_velocity_out", 10, &TrackItIDKTest::jointVelocityCallback, this);

        // Wait for connections
        ros::Duration(1.0).sleep();  
    }

    void jointVelocityCallback(const trackit_msgs::JointVelocityStamped::ConstPtr& msg) {
        received_joint_velocity = msg;
        ROS_INFO("Received joint velocity message");
    }
};

TEST_F(TrackItIDKTest, JointVelocityComputation) {
    // Publish mock Jacobian
    trackit_msgs::JacobianStamped jacobian_msg;
    jacobian_msg.joint_names = {"joint1", "joint2", "joint3"};
    jacobian_msg.jacobian.layout.dim.resize(2);
    jacobian_msg.jacobian.layout.dim[0].size = 3;
    jacobian_msg.jacobian.layout.dim[1].size = 6;
    jacobian_msg.jacobian.data = { 1.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                                   0.0, 1.0, 0.0, 0.0, 0.0, 0.0,
                                   0.0, 0.0, 1.0, 0.0, 0.0, 0.0 };
    ROS_INFO("Publishing jacobian message");
    jacobian_pub.publish(jacobian_msg);
    
    // Publish mock joint limits
    trackit_msgs::JointLimitsStamped limits_msg;
    limits_msg.joint_names = {"joint1", "joint2", "joint3"};
    limits_msg.velocity_upper_limits.data = {1.0, 1.0, 1.0};
    limits_msg.velocity_lower_limits.data = {-1.0, -1.0, -1.0};
    ROS_INFO("Publishing joint limits message");
    limits_pub.publish(limits_msg);
    
    
    // Publish a twist message
    geometry_msgs::TwistStamped twist_msg;
    twist_msg.header.frame_id = "base";
    twist_msg.twist.linear.x = 0.5;
    twist_msg.twist.linear.y = 0.5;
    twist_msg.twist.linear.z = 0.5;
    twist_msg.twist.angular.x = 0.1;
    twist_msg.twist.angular.y = 0.0;
    twist_msg.twist.angular.z = 0.0;
    ROS_INFO("Publishing twist message");
    twist_pub.publish(twist_msg);

    // Wait for message to be received
    ros::Time start_time = ros::Time::now();
    while (!received_joint_velocity && (ros::Time::now() - start_time).toSec() < 2.0) {
        ros::spinOnce();
        ros::Duration(0.1).sleep();
    }

    ASSERT_NE(received_joint_velocity, nullptr);
    EXPECT_EQ(received_joint_velocity->joint_names.size(), 3);
    EXPECT_NEAR(received_joint_velocity->joint_velocity.data[0], 0.5, 0.01);
    EXPECT_NEAR(received_joint_velocity->joint_velocity.data[1], 0.0, 0.01);
    EXPECT_NEAR(received_joint_velocity->joint_velocity.data[2], 0.0, 0.01);
}

// Run all tests
int main(int argc, char **argv) {
    ros::init(argc, argv, "test_trackit_idk");
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}