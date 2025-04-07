#include <gtest/gtest.h>
#include <ros/ros.h>
#include <geometry_msgs/TwistStamped.h>
#include <trackit_msgs/JointVelocityStamped.h>
#include <trackit_msgs/JacobianStamped.h>
#include <trackit_msgs/JointLimitsStamped.h>
#include <std_msgs/Float64MultiArray.h>
#include <urdf/model.h>

class TrackItIDKTest : public ::testing::Test {
protected:
    ros::NodeHandle nh;
    ros::Publisher twist_pub, limits_pub;
    ros::Subscriber joint_velocity_sub, jacobian_sub;

    trackit_msgs::JointVelocityStamped::ConstPtr received_joint_velocity;
    trackit_msgs::JacobianStamped::ConstPtr received_jacobian;
    trackit_msgs::JointLimitsStamped::ConstPtr received_limits;

    void SetUp() override {
        // Subscribe to the topics published by trackit_jacobian
        jacobian_sub = nh.subscribe("jacobian_inverse", 1, &TrackItIDKTest::jacobianCallback, this);

        // Publisher for Twist
        limits_pub = nh.advertise<trackit_msgs::JointLimitsStamped>("joint_limits", 10);
        twist_pub = nh.advertise<geometry_msgs::TwistStamped>("twist_in", 10);

        // Subscriber for joint velocity output
        joint_velocity_sub = nh.subscribe("joint_velocity_out", 1, &TrackItIDKTest::jointVelocityCallback, this);

        ros::Duration(1.0).sleep();  // Allow time for connections
    }

    void jacobianCallback(const trackit_msgs::JacobianStamped::ConstPtr& msg) {
        received_jacobian = msg;
        ROS_INFO("[TEST] Received Jacobian from trackit_jacobian");
    }

    void jointVelocityCallback(const trackit_msgs::JointVelocityStamped::ConstPtr& msg) {
        received_joint_velocity = msg;
        ROS_INFO("[TEST] Received Joint Velocity output from trackit_core");
    }

    // Function to publish joint limits
    void publishJointLimits() {
        trackit_msgs::JointLimitsStamped joint_limits_msg;
        joint_limits_msg.header.stamp = ros::Time::now();
    
        urdf::Model model;
        if (model.initParam("robot_description")) {
            for (const auto& joint_pair : model.joints_) {
                const urdf::JointSharedPtr& joint = joint_pair.second;
                if (joint->type == urdf::Joint::REVOLUTE || joint->type == urdf::Joint::PRISMATIC) {
                    // Fill the JointLimitsStamped message with the limits
                    joint_limits_msg.joint_names.push_back(joint->name);
                    joint_limits_msg.position_upper_limits.data.push_back(joint->limits->upper);
                    joint_limits_msg.position_lower_limits.data.push_back(joint->limits->lower);
                    joint_limits_msg.velocity_upper_limits.data.push_back(joint->limits->velocity);
                    joint_limits_msg.velocity_lower_limits.data.push_back(-joint->limits->velocity);  // Assuming negative velocity limit
                    joint_limits_msg.effort_limits.data.push_back(joint->limits->effort);

                    ROS_INFO("Joint: %s", joint->name.c_str());
                    ROS_INFO("  Lower limit: %f", joint->limits->lower);
                    ROS_INFO("  Upper limit: %f", joint->limits->upper);
                    ROS_INFO("  Velocity limit: %f", joint->limits->velocity);
                    ROS_INFO("  Effort limit: %f", joint->limits->effort);
                }
            }
        } else {
            ROS_ERROR("Failed to initialize URDF model.");
        }
    
        // Publish the joint limits
        limits_pub.publish(joint_limits_msg);
        ROS_INFO("[TEST] Published joint limits");
    }
};

TEST_F(TrackItIDKTest, JointVelocityComputation) {
    // Wait until both jacobian and joint limits have been received
    std::string robot_desc;
    if (nh.getParam("/robot_description", robot_desc)) {
        ROS_INFO_STREAM("[TEST] robot_description param found!");
    } else {
        ROS_ERROR("[TEST] robot_description param NOT found!");
    }

    ros::Duration(2.0).sleep(); // Give some time to receive messages

    ros::Time start_time = ros::Time::now();
    while ((!received_jacobian) && (ros::Time::now() - start_time).toSec() < 5.0) {
        ros::spinOnce();
        ros::Duration(0.1).sleep();
    }

    ASSERT_TRUE(received_jacobian) << "Jacobian not received from trackit_jacobian.";

    // Publish joint limits before testing joint velocity computation
    publishJointLimits();

    // Publish Twist input
    geometry_msgs::TwistStamped twist_msg;
    twist_msg.header.frame_id = "base";
    twist_msg.twist.linear.x = 0.5;
    twist_msg.twist.linear.y = 0.5;
    twist_msg.twist.linear.z = 0.0;
    twist_msg.twist.angular.x = 0.0;
    twist_msg.twist.angular.y = 0.0;
    twist_msg.twist.angular.z = 0.0;
    twist_pub.publish(twist_msg);

    // Wait for joint velocity output
    start_time = ros::Time::now();
    while (!received_joint_velocity && (ros::Time::now() - start_time).toSec() < 5.0) {
        ros::spinOnce();
        ros::Duration(0.1).sleep();
    }

    ASSERT_TRUE(received_joint_velocity) << "No joint velocity message received from trackit_core.";

    // Output result
    ROS_INFO("[TEST] Joint velocity output:");
    for (size_t i = 0; i < received_joint_velocity->joint_velocity.data.size(); ++i) {
        ROS_INFO("  Velocity[%zu]: %f", i, received_joint_velocity->joint_velocity.data[i]);
    }
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "test_trackit_idk");
    testing::InitGoogleTest(&argc, argv);
    ros::AsyncSpinner spinner(1);
    spinner.start();
    return RUN_ALL_TESTS();
}