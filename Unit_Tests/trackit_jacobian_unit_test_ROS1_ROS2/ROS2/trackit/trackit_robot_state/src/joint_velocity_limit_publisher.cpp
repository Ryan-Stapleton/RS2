#include "trackit_robot_state/joint_velocity_limit_publisher.h"

#include <kdl/chain.hpp>
#include <kdl/segment.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <urdf/model.h>
#include <sensor_msgs/msg/joint_state.h>
#include "trackit_msgs/msg/joint_limits_stamped.h"

#include <chrono>
#include <cmath>

/**
 * @file joint_velocity_limit_publisher.cpp
 * @brief A node for publishing joint velocity limits such that joint position limits (plus buffer)
 *        will not be violated within a specified time_to_pos_limit.
 *
 * @param robot_description      URDF of the robot
 * @param control_frame          Name of the robot's control frame
 * @param tool_frame             Name of the tool/end-effector frame
 * @param time_to_pos_limit      Time allowed to travel before hitting position limit
 * @param position_limit_buffer  Additional buffer subtracted from upper limits or added to lower limits
 *
 * Subscribes to:
 *   - /joint_states: current robot joint positions
 *
 * Publishes on:
 *   - /joint_limits: velocity_upper and velocity_lower limits for each joint
 */

JointVelocityLimitPublisher::JointVelocityLimitPublisher()
: rclcpp::Node("trackit_joint_velocity_limit_publisher")
{
    // Declare and retrieve ROS2 parameters
    this->declare_parameter<std::string>("robot_description", "");
    this->declare_parameter<std::string>("control_frame", "");
    this->declare_parameter<std::string>("tool_frame", "");
    this->declare_parameter<double>("time_to_pos_limit", 5.0);
    this->declare_parameter<double>("position_limit_buffer", 0.1);

    this->get_parameter("robot_description", _robot_description);
    this->get_parameter("control_frame", _control_frame);
    this->get_parameter("tool_frame", _tool_frame);
    this->get_parameter("time_to_pos_limit", _time_to_pos_limit);
    this->get_parameter("position_limit_buffer", _position_limit_buffer);

    // Create the subscriber for /joint_states
    _sub_joint_state = this->create_subscription<sensor_msgs::msg::JointState>(
        "joint_states", 10,
        std::bind(&JointVelocityLimitPublisher::jointStateCallback, this, std::placeholders::_1));

    // Create the publisher for /joint_limits (velocity limits)
    _pub_joint_limits = this->create_publisher<trackit_msgs::msg::JointLimitsStamped>(
        "joint_limits", 10);

    // Parse URDF and build KDL tree/chain
    if (!kdl_parser::treeFromString(_robot_description, _kdl_tree)) {
        RCLCPP_ERROR(get_logger(),
            "Failed to construct KDL tree from robot_description. Shutting down.");
        rclcpp::shutdown();
        return;
    }

    if (!_kdl_tree.getChain(_control_frame, _tool_frame, _kdl_chain)) {
        RCLCPP_ERROR(get_logger(),
            "Failed to get KDL chain from '%s' to '%s'. Shutting down.",
            _control_frame.c_str(), _tool_frame.c_str());
        rclcpp::shutdown();
        return;
    }

    // Parse URDF model
    if (!_robot_model.initString(_robot_description)) {
        RCLCPP_ERROR(get_logger(), "Failed to parse URDF model. Shutting down.");
        rclcpp::shutdown();
        return;
    }

    // Extract all movable joints from the KDL chain, storing position limits
    int num_segments = _kdl_chain.getNrOfSegments();
    for (int i = 0; i < num_segments; i++) {
        const KDL::Segment& seg = _kdl_chain.getSegment(i);

        // Only consider joints that are not fixed
        if (seg.getJoint().getType() != KDL::Joint::None) {
            std::string joint_name = seg.getJoint().getName();
            _joint_names.push_back(joint_name);

            // Retrieve URDF joint
            auto urdf_joint = _robot_model.getJoint(joint_name);
            if (!urdf_joint || !urdf_joint->limits) {
                RCLCPP_WARN(get_logger(),
                    "Joint '%s' lacks valid URDF limits; defaulting to 0 for velocity calc.",
                    joint_name.c_str());
                _joint_position_upper_limits.push_back(0.0);
                _joint_position_lower_limits.push_back(0.0);
            } else {
                double upper = urdf_joint->limits->upper;
                double lower = urdf_joint->limits->lower;
                // Subtract/add buffer to position limits
                _joint_position_upper_limits.push_back(upper - _position_limit_buffer);
                _joint_position_lower_limits.push_back(lower + _position_limit_buffer);
            }

            // For each joint, initialize velocity_upper_limits and velocity_lower_limits to 0
            // The message arrays must be sized and pre-filled to match the number of joints
            _msg_joint_limits.velocity_upper_limits.data.push_back(0.0);
            _msg_joint_limits.velocity_lower_limits.data.push_back(0.0);
        }
    }

    // Set up the final message's frame and joint name list
    _msg_joint_limits.header.frame_id = _control_frame;
    _msg_joint_limits.joint_names = _joint_names;

    RCLCPP_INFO(get_logger(),
        "JointVelocityLimitPublisher loaded %zu movable joints. time_to_pos_limit = %.2f, buffer = %.2f",
        _joint_names.size(), _time_to_pos_limit, _position_limit_buffer);
}

/**
 * @brief Callback to compute allowable joint velocity so that position limits (with buffer) are not violated.
 * 
 * @param msg Current joint states of the robot (sensor_msgs::msg::JointState).
 */
void JointVelocityLimitPublisher::jointStateCallback(
    const sensor_msgs::msg::JointState::SharedPtr msg)
{
    // Sanity check to ensure we have matching joint counts
    if (msg->position.size() < _joint_names.size()) {
        RCLCPP_WARN(get_logger(),
            "Received JointState with fewer joints (%zu) than expected (%zu). Skipping.",
            msg->position.size(), _joint_names.size());
        return;
    }

    // For each joint, compute velocity limits
    for (size_t i = 0; i < _joint_names.size(); i++) {
        double current_pos = msg->position[i];

        // Calculate how far we can move in the positive direction
        double pos_dist_to_limit = _joint_position_upper_limits[i] - current_pos;
        double vel_upper = pos_dist_to_limit / _time_to_pos_limit;
        if (vel_upper < 0.0) {
            // If we've exceeded the upper limit (incl buffer), enforce no positive velocity
            vel_upper = 0.0;
        }

        // Calculate how far we can move in the negative direction
        double neg_dist_to_limit = _joint_position_lower_limits[i] - current_pos;
        double vel_lower = neg_dist_to_limit / _time_to_pos_limit;
        if (vel_lower > 0.0) {
            // If we've exceeded the lower limit (incl buffer), enforce no negative velocity
            vel_lower = 0.0;
        }

        // Assign results into the message
        _msg_joint_limits.velocity_upper_limits.data[i] = vel_upper;
        _msg_joint_limits.velocity_lower_limits.data[i] = vel_lower;
    }

    // Publish velocity limits
    _msg_joint_limits.header.stamp = this->now();
    _pub_joint_limits->publish(_msg_joint_limits);
}


