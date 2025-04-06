#include "trackit_robot_state/static_joint_limit_publisher.h"

#include <kdl/chain.hpp>
#include <kdl/segment.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <urdf/model.h>
#include "trackit_msgs/msg/joint_limits_stamped.h"

#include <chrono>

/**
 * @file static_joint_limit_publisher.cpp
 * @brief A node for publishing static joint limits from a URDF file.
 *
 * @param robot_description  URDF description of the robot
 * @param control_frame      Name of the robot's control frame
 * @param tool_frame         Name of the tool/end effector frame
 * @param rate               Rate (in Hz) at which the node publishes
 *
 * Publishes:
 *   - /joint_limits: The static joint limits of the robot
 */

StaticJointLimitPublisher::StaticJointLimitPublisher()
: rclcpp::Node("trackit_joint_limit_publisher")
{
    // Declare and retrieve parameters
    this->declare_parameter<std::string>("robot_description", "");
    this->declare_parameter<std::string>("control_frame", "");
    this->declare_parameter<std::string>("tool_frame", "");
    this->declare_parameter<double>("rate", 500.0);

    this->get_parameter("robot_description", _robot_description);
    this->get_parameter("control_frame", _control_frame);
    this->get_parameter("tool_frame", _tool_frame);
    this->get_parameter("rate", _rate);

    // Create publisher
    //  - Publishes static joint limits on topic /joint_limits
    _pub_joint_limits = this->create_publisher<trackit_msgs::msg::JointLimitsStamped>(
        "joint_limits", 10);

    // Create timer to publish at specified rate
    auto period_ms = std::chrono::milliseconds(static_cast<int>(1000.0 / _rate));
    _timer = this->create_wall_timer(period_ms, std::bind(&StaticJointLimitPublisher::timerCallback, this));

    // Parse the URDF to build a KDL tree
    if (!kdl_parser::treeFromString(_robot_description, _kdl_tree)) {
        RCLCPP_ERROR(get_logger(),
            "Failed to construct KDL tree from robot_description. Shutting down.");
        rclcpp::shutdown();
        return;
    }

    // Extract chain from control_frame to tool_frame
    if (!_kdl_tree.getChain(_control_frame, _tool_frame, _kdl_chain)) {
        RCLCPP_ERROR(get_logger(),
            "Failed to get KDL chain from '%s' to '%s'. Shutting down.",
            _control_frame.c_str(), _tool_frame.c_str());
        rclcpp::shutdown();
        return;
    }

    // Parse the robot model from URDF
    if (!_robot_model.initString(_robot_description)) {
        RCLCPP_ERROR(get_logger(),
            "Failed to parse URDF model. Shutting down.");
        rclcpp::shutdown();
        return;
    }

    // Collect joint names and limits
    int num_segments = _kdl_chain.getNrOfSegments();
    std::vector<std::string> joint_names;
    std::vector<double> joint_position_upper_limits;
    std::vector<double> joint_position_lower_limits;
    std::vector<double> joint_velocity_upper_limits;
    std::vector<double> joint_velocity_lower_limits;
    std::vector<double> joint_effort_limits;

    for (int i = 0; i < num_segments; i++) {
        const KDL::Segment& seg = _kdl_chain.getSegment(i);
        // Only consider joints that are not fixed
        if (seg.getJoint().getType() != KDL::Joint::None) {
            std::string joint_name = seg.getJoint().getName();
            joint_names.push_back(joint_name);

            // Retrieve URDF joint
            auto urdf_joint = _robot_model.getJoint(joint_name);
            if (!urdf_joint || !urdf_joint->limits) {
                RCLCPP_WARN(get_logger(),
                    "Joint '%s' lacks valid URDF limits; defaulting to 0.",
                    joint_name.c_str());
                joint_position_upper_limits.push_back(0.0);
                joint_position_lower_limits.push_back(0.0);
                joint_velocity_upper_limits.push_back(0.0);
                joint_velocity_lower_limits.push_back(0.0);
                joint_effort_limits.push_back(0.0);
                continue;
            }

            double upper = urdf_joint->limits->upper;
            double lower = urdf_joint->limits->lower;
            double vel   = urdf_joint->limits->velocity;
            double eff   = urdf_joint->limits->effort;

            // Store the joint limits
            joint_position_upper_limits.push_back(upper);
            joint_position_lower_limits.push_back(lower);
            joint_velocity_upper_limits.push_back(vel);
            joint_velocity_lower_limits.push_back(-vel);
            joint_effort_limits.push_back(eff);
        }
    }

    // Prepare the message to publish
    _msg_joint_limits.header.frame_id = _control_frame;
    _msg_joint_limits.joint_names = joint_names;
    _msg_joint_limits.position_upper_limits.data.resize(joint_names.size());
    _msg_joint_limits.position_lower_limits.data.resize(joint_names.size());
    _msg_joint_limits.velocity_upper_limits.data.resize(joint_names.size());
    _msg_joint_limits.velocity_lower_limits.data.resize(joint_names.size());
    _msg_joint_limits.effort_limits.data.resize(joint_names.size());

    for (size_t i = 0; i < joint_names.size(); i++) {
        _msg_joint_limits.position_upper_limits.data[i] = joint_position_upper_limits[i];
        _msg_joint_limits.position_lower_limits.data[i] = joint_position_lower_limits[i];
        _msg_joint_limits.velocity_upper_limits.data[i] = joint_velocity_upper_limits[i];
        _msg_joint_limits.velocity_lower_limits.data[i] = joint_velocity_lower_limits[i];
        _msg_joint_limits.effort_limits.data[i]         = joint_effort_limits[i];
    }

    RCLCPP_INFO(get_logger(),
        "StaticJointLimitPublisher: Loaded %zu movable joints from URDF. Publishing at %.2f Hz.",
        joint_names.size(), _rate);
}

/**
 * @brief Timer callback to periodically publish static joint limits.
 */
void StaticJointLimitPublisher::timerCallback()
{
    _msg_joint_limits.header.stamp = this->now();
    _pub_joint_limits->publish(_msg_joint_limits);
}


