#include "trackit_robot_state/joint_limit_aggregator.h"

#include <kdl/chain.hpp>
#include <kdl/segment.hpp>
#include <cmath>
#include <chrono>

/**
 * @brief Constructor for JointLimitAggregator node.
 * Loads parameters, parses URDF, builds KDL chain, and sets up aggregator data structures.
 */
JointLimitAggregator::JointLimitAggregator()
: rclcpp::Node("joint_limit_aggregator")
{
    // Declare parameters
    this->declare_parameter<std::string>("robot_description", "");
    this->declare_parameter<std::string>("control_frame", "");
    this->declare_parameter<std::string>("tool_frame", "");
    this->declare_parameter<double>("msg_timeout", 0.1);
    this->declare_parameter<double>("rate", 50.0); // default 50 Hz

    // Read parameters
    this->get_parameter("robot_description", _robot_description);
    this->get_parameter("control_frame", _control_frame);
    this->get_parameter("tool_frame", _tool_frame);
    this->get_parameter("msg_timeout", _msg_timeout);
    this->get_parameter("rate", _rate);

    // Create publisher and subscriber
    //  - topic names can be changed to match your desired naming
    _pub_joint_limits = this->create_publisher<trackit_msgs::msg::JointLimitsStamped>(
        "joint_limits_out", 10);

    _sub_joint_limits = this->create_subscription<trackit_msgs::msg::JointLimitsStamped>(
        "joint_limits_in", 10,
        std::bind(&JointLimitAggregator::jointLimitsCallback, this, std::placeholders::_1));

    // Create timer to publish final aggregated limits at desired rate
    auto period = std::chrono::milliseconds(static_cast<int>(1000.0 / _rate));
    _timer = this->create_wall_timer(
        period, std::bind(&JointLimitAggregator::timerCallback, this));

    // Attempt to parse the robot_description as a KDL tree
    if (!kdl_parser::treeFromString(_robot_description, _kdl_tree)) {
        RCLCPP_ERROR(get_logger(),
            "Failed to construct KDL Tree from robot_description. Shutting down aggregator.");
        rclcpp::shutdown();
        return;
    }

    // Attempt to extract a chain from _control_frame to _tool_frame
    if (!_kdl_tree.getChain(_control_frame, _tool_frame, _kdl_chain)) {
        RCLCPP_ERROR(get_logger(),
            "Failed to get KDL chain from '%s' to '%s'. Shutting down aggregator.",
            _control_frame.c_str(), _tool_frame.c_str());
        rclcpp::shutdown();
        return;
    }

    // Parse URDF model
    if (!_robot_model.initString(_robot_description)) {
        RCLCPP_ERROR(get_logger(), 
            "Failed to parse URDF model. Shutting down aggregator.");
        rclcpp::shutdown();
        return;
    }

    // Build arrays from the KDL chain
    int num_segments = _kdl_chain.getNrOfSegments();
    for (int i = 0; i < num_segments; ++i) {
        const KDL::Segment& segment = _kdl_chain.getSegment(i);
        if (segment.getJoint().getType() == KDL::Joint::None) {
            // Skip fixed joints
            continue;
        }

        // We have a movable joint
        std::string joint_name = segment.getJoint().getName();
        _joint_names.push_back(joint_name);

        // Extract URDF limits
        auto urdf_joint = _robot_model.getJoint(joint_name);
        if (!urdf_joint || !urdf_joint->limits) {
            // If no valid URDF limit, skip or handle gracefully
            RCLCPP_WARN(get_logger(), 
                "Joint '%s' has no valid URDF limit. Using fallback zeroes.", joint_name.c_str());
            _joint_position_upper_urdf_limits.push_back(0.0);
            _joint_position_lower_urdf_limits.push_back(0.0);
            _joint_velocity_urdf_limits.push_back(0.0);
            _joint_effort_urdf_limits.push_back(0.0);

            _joint_position_upper_limits.push_back(0.0);
            _joint_position_lower_limits.push_back(0.0);
            _joint_velocity_upper_limits.push_back(0.0);
            _joint_velocity_lower_limits.push_back(0.0);
            _joint_effort_limits.push_back(0.0);

            continue;
        }

        // URDF-based limits
        double upper = urdf_joint->limits->upper;
        double lower = urdf_joint->limits->lower;
        double velocity = urdf_joint->limits->velocity;
        double effort = urdf_joint->limits->effort;

        _joint_position_upper_urdf_limits.push_back(upper);
        _joint_position_lower_urdf_limits.push_back(lower);
        _joint_velocity_urdf_limits.push_back(velocity);
        _joint_effort_urdf_limits.push_back(effort);

        // aggregator-based starts as same as URDF defaults
        _joint_position_upper_limits.push_back(upper);
        _joint_position_lower_limits.push_back(lower);
        _joint_velocity_upper_limits.push_back(velocity);
        _joint_velocity_lower_limits.push_back(-velocity);
        _joint_effort_limits.push_back(effort);
    }

    // Populate the outgoing JointLimitsStamped message once
    _msg_joint_limits.header.frame_id = _control_frame;
    _msg_joint_limits.joint_names = _joint_names;
    _msg_joint_limits.position_upper_limits.data = _joint_position_upper_limits;
    _msg_joint_limits.position_lower_limits.data = _joint_position_lower_limits;
    _msg_joint_limits.velocity_upper_limits.data = _joint_velocity_upper_limits;
    _msg_joint_limits.velocity_lower_limits.data = _joint_velocity_lower_limits;
    _msg_joint_limits.effort_limits.data = _joint_effort_limits;

    // Initialize timestamps vector to 0 for each joint
    int num_joints = static_cast<int>(_joint_names.size());
    _joint_position_upper_timestamps.resize(num_joints, rclcpp::Time((int64_t)0, RCL_ROS_TIME));
    _joint_position_lower_timestamps.resize(num_joints, rclcpp::Time((int64_t)0, RCL_ROS_TIME));
    _joint_velocity_upper_timestamps.resize(num_joints, rclcpp::Time((int64_t)0, RCL_ROS_TIME));
    _joint_velocity_lower_timestamps.resize(num_joints, rclcpp::Time((int64_t)0, RCL_ROS_TIME));
    _joint_effort_timestamps.resize(num_joints, rclcpp::Time((int64_t)0, RCL_ROS_TIME));

    RCLCPP_INFO(this->get_logger(), 
        "JointLimitAggregator constructed with %d movable joints. Publishing at %.2f Hz.",
        num_joints, _rate);
}

/**
 * @brief Merges incoming aggregator-based limits if they are more restrictive 
 *        than what we already have. Updates timestamps for each limit that changed.
 */
void JointLimitAggregator::jointLimitsCallback(
    const trackit_msgs::msg::JointLimitsStamped::SharedPtr msg)
{
    auto msg_size = static_cast<int>(msg->joint_names.size());
    auto aggregator_size = static_cast<int>(_joint_names.size());

    // Must match aggregator's joint count to merge properly
    if (msg_size != aggregator_size) {
        RCLCPP_WARN(this->get_logger(),
            "Received JointLimitsStamped with %d joints, but aggregator expects %d. Skipping.",
            msg_size, aggregator_size);
        return;
    }

    // Merge each limit if it's more restrictive
    // position_upper
    if (!msg->position_upper_limits.data.empty() && 
        static_cast<int>(msg->position_upper_limits.data.size()) == aggregator_size)
    {
        for (int i = 0; i < aggregator_size; i++) {
            double incoming = msg->position_upper_limits.data[i];
            // If incoming is smaller, it's more restrictive
            if (_joint_position_upper_limits[i] > incoming) {
                _joint_position_upper_limits[i] = incoming;
                _joint_position_upper_timestamps[i] = rclcpp::Time(msg->header.stamp);
            }
        }
    }

    // position_lower
    if (!msg->position_lower_limits.data.empty() && 
        static_cast<int>(msg->position_lower_limits.data.size()) == aggregator_size)
    {
        for (int i = 0; i < aggregator_size; i++) {
            double incoming = msg->position_lower_limits.data[i];
            // If incoming is larger, it's more restrictive
            if (_joint_position_lower_limits[i] < incoming) {
                _joint_position_lower_limits[i] = incoming;
                _joint_position_lower_timestamps[i] = rclcpp::Time(msg->header.stamp);
            }
        }
    }

    // velocity_upper
    if (!msg->velocity_upper_limits.data.empty() &&
        static_cast<int>(msg->velocity_upper_limits.data.size()) == aggregator_size)
    {
        for (int i = 0; i < aggregator_size; i++) {
            double incoming = msg->velocity_upper_limits.data[i];
            // If incoming is smaller, it's more restrictive
            if (_joint_velocity_upper_limits[i] > incoming) {
                _joint_velocity_upper_limits[i] = incoming;
                _joint_velocity_upper_timestamps[i] = rclcpp::Time(msg->header.stamp);
            }
        }
    }

    // velocity_lower
    if (!msg->velocity_lower_limits.data.empty() &&
        static_cast<int>(msg->velocity_lower_limits.data.size()) == aggregator_size)
    {
        for (int i = 0; i < aggregator_size; i++) {
            double incoming = msg->velocity_lower_limits.data[i];
            // If incoming is larger (less negative), it's more restrictive
            if (_joint_velocity_lower_limits[i] < incoming) {
                _joint_velocity_lower_limits[i] = incoming;
                _joint_velocity_lower_timestamps[i] = rclcpp::Time(msg->header.stamp);
            }
        }
    }

    // effort
    if (!msg->effort_limits.data.empty() &&
        static_cast<int>(msg->effort_limits.data.size()) == aggregator_size)
    {
        for (int i = 0; i < aggregator_size; i++) {
            double incoming = msg->effort_limits.data[i];
            // If incoming is smaller, it's more restrictive
            if (_joint_effort_limits[i] > incoming) {
                _joint_effort_limits[i] = incoming;
                _joint_effort_timestamps[i] = rclcpp::Time(msg->header.stamp);
            }
        }
    }
}

/**
 * @brief Periodically publishes the merged aggregator-based + URDF-based limits.
 *        Falls back to URDF if aggregator-based values are stale.
 */
void JointLimitAggregator::timerCallback()
{
    rclcpp::Time current_time = this->now();

    // Prepare final published message
    _msg_joint_limits.header.stamp = current_time;

    // For each joint, if aggregator-based limit is still recent, use it if more restrictive;
    // otherwise revert to URDF.
    for (size_t i = 0; i < _joint_names.size(); i++) {
        // position upper
        if (isRecent(current_time, _joint_position_upper_timestamps[i])) {
            // Use aggregator-based if it's smaller than URDF
            double aggregator_val = _joint_position_upper_limits[i];
            double urdf_val       = _joint_position_upper_urdf_limits[i];
            _msg_joint_limits.position_upper_limits.data[i] =
                (urdf_val > aggregator_val) ? aggregator_val : urdf_val;
        } else {
            // aggregator-based stale => revert
            _msg_joint_limits.position_upper_limits.data[i] = _joint_position_upper_urdf_limits[i];
            _joint_position_upper_limits[i] = _joint_position_upper_urdf_limits[i];
        }

        // position lower
        if (isRecent(current_time, _joint_position_lower_timestamps[i])) {
            double aggregator_val = _joint_position_lower_limits[i];
            double urdf_val       = _joint_position_lower_urdf_limits[i];
            _msg_joint_limits.position_lower_limits.data[i] =
                (urdf_val < aggregator_val) ? aggregator_val : urdf_val;
        } else {
            _msg_joint_limits.position_lower_limits.data[i] = _joint_position_lower_urdf_limits[i];
            _joint_position_lower_limits[i] = _joint_position_lower_urdf_limits[i];
        }

        // velocity upper
        if (isRecent(current_time, _joint_velocity_upper_timestamps[i])) {
            double aggregator_val = _joint_velocity_upper_limits[i];
            double urdf_val       = _joint_velocity_urdf_limits[i];
            _msg_joint_limits.velocity_upper_limits.data[i] =
                (urdf_val > aggregator_val) ? aggregator_val : urdf_val;
        } else {
            _msg_joint_limits.velocity_upper_limits.data[i] = _joint_velocity_urdf_limits[i];
            _joint_velocity_upper_limits[i] = _joint_velocity_urdf_limits[i];
        }

        // velocity lower
        if (isRecent(current_time, _joint_velocity_lower_timestamps[i])) {
            double aggregator_val = _joint_velocity_lower_limits[i];
            double urdf_val       = -_joint_velocity_urdf_limits[i];
            _msg_joint_limits.velocity_lower_limits.data[i] =
                (urdf_val < aggregator_val) ? aggregator_val : urdf_val;
        } else {
            _msg_joint_limits.velocity_lower_limits.data[i] = -_joint_velocity_urdf_limits[i];
            _joint_velocity_lower_limits[i] = -_joint_velocity_urdf_limits[i];
        }

        // effort
        if (isRecent(current_time, _joint_effort_timestamps[i])) {
            double aggregator_val = _joint_effort_limits[i];
            double urdf_val       = _joint_effort_urdf_limits[i];
            _msg_joint_limits.effort_limits.data[i] =
                (urdf_val < aggregator_val) ? aggregator_val : urdf_val;
        } else {
            _msg_joint_limits.effort_limits.data[i] = _joint_effort_urdf_limits[i];
            _joint_effort_limits[i] = _joint_effort_urdf_limits[i];
        }
    }

    // Publish final message
    _pub_joint_limits->publish(_msg_joint_limits);
}

/**
 * @brief Determines if the aggregator-based limit is still \"recent\".
 * @param current_time The current ROS2 system time.
 * @param msg_time The aggregator-based limit's last update time.
 * @return True if (current_time - msg_time) < _msg_timeout, false otherwise.
 */
bool JointLimitAggregator::isRecent(const rclcpp::Time &current_time, const rclcpp::Time &msg_time)
{
    double dt = (current_time - msg_time).seconds();
    if (std::fabs(dt) > _msg_timeout) {
        return false;
    }
    return true;
}


