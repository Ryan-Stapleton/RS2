#ifndef TRACKIT_ROBOT_STATE_JOINT_LIMIT_AGGREGATOR_H
#define TRACKIT_ROBOT_STATE_JOINT_LIMIT_AGGREGATOR_H

#include "rclcpp/rclcpp.hpp"
#include <kdl_parser/kdl_parser.hpp>
#include <urdf/model.h>
#include "trackit_msgs/msg/joint_limits_stamped.hpp"

/**
 * @class JointLimitAggregator
 * @brief Aggregates and merges joint limits from incoming messages with URDF-based defaults.
 *
 * This node:
 *  1. Loads URDF joint limits for each non-fixed joint.
 *  2. Listens for incoming JointLimitsStamped messages.
 *  3. Merges the aggregator-based, more restrictive limits if they are more recent
 *     than the URDF default or older aggregator data.
 *  4. Periodically publishes the resulting final joint limits.
 */
class JointLimitAggregator : public rclcpp::Node {
public:
    /**
     * @brief Constructor for JointLimitAggregator node.
     * Initializes parameters, subscribers, and publishers.
     */
    JointLimitAggregator();

private:
    /**
     * @brief ROS2 subscription callback for new JointLimitsStamped data.
     * Merges incoming data with aggregator's more restrictive limits, if applicable.
     * @param msg The new JointLimitsStamped message.
     */
    void jointLimitsCallback(const trackit_msgs::msg::JointLimitsStamped::SharedPtr msg);

    /**
     * @brief Timer callback for periodically publishing merged joint limits.
     * Checks if aggregator-based limits are still \"recent\". If not, reverts to URDF defaults.
     */
    void timerCallback();

    /**
     * @brief Determines if the aggregator's stored limit is still considered recent.
     * @param current_time Current ROS2 time.
     * @param msg_time The timestamp when the aggregator-based limit was last updated.
     * @return True if the difference in time is less than _msg_timeout, otherwise false.
     */
    bool isRecent(const rclcpp::Time &current_time, const rclcpp::Time &msg_time);

    // Node parameters
    std::string _robot_description;
    std::string _control_frame;
    std::string _tool_frame;
    double _msg_timeout;
    double _rate;

    // Joints we are tracking
    std::vector<std::string> _joint_names;

    // URDF-based default limits
    std::vector<double> _joint_position_upper_urdf_limits;
    std::vector<double> _joint_position_lower_urdf_limits;
    std::vector<double> _joint_velocity_urdf_limits;
    std::vector<double> _joint_effort_urdf_limits;

    // Aggregator-based current limits (potentially more restrictive)
    std::vector<double> _joint_position_upper_limits;
    std::vector<double> _joint_position_lower_limits;
    std::vector<double> _joint_velocity_upper_limits;
    std::vector<double> _joint_velocity_lower_limits;
    std::vector<double> _joint_effort_limits;

    // Timestamps for aggregator-based limits
    std::vector<rclcpp::Time> _joint_position_upper_timestamps;
    std::vector<rclcpp::Time> _joint_position_lower_timestamps;
    std::vector<rclcpp::Time> _joint_velocity_upper_timestamps;
    std::vector<rclcpp::Time> _joint_velocity_lower_timestamps;
    std::vector<rclcpp::Time> _joint_effort_timestamps;

    // KDL tree/chain and URDF model
    KDL::Tree _kdl_tree;
    KDL::Chain _kdl_chain;
    urdf::Model _robot_model;

    // ROS2 interfaces
    rclcpp::Publisher<trackit_msgs::msg::JointLimitsStamped>::SharedPtr _pub_joint_limits;
    rclcpp::Subscription<trackit_msgs::msg::JointLimitsStamped>::SharedPtr _sub_joint_limits;
    rclcpp::TimerBase::SharedPtr _timer;

    // The final merged JointLimitsStamped message to publish
    trackit_msgs::msg::JointLimitsStamped _msg_joint_limits;
};

#endif // TRACKIT_ROBOT_STATE_JOINT_LIMIT_AGGREGATOR_H
