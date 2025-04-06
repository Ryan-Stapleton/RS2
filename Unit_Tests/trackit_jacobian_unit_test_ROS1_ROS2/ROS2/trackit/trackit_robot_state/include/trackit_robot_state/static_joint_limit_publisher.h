#ifndef TRACKIT_ROBOT_STATE_STATIC_JOINT_LIMIT_PUBLISHER_H
#define TRACKIT_ROBOT_STATE_STATIC_JOINT_LIMIT_PUBLISHER_H

#include "rclcpp/rclcpp.hpp"
#include <kdl_parser/kdl_parser.hpp>
#include <urdf/model.h>
#include "trackit_msgs/msg/joint_limits_stamped.hpp"

class StaticJointLimitPublisher : public rclcpp::Node {
public:
    /**
     * @brief Constructor for StaticJointLimitPublisher node.
     * Initializes parameters, publishers, and timers.
     */
    StaticJointLimitPublisher();

private:
    /**
     * @brief Timer callback function to periodically publish static joint limits.
     */
    void timerCallback();

    // Node parameters
    std::string _robot_description;
    std::string _control_frame;
    std::string _tool_frame;
    double _rate;

    // KDL & URDF models
    KDL::Tree _kdl_tree;
    KDL::Chain _kdl_chain;
    urdf::Model _robot_model;

    // Joint data
    std::vector<std::string> _joint_names;
    std::vector<double> _joint_position_upper_limits;
    std::vector<double> _joint_position_lower_limits;
    std::vector<double> _joint_velocity_upper_limits;
    std::vector<double> _joint_velocity_lower_limits;
    std::vector<double> _joint_effort_limits;

    // ROS2 Interfaces
    rclcpp::Publisher<trackit_msgs::msg::JointLimitsStamped>::SharedPtr _pub_joint_limits;
    rclcpp::TimerBase::SharedPtr _timer;

    // Message storage
    trackit_msgs::msg::JointLimitsStamped _msg_joint_limits;
};

#endif // TRACKIT_ROBOT_STATE_STATIC_JOINT_LIMIT_PUBLISHER_H
