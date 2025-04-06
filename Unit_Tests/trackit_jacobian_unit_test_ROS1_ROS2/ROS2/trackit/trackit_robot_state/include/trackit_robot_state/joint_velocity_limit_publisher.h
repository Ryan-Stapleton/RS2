#ifndef TRACKIT_ROBOT_STATE_JOINT_VELOCITY_LIMIT_PUBLISHER_H
#define TRACKIT_ROBOT_STATE_JOINT_VELOCITY_LIMIT_PUBLISHER_H

#include "rclcpp/rclcpp.hpp"
#include <kdl_parser/kdl_parser.hpp>
#include <urdf/model.h>
#include <sensor_msgs/msg/joint_state.hpp>
#include "trackit_msgs/msg/joint_limits_stamped.hpp"

class JointVelocityLimitPublisher : public rclcpp::Node {
public:
    /**
     * @brief Constructor for JointVelocityLimitPublisher node.
     * Initializes parameters, subscribers, and publishers.
     */
    JointVelocityLimitPublisher();

private:
    /**
     * @brief Callback function to process incoming joint state messages.
     * @param msg Received JointState message.
     */
    void jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg);

    // Node parameters
    std::string _robot_description;
    std::string _control_frame;
    std::string _tool_frame;
    double _time_to_pos_limit;
    double _position_limit_buffer;

    // KDL & URDF models
    KDL::Tree _kdl_tree;
    KDL::Chain _kdl_chain;
    urdf::Model _robot_model;

    // Joint data
    std::vector<std::string> _joint_names;
    std::vector<double> _joint_position_upper_limits;
    std::vector<double> _joint_position_lower_limits;

    // ROS2 Interfaces
    rclcpp::Publisher<trackit_msgs::msg::JointLimitsStamped>::SharedPtr _pub_joint_limits;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr _sub_joint_state;

    // Message storage
    trackit_msgs::msg::JointLimitsStamped _msg_joint_limits;
};

#endif // TRACKIT_ROBOT_STATE_JOINT_VELOCITY_LIMIT_PUBLISHER_H
