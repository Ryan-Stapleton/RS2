#ifndef TRACKIT_CONTROL_INTERFACE_ROBOT_COMMAND_H
#define TRACKIT_CONTROL_INTERFACE_ROBOT_COMMAND_H

#include <ros/ros.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/Bool.h>

#include "trackit_msgs/JointVelocityStamped.h"

class RobotCommand {

public:
    RobotCommand(ros::NodeHandle* n);

private:
    void trackit_msg_callback(const trackit_msgs::JointVelocityStamped::ConstPtr& msg);
    void deadman_callback(const std_msgs::Bool::ConstPtr& msg);

    ros::NodeHandle _n;
	ros::NodeHandle _nh;

    ros::Subscriber _sub_trackit_msg;
    ros::Subscriber _sub_deadman_msg;
    ros::Publisher  _pub_joint_command;

    bool _use_deadman;
    bool _enable;

    std::vector<std::string> _joint_names;
};

#endif