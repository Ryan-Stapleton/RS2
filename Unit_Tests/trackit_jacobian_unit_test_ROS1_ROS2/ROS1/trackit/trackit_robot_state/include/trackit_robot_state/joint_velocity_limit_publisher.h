#ifndef TRACKIT_ROBOT_STATE_JOINT_VELOCITY_LIMIT_PUBLISHER_H
#define TRACKIT_ROBOT_STATE_JOINT_VELOCITY_LIMIT_PUBLISHER_H

#include <ros/ros.h>
#include <kdl_parser/kdl_parser.hpp>
#include <urdf/model.h>
#include <sensor_msgs/JointState.h>

#include "trackit_msgs/JointLimitsStamped.h"

class JointVelocityLimitPublisher {

public:
	JointVelocityLimitPublisher(ros::NodeHandle* n);

private:
	void jointStateCallback(const sensor_msgs::JointState::ConstPtr& msg);

	ros::NodeHandle _n;
	ros::NodeHandle _nh;

	std::string _robot_description;
	std::string _control_frame;
	std::string _tool_frame;

	std::vector<std::string> _joint_names;

	std::vector<double> _joint_position_upper_limits;
	std::vector<double> _joint_position_lower_limits;
	std::vector<double> _joint_velocity_limits_urdf;
	std::vector<double> _joint_velocity_upper_limits;
	std::vector<double> _joint_velocity_lower_limits;
	std::vector<double> _joint_effort_limits;

	KDL::Tree _kdl_tree;
	KDL::Chain _kdl_chain;

	urdf::Model _robot_model;	

	ros::Publisher _pub_joint_limits;
	ros::Subscriber _sub_joint_state;

	trackit_msgs::JointLimitsStamped _msg_joint_limits;
	
	double _time_to_pos_limit;
	double _position_limit_buffer;
	
};
#endif // TRACKIT_ROBOT_STATE_JOINT_VELOCITY_LIMIT_PUBLISHER_H