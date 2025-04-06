#ifndef TRACKIT_ROBOT_STATE_STATIC_JOINT_LIMIT_PUBLISHER_H
#define TRACKIT_ROBOT_STATE_STATIC_JOINT_LIMIT_PUBLISHER_H

#include <ros/ros.h>
#include <kdl_parser/kdl_parser.hpp>
#include <urdf/model.h>

#include "trackit_msgs/JointLimitsStamped.h"

class StaticJointLimitPublisher {

public:
	StaticJointLimitPublisher(ros::NodeHandle* n);

private:
	void timerCallback(const ros::TimerEvent& event);

	ros::NodeHandle _n;
	ros::NodeHandle _nh;

	std::string _robot_description;
	std::string _control_frame;
	std::string _tool_frame;
	std::vector<std::string> _joint_names;

	std::vector<double> _joint_position_upper_limits;
	std::vector<double> _joint_position_lower_limits;
	std::vector<double> _joint_velocity_upper_limits;
	std::vector<double> _joint_velocity_lower_limits;
	std::vector<double> _joint_effort_limits;

	KDL::Tree _kdl_tree;
	KDL::Chain _kdl_chain;

	urdf::Model _robot_model;	

	ros::Publisher _pub_joint_limits;
	ros::Timer _timer;

	trackit_msgs::JointLimitsStamped _msg_joint_limits;
	
	double _rate;
	
};
#endif // TRACKIT_ROBOT_STATE_STATIC_JOINT_LIMIT_PUBLISHER_H