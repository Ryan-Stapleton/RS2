#ifndef TRACKIT_ROBOT_STATE_JOINT_LIMIT_AGGREGATOR_H
#define TRACKIT_ROBOT_STATE_JOINT_LIMIT_AGGREGATOR_H

#include <ros/ros.h>
#include <kdl_parser/kdl_parser.hpp>
#include <urdf/model.h>

#include "trackit_msgs/JointLimitsStamped.h"

class JointLimitAggregator {

public:
	JointLimitAggregator(ros::NodeHandle* n);

private:
	void jointLimitsCallback(const trackit_msgs::JointLimitsStamped::ConstPtr& msg);
	void timerCallback(const ros::TimerEvent& event);

	bool isRecent(const ros::Time current_time, const ros::Time msg_time);

	ros::NodeHandle _n;
	ros::NodeHandle _nh;

	ros::Timer _timer;

	std::string _robot_description;
	std::string _control_frame;
	std::string _tool_frame;

	std::vector<std::string> _joint_names;

	std::vector<double> _joint_position_upper_urdf_limits;
	std::vector<double> _joint_position_lower_urdf_limits;
	std::vector<double> _joint_position_upper_limits;
	std::vector<double> _joint_position_lower_limits;
	std::vector<double> _joint_velocity_urdf_limits;
	std::vector<double> _joint_velocity_upper_limits;
	std::vector<double> _joint_velocity_lower_limits;
	std::vector<double> _joint_effort_urdf_limits;
	std::vector<double> _joint_effort_limits;

	std::vector<ros::Time> _joint_position_upper_timestamps;
	std::vector<ros::Time> _joint_position_lower_timestamps;
	std::vector<ros::Time> _joint_velocity_upper_timestamps;
	std::vector<ros::Time> _joint_velocity_lower_timestamps;
	std::vector<ros::Time> _joint_effort_timestamps;

	KDL::Tree _kdl_tree;
	KDL::Chain _kdl_chain;

	urdf::Model _robot_model;	

	ros::Publisher  _pub_joint_limits;
	ros::Subscriber _sub_joint_limits;

	trackit_msgs::JointLimitsStamped _msg_joint_limits;
	
	double _msg_timeout;
	double _rate;

};
#endif // TRACKIT_ROBOT_STATE_JOINT_LIMIT_AGGREGATOR_H