#ifndef TRACKIT_NULLSPACE_TASK_JOINT_MIDDLE_H
#define TRACKIT_NULLSPACE_TASK_JOINT_MIDDLE_H

#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <eigen3/Eigen/Dense>

#include "trackit_msgs/JointLimitsStamped.h"
#include "trackit_msgs/JointVelocityStamped.h"


class JointMiddle {

public:
	JointMiddle(ros::NodeHandle* n);

private:
	void jointLimitsCallback(const trackit_msgs::JointLimitsStamped::ConstPtr& msg);
	void jointStateCallback(const sensor_msgs::JointState::ConstPtr& msg);

	void msgToMatrix(const std_msgs::Float64MultiArray& msg, Eigen::MatrixXd& mat);

	ros::NodeHandle _n;
	ros::NodeHandle _nh;

	ros::Publisher _pub_joint_velocity;
	ros::Subscriber _sub_joint_limits;
	ros::Subscriber _sub_joint_states;

	std::vector<std::string> 	_joint_names;

	Eigen::MatrixXd _joint_upper_limits;
	Eigen::MatrixXd _joint_lower_limits;
	Eigen::MatrixXd _midpoint;

	double _max_velocity;
};
#endif // TRACKIT_NULLSPACE_TASK_JOINT_MIDDLE_H