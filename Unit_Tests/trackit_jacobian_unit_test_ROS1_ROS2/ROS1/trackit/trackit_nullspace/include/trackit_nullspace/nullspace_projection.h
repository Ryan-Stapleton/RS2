#ifndef TRACKIT_NULLSPACE_NULLSPACE_PROJECTION_H
#define TRACKIT_NULLSPACE_NULLSPACE_PROJECTION_H

#include <ros/ros.h>
#include <eigen3/Eigen/Dense>

#include "trackit_msgs/JacobianStamped.h"
#include "trackit_msgs/JointVelocityStamped.h"

class NullSpace {

public:
	NullSpace(ros::NodeHandle* n);

private:
	void jointVelocityCallback(const trackit_msgs::JointVelocityStamped::ConstPtr& msg);
	void jacobianCallback(const trackit_msgs::JacobianStamped::ConstPtr& msg);

	bool isMsgCurrent(ros::Time time);

	Eigen::MatrixXd jointVelocityMsgToEigen(const trackit_msgs::JointVelocityStamped msg);
	std_msgs::Float64MultiArray eigenToJointVelocityMsg(const Eigen::MatrixXd& eig_joint_velocity);
	Eigen::MatrixXd pseudoInverse(const Eigen::MatrixXd&, const double lambda);

	ros::NodeHandle _n;
	ros::NodeHandle _nh;

	ros::Subscriber _sub_joint_vel;
	ros::Subscriber _sub_jacobian;

	ros::Publisher _pub_projected_joint_vel;

	trackit_msgs::JointVelocityStamped _msg_joint_velocity;

	Eigen::MatrixXd _eig_jacobian;

	bool _first_jacobian_msg;

	double _time_diff_threshold;
	double _lambda;

	std::vector<std::string> _joint_names;

	int _joint_num;
};

#endif