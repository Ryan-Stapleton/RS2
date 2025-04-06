#ifndef TRACKIT_CORE_IDK_H
#define TRACKIT_CORE_IDK_H

#include <ros/ros.h>
#include <geometry_msgs/TwistStamped.h>
#include <sensor_msgs/JointState.h>

#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/SVD>

#include "trackit_msgs/JacobianStamped.h"
#include "trackit_msgs/JointVelocityStamped.h"
#include "trackit_msgs/JointLimitsStamped.h"

class IDK {

public:
	IDK(ros::NodeHandle* n);

private:
	void twistInCallback(const geometry_msgs::TwistStamped::ConstPtr& msg);
	void twistExternalCallback(const geometry_msgs::TwistStamped::ConstPtr& msg);
	
	void jointVelocityExternalCallback(const trackit_msgs::JointVelocityStamped::ConstPtr& msg);
	void jointVelocityNullCallback(const trackit_msgs::JointVelocityStamped::ConstPtr& msg);
	void jointLimitsCallback(const trackit_msgs::JointLimitsStamped::ConstPtr& msg);
	
	void jacobianCallback(const trackit_msgs::JacobianStamped::ConstPtr& msg);
	
	void timerCallback(const ros::TimerEvent&);

	void checkWithinLimit(Eigen::MatrixXd& joint_velocity);

	bool isMsgCurrent(ros::Time msg_time);

	Eigen::Matrix<double,6,1> twistMsgToEigen(const geometry_msgs::TwistStamped msg_twist);
	Eigen::MatrixXd jointVelocityMsgToEigen(const trackit_msgs::JointVelocityStamped msg);

	ros::NodeHandle _n;
	ros::NodeHandle _nh;

	ros::Subscriber _sub_twist_in;
	ros::Subscriber _sub_twist_external;
	ros::Subscriber _sub_joint_external;
	ros::Subscriber _sub_joint_null;
	ros::Subscriber _sub_jacobian;
	ros::Subscriber _sub_joint_limits;

	ros::Publisher _pub_joint_velocity;

	ros::Timer	_timer;

	geometry_msgs::TwistStamped			_msg_twist_in;
	geometry_msgs::TwistStamped			_msg_twist_external;
	trackit_msgs::JointVelocityStamped	_msg_joint_external;
	trackit_msgs::JointVelocityStamped	_msg_joint_null;
	trackit_msgs::JacobianStamped		_msg_jacobian_inverse;

	Eigen::MatrixXd _eig_jacobian_inv;
	Eigen::MatrixXd _eig_joint_state;
	Eigen::MatrixXd _eig_joint_vel_limits;

	double _control_rate;
	double _max_lin_vel;
	double _max_ang_vel;
	double _time_diff_threshold;
	
	std::string _control_frame;
	
	std::vector<std::string> _joint_names;

	bool _first_jacobian_msg;

	int	_joint_num;

	std::vector<int> _mask;

};

#endif // TRACKIT_CORE_IDK_H