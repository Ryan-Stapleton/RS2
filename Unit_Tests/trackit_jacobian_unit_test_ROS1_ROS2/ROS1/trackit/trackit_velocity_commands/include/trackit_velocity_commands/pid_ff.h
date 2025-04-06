#ifndef TRACKIT_VELOCITY_COMMANDS_PID_FF_H
#define TRACKIT_VELOCITY_COMMANDS_PID_FF_H

#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/TwistStamped.h>

#include <eigen3/Eigen/Dense>

class PIDFF {

public:
	PIDFF(ros::NodeHandle* n);
	
private:
	void twistFFCallback(const geometry_msgs::TwistStamped::ConstPtr& msg);
	void timerCallback(const ros::TimerEvent&);

	Eigen::Vector3d calculateQuatError(const Eigen::Quaterniond& quat_ee, const Eigen::Quaterniond& quat_target);

	ros::NodeHandle _n;
	ros::NodeHandle _nh;
	
	ros::Subscriber _sub_twist_ff;
	ros::Publisher _pub_twist;
	ros::Publisher _pub_twist_error;
	ros::Timer _timer;

	tf2_ros::Buffer _tf_buffer;
	tf2_ros::TransformListener _tf_listen;

	double _rate;
	
	double _omg_i_lin_max;
	double _omg_i_ang_max;
	double _gamma_ratio;
	
	std::string _control_frame;
	std::string _ee_frame;
	std::string _target_frame;

	Eigen::Matrix<double, 6, 1> _error_prev;
	Eigen::Matrix<double, 6, 1> _omg_i_prev;
	Eigen::Matrix<double, 6, 1> _omg_d_prev;
	Eigen::Matrix<double, 6, 1> _gamma;
	Eigen::Matrix<double, 6, 1> _Kd;

	Eigen::DiagonalMatrix<double, 6> _Kp;
	Eigen::DiagonalMatrix<double, 6> _Ki;

	geometry_msgs::TwistStamped _msg_twist_ff;
};

#endif //TRACKIT_VELOCITY_COMMANDS_PID_FF_H
