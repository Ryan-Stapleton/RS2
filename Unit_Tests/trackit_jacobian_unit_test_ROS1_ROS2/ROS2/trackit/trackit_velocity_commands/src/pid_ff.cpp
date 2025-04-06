/**
 * @file pid_ff.cpp
 * @brief A node for a PID controller with feedforward twist.
 * 
 * @param rate the rate at which the twist is published
 * @param Kp_lin linear proportional gain
 * @param Kp_ang angular proportional gain
 * @param Ki_lin linear integral gain
 * @param Ki_ang angular integral gain
 * @param Kd_lin linear derivative gain
 * @param Kd_ang angular derivative gain
 * @param omega_i_lin_max linear integral limit
 * @param omega_i_ang_max angular integral limit
 * @param control_frame control frame of the robot, also the frame of the twist
 * @param ee_frame frame of the end-effector
 * @param target_frame frame of the target for the end effector
 * 
 * Subscribers:	
 * 	- twist_in: the feedforward twist message
 * 
 * Publishers:
 * 	  twist_pid: the output twist
 * 
 */

#include "trackit_velocity_commands/pid_ff.h"

/**
 * Constructor for the PIDFF class.
 *
 * @param n Pointer to the ROS NodeHandle object
 *
 * @throws None
 */
PIDFF::PIDFF(ros::NodeHandle* n):_n(*n), _tf_listen(_tf_buffer) {
	// private nodehandle
	ros::NodeHandle _nh("~");

	// node parameters
	double Kp_lin, Kp_ang, Ki_lin, Ki_ang, Kd_lin, Kd_ang;
	_nh.param<double>("rate"			, _rate				, 500.0);
	_nh.param<double>("Kp_linear" 		, Kp_lin			, 0.0);
	_nh.param<double>("Kp_angular"		, Kp_ang			, 0.0);
	_nh.param<double>("Ki_linear" 		, Ki_lin			, 0.0);
	_nh.param<double>("Ki_angular"		, Ki_ang			, 0.0);
	_nh.param<double>("Kd_linear" 		, Kd_lin			, 0.0);
	_nh.param<double>("Kd_angular"		, Kd_ang			, 0.0);
	_nh.param<double>("gamma_ratio"		, _gamma_ratio		, 0.1);
	_nh.param<double>("omega_i_lin_max"	, _omg_i_lin_max	, 0.1);
	_nh.param<double>("omega_i_ang_max"	, _omg_i_ang_max	, 0.1);

	_nh.param<std::string>("control_frame"	, _control_frame	, std::string());
	_nh.param<std::string>("ee_frame"	 	, _ee_frame			, "ee_frame");
	_nh.param<std::string>("target_frame"	, _target_frame		, "target_frame");

	// subscriber
	_sub_twist_ff = _n.subscribe("twist_in", 1, &PIDFF::twistFFCallback, this);

	// publisher
	_pub_twist = _n.advertise<geometry_msgs::TwistStamped>("twist_pid", 1);
	
	// timer
	_timer = _n.createTimer(ros::Duration(1.0/_rate), &PIDFF::timerCallback, this);

	// initialize variables
	_error_prev.setZero();
	_omg_i_prev.setZero();
	_omg_d_prev.setZero();

	_Kp.diagonal() << Kp_lin, Kp_lin, Kp_lin, Kp_ang, Kp_ang, Kp_ang;
	_Ki.diagonal() << Ki_lin, Ki_lin, Ki_lin, Ki_ang, Ki_ang, Ki_ang;
	_Kd << Kd_lin, Kd_lin, Kd_lin, Kd_ang, Kd_ang, Kd_ang;

	Eigen::Matrix<double,6,1> kp_temp;
	kp_temp << Kp_lin, Kp_lin, Kp_lin, Kp_ang, Kp_ang, Kp_ang;

	for (int i = 0; i < 6; i++)	{
		ROS_INFO_STREAM("i: " << i << " Kp: " << kp_temp(i,0) << " Kd: " << _Kd(i,0));
		_gamma(i,0) = _gamma_ratio * _Kd(i,0)/kp_temp(i,0); 
	}

	_msg_twist_ff.header.stamp = ros::Time(0.0);

}

/**
 * Sets the value of the member variable _msg_twist_ff to the value of the 
 * input message.
 *
 * @param msg A constant pointer to a TwistStamped message
 *
 * @throws None
 */
void PIDFF::twistFFCallback(const geometry_msgs::TwistStamped::ConstPtr& msg) {
	_msg_twist_ff = *msg;
}

/**
 * Executes the timer callback for the PIDFF class.
 * This calculates the error between the current pose and the target pose,
 * which is then used to calculate the output twist.
 * Feed forward velocity is also added here.
 * 
 * @param const ros::TimerEvent& The timer event.
 *
 * @return void
 *
 * @throws None
 */
void PIDFF::timerCallback(const ros::TimerEvent&) {
	// Feed forward velocity component
	Eigen::Matrix<double, 6, 1> omega_ff;
	omega_ff.setZero();
	
	// If the ff twist is recent, set omega_ff to the ff twist
	if(!_msg_twist_ff.header.stamp.toSec() == 0.0) {
		omega_ff(0,0) = _msg_twist_ff.twist.linear.x;
		omega_ff(1,0) = _msg_twist_ff.twist.linear.y;
		omega_ff(2,0) = _msg_twist_ff.twist.linear.z;
		omega_ff(3,0) = _msg_twist_ff.twist.angular.x;
		omega_ff(4,0) = _msg_twist_ff.twist.angular.y;
		omega_ff(5,0) = _msg_twist_ff.twist.angular.z;
	}

	// Compare poses in the control frame
	Eigen::Matrix<double, 6, 1> error;

	geometry_msgs::TransformStamped tf_ee;
	geometry_msgs::TransformStamped tf_target;
	try {
		tf_ee 		= _tf_buffer.lookupTransform(_control_frame, _ee_frame, ros::Time(0), ros::Duration(1.0));
		tf_target 	= _tf_buffer.lookupTransform(_control_frame, _target_frame, ros::Time(0), ros::Duration(1.0));
	}
	catch (tf2::TransformException& ex) {
		ROS_ERROR("%s", ex.what());
		return;
	}

	// calculate the error
	Eigen::Vector3d ee_pos(tf_ee.transform.translation.x, tf_ee.transform.translation.y, tf_ee.transform.translation.z);
	Eigen::Vector3d target_pos(tf_target.transform.translation.x, tf_target.transform.translation.y, tf_target.transform.translation.z);
	Eigen::Vector3d error_pos = target_pos - ee_pos;

	Eigen::Quaterniond ee_quat(tf_ee.transform.rotation.w, tf_ee.transform.rotation.x, tf_ee.transform.rotation.y, tf_ee.transform.rotation.z);
	Eigen::Quaterniond target_quat(tf_target.transform.rotation.w, tf_target.transform.rotation.x, tf_target.transform.rotation.y, tf_target.transform.rotation.z);
	Eigen::Vector3d error_quat = calculateQuatError(ee_quat, target_quat);

	error.head(3) = error_pos;
	error.tail(3) = error_quat;

	// Calculate the Proportional component
	Eigen::Matrix<double, 6, 1> omega_p;
	omega_p = _Kp * error;

	// Calculate the Integral component
	Eigen::Matrix<double, 6, 1> omega_i;
	omega_i = _omg_i_prev + _Ki * (error + _error_prev) / (2 * _rate);

	// Clamp the integral component
	if(omega_i.head(3).norm() > _omg_i_lin_max) {
		omega_i.head(3) = _omg_i_lin_max / omega_i.head(3).norm() * omega_i.head(3);
	}
	if(omega_i.tail(3).norm() > _omg_i_ang_max) {
		omega_i.tail(3) = _omg_i_ang_max / omega_i.tail(3).norm() * omega_i.tail(3);
	}

	// Calculate the Derivative component
	Eigen::Matrix<double, 6, 1> omega_d;
	for (int i = 0; i < 6; i++) {
		omega_d(i,1) = ((2 * _gamma(i,1) - 1 / _rate) * _omg_d_prev(i,1) + 2 * _Kd(i,1) * (error(i,1) - _error_prev(i,1))) / (2 * _gamma(i,1) + 1 / _rate);
	}

	// Compute the total velocity
	Eigen::Matrix<double, 6, 1> omega;
	omega = omega_ff + omega_p + omega_i + omega_d;

	// set the current values to the previous values for the next iteration
	_error_prev = error;
	_omg_i_prev = omega_i;
	_omg_d_prev = omega_d;

	// Publish the twist
	geometry_msgs::TwistStamped msg_twist;
	msg_twist.header.stamp = ros::Time::now();
	msg_twist.header.frame_id = _control_frame;
	msg_twist.twist.linear.x = omega(0,0);
	msg_twist.twist.linear.y = omega(1,0);
	msg_twist.twist.linear.z = omega(2,0);
	msg_twist.twist.angular.x = omega(3,0);
	msg_twist.twist.angular.y = omega(4,0);
	msg_twist.twist.angular.z = omega(5,0);

	_pub_twist.publish(msg_twist);
}

/**
 * Calculates the quaternion error between the given end effector quaternion and the target quaternion.
 *
 * @param quat_ee the end effector quaternion
 * @param quat_target the target quaternion
 *
 * @return the quaternion error as a 3D axis angle vector
 *
 * @throws None
 */
Eigen::Vector3d PIDFF::calculateQuatError(const Eigen::Quaterniond& quat_ee, const Eigen::Quaterniond& quat_target) {
	// calculate quaternion error
	Eigen::Quaterniond quat_error;
	quat_error = quat_target * quat_ee.conjugate();
	
	// calculate the axis and angle
	double angle = 2*acos(quat_error.w());
	double norm = quat_error.vec().norm();

	// if the angle is small, set it to zero and axis as (1, 0, 0)
	Eigen::Vector3d axis;
	if(norm > 1e-4) {
		axis = quat_error.vec()/norm;
	}
	else {
		axis << 1, 0, 0;
		angle = 0;
	}
	
	// Limiting the angle to [-pi, pi]
	if(angle > M_PI) {
		angle = 2 * M_PI - angle;
		axis = -axis;
	}

	// Sanity check
	if((angle < -M_PI) || (angle > M_PI)) {
		ROS_ERROR_STREAM("Axis angle error outside of bounds. Angle error: " << angle << " rad.");
		angle = 0;
	}

	return angle*axis;
}

int main(int argc, char** argv) {
	ros::init(argc, argv, "trackit_pid_ff");
	ros::NodeHandle n;
	PIDFF pid_ff(&n);
	ros::spin();
	return 0;
}