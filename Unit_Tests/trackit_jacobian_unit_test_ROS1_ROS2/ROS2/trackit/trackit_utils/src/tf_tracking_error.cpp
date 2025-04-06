/**
 * @file tf_tracking_error.cpp
 * @brief a node to calculate the twist necessary to nmove one frame to another
 * 
 * This node calculates the error between two frames and convert it to a twist.
 * Components of the twist can be enabled/disabled.
 * 
 * @param rate the rate at which the node runs
 * @param max_lin_velocity the maximum linear velocity of the published twist.
 * @param max_ang_velocity the maximum angular velocity of the published twist.
 * @param header_frame the frame of the twist messages and tf frames.
 * @param tracked_frame the frame being tracked.
 * @param target_frame the target to move the tracked frame to.
 * @param enable_pos_x enable the translational x component of the twist.
 * @param enable_pos_y enable the translational y component of the twist.
 * @param enable_pos_z enable the translational z component of the twist.
 * @param enable_rot_x enable the rotational x component of the twist.
 * @param enable_rot_y enable the rotational y component of the twist.
 * @param enable_rot_z enable the rotational z component of the twist.
 * 
 * Publishers:
 *     twist_tracking_error: the twist to move from the tracked frame to the target frame.
 * 
 */

#include "trackit_utils/tf_tracking_error.h"

/**
 * Constructor for the TFTrackingError class.
 *
 * @param n pointer to the ros::NodeHandle object
 *
 * @return void
 *
 * @throws None
 */
TFTrackingError::TFTrackingError(ros::NodeHandle* n):_n(*n), _tf_listen(_tf_buffer) {
	// private nodehandle
	_nh = ros::NodeHandle("~");
	
	// node parameters
	_nh.param<double>("rate", _rate, 500);
	_nh.param<double>("max_lin_velocity", _max_lin_velocity, 0.2);
	_nh.param<double>("max_ang_velocity", _max_ang_velocity, 1.0);

	_nh.param<std::string>("header_frame"	, _header_frame	, "ee_link");
	_nh.param<std::string>("tracked_frame"	, _tracked_frame, "ee_link");
	_nh.param<std::string>("target_frame"	, _target_frame	, "ee_target");

	_nh.param<bool>("enable_pos_x", _enable_pos_x, true);
	_nh.param<bool>("enable_pos_y", _enable_pos_y, true);
	_nh.param<bool>("enable_pos_z", _enable_pos_z, true);
	
	_nh.param<bool>("enable_rot_x", _enable_rot_x, true);
	_nh.param<bool>("enable_rot_y", _enable_rot_y, true);
	_nh.param<bool>("enable_rot_z", _enable_rot_z, true);

	// timer
	_timer = _n.createTimer(ros::Duration(1.0/_rate), &TFTrackingError::timerCallback, this);
	
	// publisher
	_pub_twist = _n.advertise<geometry_msgs::TwistStamped>("twist_tracking_error", 1);

}

/**
 * Callback function for the timer.
 * The error between the tracked frame and the target frame is calculated
 * and the twist is published in this callback.
 * 
 * @param const ros::TimerEvent& event: The timer event.
 *
 * @return void
 *
 * @throws None
 */
void TFTrackingError::timerCallback(const ros::TimerEvent&) {
	geometry_msgs::TwistStamped twist_error;
	
	geometry_msgs::TransformStamped tf_tracked;
	geometry_msgs::TransformStamped tf_target;

	// get the transform between the tracked frame and the target frame
	try {
		tf_tracked = _tf_buffer.lookupTransform(_header_frame, _tracked_frame, ros::Time(0), ros::Duration(1.0));
		tf_target = _tf_buffer.lookupTransform(_header_frame, _target_frame, ros::Time(0), ros::Duration(1.0));
	}
	catch (tf2::TransformException& ex) {
		ROS_ERROR("%s", ex.what());
		return;
	}

	// calculate the error
	Eigen::Vector3d tracked_vec = extractPosition(tf_tracked);
	Eigen::Vector3d target_vec = extractPosition(tf_target);
	Eigen::Vector3d error_vec = target_vec - tracked_vec;
	
	Eigen::Quaterniond tracked_quat = extractQuaternion(tf_tracked);
	Eigen::Quaterniond target_quat = extractQuaternion(tf_target);
	Eigen::Vector3d error_axis_angle = calculateQuatError(tracked_quat, target_quat);

	// limit the twist
	limitToMax(error_vec, _max_lin_velocity);
	limitToMax(error_axis_angle, _max_ang_velocity);

	geometry_msgs::TwistStamped twist;
	twist.header.frame_id = _header_frame;
	twist.header.stamp = ros::Time::now();

	// set disabled components to zero
	if(_enable_pos_x) {
		twist.twist.linear.x = error_vec[0];
	}
	else {
		twist.twist.linear.x = 0.0;
	}

	if(_enable_pos_y) {
		twist.twist.linear.y = error_vec[1];
	}
	else {
		twist.twist.linear.y = 0.0;
	}

	if(_enable_pos_z) {
		twist.twist.linear.z = error_vec[2];
	}
	else {
		twist.twist.linear.z = 0.0;
	}

	if(_enable_rot_x) {
		twist.twist.angular.x = error_axis_angle[0];
	}
	else {
		twist.twist.angular.x = 0.0;
	}

	if(_enable_rot_y) {
		twist.twist.angular.y = error_axis_angle[1];
	}
	else {
		twist.twist.angular.y = 0.0;
	}

	if(_enable_rot_z) {
		twist.twist.angular.z = error_axis_angle[2];
	}
	else {
		twist.twist.angular.z = 0.0;
	}

	// publish the twist
	_pub_twist.publish(twist);
}

/**
 * Extracts the position from a given TransformStamped message.
 *
 * @param tf_in the input TransformStamped message
 *
 * @return the position as a Vector3d object
 *
 * @throws None
 */
Eigen::Vector3d TFTrackingError::extractPosition(const geometry_msgs::TransformStamped& tf_in) {
	Eigen::Vector3d vec_position(tf_in.transform.translation.x, tf_in.transform.translation.y, tf_in.transform.translation.z);
	return vec_position;
}

/**
 * Extracts a quaternion from a TransformStamped message.
 *
 * @param tf_in the input TransformStamped message
 *
 * @return the extracted quaternion as a Quaterniond object
 *
 * @throws None
 */
Eigen::Quaterniond TFTrackingError::extractQuaternion(const geometry_msgs::TransformStamped& tf_in) {
	Eigen::Quaterniond vec_quaternion(tf_in.transform.rotation.w, tf_in.transform.rotation.x, tf_in.transform.rotation.y, tf_in.transform.rotation.z);
	return vec_quaternion;
}

/**
 * Calculate the quaternion error between a tracked quaternion and a target quaternion.
 *
 * @param quat_tracked the tracked quaternion
 * @param quat_target the target quaternion
 *
 * @return the vector representing the quaternion error
 *
 * @throws None
 */
Eigen::Vector3d TFTrackingError::calculateQuatError(const Eigen::Quaterniond& quat_tracked, const Eigen::Quaterniond& quat_target) {
	Eigen::Quaterniond quat_error;

	// calculate the error
	quat_error = quat_target * quat_tracked.conjugate();
	
	double angle = 2*acos(quat_error.w());
	double norm = quat_error.vec().norm();

	// calculate the axis and angle
	Eigen::Vector3d axis;
	if(norm > 1e-4) {
		axis = quat_error.vec()/norm;
	}
	else { // the error is negligable
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

/**
 * Limits the magnitude of the given vector to the specified maximum value.
 *
 * @param vec The vector to be limited.
 * @param norm The maximum magnitude allowed for the vector.
 */
void TFTrackingError::limitToMax(Eigen::Vector3d& vec, double norm) {
	if(vec.norm() > norm) {
		vec = vec/vec.norm()*norm;
	}
}

int main(int argc, char** argv) {
	ros::init(argc, argv, "tf_tracking_error");
	ros::NodeHandle n;
	TFTrackingError tf_tracking_error(&n);
	ros::spin();
	return 0;
}