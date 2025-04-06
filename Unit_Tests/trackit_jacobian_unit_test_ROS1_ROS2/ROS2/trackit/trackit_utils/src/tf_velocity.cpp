/**
 * @file tf_velocity.cpp
 * @brief this node calculates the twist performed between the previous frame and the current frame.
 * 
 * @param rate the rate at which the node runs
 * @param reference_frame the frame of the twist messages and tf frames.
 * @param tracked_frame the frame being tracked.
 * 
 * Publishers:
 * 	- twist_tf: the twist performed between the previous frame and the current frame.
 * 
 */

#include "trackit_utils/tf_velocity.h"

/**
 * TFVelocity constructor.
 *
 * @param n Pointer to the ROS node handle.
 *
 * @return None.
 *
 * @throws None.
 */
TFVelocity::TFVelocity(ros::NodeHandle* n):_n(*n), _tf_listen(_tf_buffer) {
	// private nodehandle
	_nh = ros::NodeHandle("~");
	
	// node parameters
	_nh.param<double>("rate", _rate, 500);
	
	_nh.param<std::string>("reference_frame"	, _reference_frame	, "base_link");
	_nh.param<std::string>("tracked_frame"		, _tracked_frame	, "ee_link");

	// timer
	_timer = _n.createTimer(ros::Duration(1.0/_rate), &TFVelocity::timerCallback, this);
	
	// publishers
	_pub_twist = _n.advertise<geometry_msgs::TwistStamped>("twist_tf", 1);

	_first = true;
}

/**
 * Callback function for the timer.
 * The twist is calculated between the previous frame and the current frame in this callback.
 * 
 * @param event the timer event
 *
 * @return void
 *
 * @throws tf2::TransformException if there is an error in transforming the frames
 */
void TFVelocity::timerCallback(const ros::TimerEvent&) {

	geometry_msgs::TwistStamped twist_tf;
	geometry_msgs::TransformStamped tf_tracked;

	// get the latest transform between the tracked frame and the reference frame
	try {
		tf_tracked = _tf_buffer.lookupTransform(_reference_frame, _tracked_frame, ros::Time(0), ros::Duration(1.0));
	}
	catch (tf2::TransformException& ex) {
		ROS_ERROR("%s", ex.what());
		return;
	}

	// the first frame has no previous frame so can't calculate the twist
	if(_first) {
		_tf_prev = tf_tracked;
		_first = false;
	}
	else  {
		// calculate the time difference
		double delta_t = tf_tracked.header.stamp.toSec() - _tf_prev.header.stamp.toSec();
		twist_tf.header = tf_tracked.header;

		if(delta_t == 0.0) {
			// It is likely that it is the same tf frame so setting the twist to zero
			ROS_WARN_STREAM("TF frame and are likeley the same. Setting twist to zero. The two timestamps: "<<tf_tracked.header.stamp.toSec() << ". " << _tf_prev.header.stamp.toSec());
			twist_tf.twist.linear.x = 0.0;
			twist_tf.twist.linear.y = 0.0;
			twist_tf.twist.linear.z = 0.0;
			twist_tf.twist.angular.x = 0.0;
			twist_tf.twist.angular.y = 0.0;
			twist_tf.twist.angular.z = 0.0;

		}
		else {
			// calculate the twist
			Eigen::Vector3d tracked_vec = extractPosition(tf_tracked);
			Eigen::Vector3d prev_vec = extractPosition(_tf_prev);
			Eigen::Vector3d lin_vel_vec = (tracked_vec - prev_vec)/delta_t;

			Eigen::Quaterniond tracked_quat = extractQuaternion(tf_tracked);
			Eigen::Quaterniond prev_quat = extractQuaternion(_tf_prev);
			Eigen::Vector3d ang_vel_vec = calculateQuatError(tracked_quat, prev_quat)/delta_t;

			twist_tf.twist.linear.x = lin_vel_vec[0];
			twist_tf.twist.linear.y = lin_vel_vec[1];
			twist_tf.twist.linear.z = lin_vel_vec[2];

			twist_tf.twist.angular.x = ang_vel_vec[0];
			twist_tf.twist.angular.y = ang_vel_vec[1];
			twist_tf.twist.angular.z = ang_vel_vec[2];

			_tf_prev = tf_tracked;
		}
	}

	// publish the twist
	_pub_twist.publish(twist_tf);
}

/**
 * Extracts the position from a given geometry_msgs::TransformStamped object.
 *
 * @param tf_in The input TransformStamped object.
 *
 * @return The position vector extracted from the input object.
 *
 * @throws None
 */
Eigen::Vector3d TFVelocity::extractPosition(const geometry_msgs::TransformStamped& tf_in) {
	Eigen::Vector3d vec_position(tf_in.transform.translation.x, tf_in.transform.translation.y, tf_in.transform.translation.z);
	return vec_position;
}

/**
 * Extracts a quaternion from a given TransformStamped message.
 *
 * @param tf_in the TransformStamped message to extract the quaternion from
 *
 * @return the extracted quaternion as an Eigen::Quaterniond object
 *
 * @throws None
 */
Eigen::Quaterniond TFVelocity::extractQuaternion(const geometry_msgs::TransformStamped& tf_in) {
	Eigen::Quaterniond vec_quaternion(tf_in.transform.rotation.w, tf_in.transform.rotation.x, tf_in.transform.rotation.y, tf_in.transform.rotation.z);
	return vec_quaternion;
}

/**
 * Calculates the error between two quaternions and returns the corresponding axis-angle representation.
 *
 * @param quat_tracked the tracked quaternion
 * @param quat_target the target quaternion
 *
 * @return the axis-angle representation of the error between the two quaternions
 *
 * @throws None
 */
Eigen::Vector3d TFVelocity::calculateQuatError(const Eigen::Quaterniond& quat_tracked, const Eigen::Quaterniond& quat_target) {
	Eigen::Quaterniond quat_error;
	quat_error = quat_target * quat_tracked.conjugate();
	
	double angle = 2*acos(quat_error.w());
	double norm = quat_error.vec().norm();

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
	ros::init(argc, argv, "tf_velocity");
	ros::NodeHandle n;
	TFVelocity tf_velocity(&n);
	ros::spin();
	return 0;
}