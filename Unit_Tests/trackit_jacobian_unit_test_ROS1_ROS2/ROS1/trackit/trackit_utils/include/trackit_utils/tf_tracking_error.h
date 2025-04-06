#ifndef TRACKIT_UTILS_TF_TRACKING_ERROR_H
#define TRACKIT_UTILS_TF_TRACKING_ERROR_H

#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <eigen3/Eigen/Dense>

class TFTrackingError {

public:
	TFTrackingError(ros::NodeHandle* n);

private:
	void timerCallback(const ros::TimerEvent&);

	Eigen::Vector3d extractPosition(const geometry_msgs::TransformStamped& tf_in);
	Eigen::Quaterniond extractQuaternion(const geometry_msgs::TransformStamped& tf_in);
	Eigen::Vector3d calculateQuatError(const Eigen::Quaterniond& quat_tracked, const Eigen::Quaterniond& quat_target);

	void limitToMax(Eigen::Vector3d& vec, double norm);

	ros::NodeHandle _n;
	ros::NodeHandle _nh;

	ros::Timer _timer;
	ros::Publisher _pub_twist;

	tf2_ros::Buffer _tf_buffer;
	tf2_ros::TransformListener _tf_listen;

	double _rate;
	double _max_lin_velocity;
	double _max_ang_velocity;

	std::string _tracked_frame;
	std::string _target_frame;
	std::string _header_frame;

	bool _enable_pos_x;
	bool _enable_pos_y;
	bool _enable_pos_z;
	bool _enable_rot_x;
	bool _enable_rot_y;
	bool _enable_rot_z;
	
};

#endif // TRACKIT_UTILS_TF_TRACKING_ERROR_H