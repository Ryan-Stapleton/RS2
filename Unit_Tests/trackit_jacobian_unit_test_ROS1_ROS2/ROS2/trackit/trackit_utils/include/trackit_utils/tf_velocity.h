#ifndef TRACKIT_UTILS_TF_VELOCITY_H
#define TRACKIT_UTILS_TF_VELOCITY_H

#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <eigen3/Eigen/Dense>

class TFVelocity {

public:
	TFVelocity(ros::NodeHandle* n);

private:
	void timerCallback(const ros::TimerEvent&);

	Eigen::Vector3d extractPosition(const geometry_msgs::TransformStamped& tf_in);
	Eigen::Quaterniond extractQuaternion(const geometry_msgs::TransformStamped& tf_in);
	Eigen::Vector3d calculateQuatError(const Eigen::Quaterniond& quat_tracked, const Eigen::Quaterniond& quat_target);

	ros::NodeHandle _n;
	ros::NodeHandle _nh;

	ros::Timer _timer;
	ros::Publisher _pub_twist;

	tf2_ros::Buffer _tf_buffer;
	tf2_ros::TransformListener _tf_listen;

	bool _first;

	double _rate;

	std::string _tracked_frame;
	std::string _reference_frame;
	
	geometry_msgs::TransformStamped _tf_prev;
};

#endif // TRACKIT_UTILS_TF_VELOCITY_H