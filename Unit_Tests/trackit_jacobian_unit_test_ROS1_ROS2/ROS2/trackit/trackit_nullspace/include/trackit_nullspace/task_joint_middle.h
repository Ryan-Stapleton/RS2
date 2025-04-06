#ifndef TRACKIT_NULLSPACE_TASK_JOINT_MIDDLE_H
#define TRACKIT_NULLSPACE_TASK_JOINT_MIDDLE_H

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include <eigen3/Eigen/Dense>

#include "trackit_msgs/msg/joint_limits_stamped.hpp"
#include "trackit_msgs/msg/joint_velocity_stamped.hpp"

class JointMiddle : public rclcpp::Node{

public:
	JointMiddle();

private:
	void jointLimitsCallback(const std::shared_ptr<trackit_msgs::msg::JointLimitsStamped> msg);
	void jointStateCallback(const std::shared_ptr<sensor_msgs::msg::JointState> msg);

	void msgToMatrix(const std_msgs::msg::Float64MultiArray& msg, Eigen::MatrixXd& mat);

	// ros::NodeHandle _n;
	std::shared_ptr<rclcpp::Node> _n = rclcpp::Node::make_shared("_n");
	// ros::NodeHandle _nh;
	std::shared_ptr<rclcpp::Node> _nh = rclcpp::Node::make_shared("_nh");

	// ros::Publisher _pub_joint_velocity;
	rclcpp::Publisher<trackit_msgs::msg::JointVelocityStamped>::SharedPtr _pub_joint_vel;
	// ros::Subscriber _sub_joint_limits;
	rclcpp::Subscription<trackit_msgs::msg::JointLimitsStamped>::SharedPtr _sub_joint_limits;
	// ros::Subscriber _sub_joint_states;
	rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr _sub_joint_states;

	std::vector<std::string> 	_joint_names;

	Eigen::MatrixXd _joint_upper_limits;
	Eigen::MatrixXd _joint_lower_limits;
	Eigen::MatrixXd _midpoint;

	double _max_velocity;
};
#endif // TRACKIT_NULLSPACE_TASK_JOINT_MIDDLE_H