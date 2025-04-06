#ifndef TRACKIT_NULLSPACE_NULLSPACE_PROJECTION_H
#define TRACKIT_NULLSPACE_NULLSPACE_PROJECTION_H

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/time.hpp"
#include <eigen3/Eigen/Dense>

#include "trackit_msgs/msg/jacobian_stamped.hpp"
#include "trackit_msgs/msg/joint_velocity_stamped.hpp"

class NullSpace : public rclcpp::Node{

public:
	NullSpace();

private:
	void jointVelocityCallback(const std::shared_ptr<trackit_msgs::msg::JointVelocityStamped> msg);
	void jacobianCallback(const std::shared_ptr<trackit_msgs::msg::JacobianStamped> msg);

	bool isMsgCurrent(rclcpp::Time time);

	Eigen::MatrixXd jointVelocityMsgToEigen(const trackit_msgs::msg::JointVelocityStamped msg);
	std_msgs::msg::Float64MultiArray eigenToJointVelocityMsg(const Eigen::MatrixXd& eig_joint_velocity);
	Eigen::MatrixXd pseudoInverse(const Eigen::MatrixXd&, const double lambda);

	// ros::NodeHandle _n;
	std::shared_ptr<rclcpp::Node> _n = rclcpp::Node::make_shared("_n");
	// ros::NodeHandle _nh;
	std::shared_ptr<rclcpp::Node> _nh = rclcpp::Node::make_shared("_nh");

	// ros::Subscriber _sub_joint_vel;
	rclcpp::Subscription<trackit_msgs::msg::JointVelocityStamped>::SharedPtr _sub_joint_vel;
	// ros::Subscriber _sub_jacobian;
	rclcpp::Subscription<trackit_msgs::msg::JacobianStamped>::SharedPtr _sub_jacobian;

	// ros::Publisher _pub_projected_joint_vel;
	rclcpp::Publisher<trackit_msgs::msg::JointVelocityStamped>::SharedPtr _pub_projected_joint_vel;

	trackit_msgs::msg::JointVelocityStamped _msg_joint_velocity;

	Eigen::MatrixXd _eig_jacobian;

	bool _first_jacobian_msg;

	double _time_diff_threshold;
	double _lambda;

	std::vector<std::string> _joint_names;

	int _joint_num;
};

#endif