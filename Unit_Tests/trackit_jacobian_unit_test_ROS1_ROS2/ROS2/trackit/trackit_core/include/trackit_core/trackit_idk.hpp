#ifndef TRACKIT_CORE_IDK_H
#define TRACKIT_CORE_IDK_H

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/SVD>

#include "trackit_msgs/msg/jacobian_stamped.hpp"
#include "trackit_msgs/msg/joint_limits_stamped.hpp"
#include "trackit_msgs/msg/joint_velocity_stamped.hpp"

class IDK : public rclcpp::Node {
public:
	IDK();

private:
	void twistInCallback(const geometry_msgs::msg::TwistStamped::SharedPtr msg);
	void twistExternalCallback(const geometry_msgs::msg::TwistStamped::SharedPtr msg);
	
	void jointVelocityExternalCallback(const trackit_msgs::msg::JointVelocityStamped::SharedPtr  msg);
	void jointVelocityNullCallback(const trackit_msgs::msg::JointVelocityStamped::SharedPtr msg);
	void jointLimitsCallback(const trackit_msgs::msg::JointLimitsStamped::SharedPtr  msg);
	
	void jacobianCallback(const trackit_msgs::msg::JacobianStamped::SharedPtr msg);
	
	void timerCallback();

	void checkWithinLimit(Eigen::MatrixXd& joint_velocity);

	bool isMsgCurrent(rclcpp::Time msg_time);

	Eigen::Matrix<double,6,1> twistMsgToEigen(const geometry_msgs::msg::TwistStamped msg_twist);
	Eigen::MatrixXd jointVelocityMsgToEigen(const trackit_msgs::msg::JointVelocityStamped msg);

	rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr _sub_twist_in;
	rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr _sub_twist_external;
	rclcpp::Subscription<trackit_msgs::msg::JointVelocityStamped>::SharedPtr _sub_joint_external;
	rclcpp::Subscription<trackit_msgs::msg::JointVelocityStamped>::SharedPtr _sub_joint_null;
	rclcpp::Subscription<trackit_msgs::msg::JacobianStamped>::SharedPtr _sub_jacobian;
	rclcpp::Subscription<trackit_msgs::msg::JointLimitsStamped>::SharedPtr _sub_joint_limits;

	rclcpp::Publisher<trackit_msgs::msg::JointVelocityStamped>::SharedPtr _pub_joint_velocity;

	rclcpp::TimerBase::SharedPtr _timer;

	geometry_msgs::msg::TwistStamped _msg_twist_in;
	geometry_msgs::msg::TwistStamped _msg_twist_external;
	trackit_msgs::msg::JointVelocityStamped	_msg_joint_external;
	trackit_msgs::msg::JointVelocityStamped _msg_joint_null;
	trackit_msgs::msg::JacobianStamped _msg_jacobian_inverse;

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

	std::vector<long int> _mask;

};

#endif // TRACKIT_CORE_IDK_H