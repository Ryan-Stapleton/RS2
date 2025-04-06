#ifndef TRACK_IT_TRACK_IT_JACOBIAN_H
#define TRACK_IT_TRACK_IT_JACOBIAN_H

#include <rclcpp/rclcpp.hpp>

#include <sensor_msgs/msg/joint_state.hpp>

#include <kdl/chainjnttojacsolver.hpp>
#include <kdl/jacobian.hpp>
#include <kdl/kdl.hpp>
#include <kdl/segment.hpp>
#include <kdl/joint.hpp>
#include <kdl_parser/kdl_parser.hpp>

#include <boost/scoped_ptr.hpp>

#include <eigen3/Eigen/Dense>

#include <algorithm>

#include "trackit_msgs/msg/jacobian_stamped.hpp"


class TrackItJacobian : public rclcpp::Node {

public:
	TrackItJacobian(const rclcpp::NodeOptions &options);

private:
	void jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg);

	rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr _sub_joint_state;
	rclcpp::Publisher<trackit_msgs::msg::JacobianStamped>::SharedPtr _pub_jacobian;
	rclcpp::Publisher<trackit_msgs::msg::JacobianStamped>::SharedPtr _pub_jacobian_inverse;

	std::string _robot_description;
	std::string _control_frame;
	std::string _tool_frame;
	std::vector<std::string> _joint_names;

	KDL::Tree 	_kdl_tree;
	KDL::Chain	_kdl_chain;

	std::unique_ptr<KDL::ChainJntToJacSolver> _jacobian_solver; //boost::scoped_ptr<KDL::ChainJntToJacSolver> _jacobian_solver;

	Eigen::MatrixXd _prev_jacobian;

	int _num_joints;

	bool _damped_inverse;
	
	double _damping;

};

#endif //TRACK_IT_TRACK_IT_JACOBIAN