#ifndef TRACK_IT_TRACK_IT_JACOBIAN_H
#define TRACK_IT_TRACK_IT_JACOBIAN_Hkdl trainkdl

#include <ros/ros.h>

#include <sensor_msgs/JointState.h>

#include <kdl/chainjnttojacsolver.hpp>
#include <kdl/jacobian.hpp>
#include <kdl/kdl.hpp>
#include <kdl/segment.hpp>
#include <kdl/joint.hpp>
#include <kdl_parser/kdl_parser.hpp>

#include <boost/scoped_ptr.hpp>

#include <eigen3/Eigen/Dense>

#include <algorithm>

#include "trackit_msgs/JacobianStamped.h"

class TrackItJacobian {

public:
	TrackItJacobian(ros::NodeHandle* n);

private:
	void jointStateCallback(const sensor_msgs::JointState::ConstPtr& msg);
	
	ros::NodeHandle _n;
	ros::NodeHandle _nh;

	ros::Subscriber _sub_joint_state;
	ros::Publisher 	_pub_jacobian;
	ros::Publisher 	_pub_jacobian_inverse;

	std::string _robot_description;
	std::string _control_frame;
	std::string _tool_frame;
	std::vector<std::string> _joint_names;

	KDL::Tree 	_kdl_tree;
	KDL::Chain	_kdl_chain;

	boost::scoped_ptr<KDL::ChainJntToJacSolver> _jacobian_solver;

	Eigen::MatrixXd _prev_jacobian;

	int _num_joints;

	bool _damped_inverse;
	
	double _damping;

};

#endif //TRACK_IT_TRACK_IT_JACOBIAN