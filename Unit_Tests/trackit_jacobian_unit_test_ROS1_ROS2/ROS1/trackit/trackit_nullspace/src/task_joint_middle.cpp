/**
 * @file task_joint_middle.cpp
 * @brief Implementation of the JointMiddle class functions.
 * 
 * This file contains the implementation of the JointMiddle class functions.
 * This node calculates a joint velocity task to move the robot towards the middle of the joint limits
 * 
 * @param max_velocity the maximum velocity for the joint velocity task
 * 
 * Subscribers:
 * 	- joint_limits: The joint limits of the robot.
 * 	- joint_states: The current joint state of the robot.
 * 
 * Publishers:
 * 	- nullspace_joint_velocity: The calculated joint velocity to be sent to the nullspace projection.
 * 
 */

#include "trackit_nullspace/task_joint_middle.h"

/**
 * Constructor for the JointMiddle class.
 *
 * @param n A pointer to a `ros::NodeHandle` object.
 *
 * @throws None
 */
JointMiddle::JointMiddle(ros::NodeHandle* n):_n(*n) {
	// private nodehandle
	_nh = ros::NodeHandle("~");
	
	// node parameters
	_nh.param<double>("max_velocity", _max_velocity, 0.1);

	// publishers
	_pub_joint_velocity = _n.advertise<trackit_msgs::JointVelocityStamped>("nullspace_joint_velocity", 1);

	// subscribers
	_sub_joint_limits = _n.subscribe("joint_limits", 1, &JointMiddle::jointLimitsCallback, this);
	_sub_joint_states = _n.subscribe("joint_states", 1, &JointMiddle::jointStateCallback, this);
	
	// initialize joint names
	_joint_names.resize(0);
	_joint_names.clear();

}

/**
 * Main callback function for the joint state.
 * This callback calculates the joint velocity from the latest joint limits .
 * 
 * @param msg Pointer to the joint state message.
 *
 * @throws None
 */
void JointMiddle::jointStateCallback(const sensor_msgs::JointState::ConstPtr& msg) {

	if(_joint_names.size() > 0) {

		Eigen::MatrixXd current_joint_position;
		current_joint_position.resize(_joint_names.size(),1);

		for (int i = 0; i < _joint_names.size(); i++) {
			current_joint_position(i,0) = msg->position[std::find(msg->name.begin(), msg->name.end(), _joint_names[i]) - msg->name.begin()];	
		}

		Eigen::MatrixXd difference = _midpoint - current_joint_position;
		Eigen::MatrixXd magnitude = difference/_max_velocity;
		magnitude = magnitude.cwiseAbs();

		if(magnitude.maxCoeff() > 1) {
			difference = difference/magnitude.maxCoeff();
		}

		trackit_msgs::JointVelocityStamped msg_joint_velocity;
		msg_joint_velocity.header = msg->header;
		msg_joint_velocity.joint_names = _joint_names;

		msg_joint_velocity.joint_velocity.data.resize(_joint_names.size());
		for (int i = 0; i < _joint_names.size(); i++) {
			msg_joint_velocity.joint_velocity.data[i] = difference(i,0);
		}
		
		_pub_joint_velocity.publish(msg_joint_velocity);
	}
	
}

/**
 * Callback function for joint limits.
 *
 * @param msg Pointer to the joint limits message
 *
 * @return void
 *
 * @throws None
 */
void JointMiddle::jointLimitsCallback(const trackit_msgs::JointLimitsStamped::ConstPtr& msg) {

	_joint_names = msg->joint_names;
	msgToMatrix(msg->position_upper_limits, _joint_upper_limits);
	msgToMatrix(msg->position_lower_limits, _joint_lower_limits);

	_midpoint.resize(_joint_names.size(),1);
	_midpoint.setZero();

	_midpoint = (_joint_upper_limits + _joint_lower_limits)/2;
}

/**
 * Insert the values from the Float64MultiArray message into the Eigen matrix
 *
 * @param msg The input Float64MultiArray message
 * @param mat The Eigen matrix to be filled
 *
 * @throws None
 */
void JointMiddle::msgToMatrix(const std_msgs::Float64MultiArray& msg, Eigen::MatrixXd& mat) {
	mat.resize(msg.data.size(),1);
	mat.setZero();
	for (int i = 0; i < msg.data.size(); i++) {
		mat(i,0) = msg.data[i];
	}
}

int main(int argc, char** argv) {
	ros::init(argc, argv, "task_joint_middle_node");
	ros::NodeHandle n;
	JointMiddle task_joint_middle(&n);
	ros::spin();
	return 0;
}