/**
 * @file joint_velocity_limit_publisher.cpp
 * @brief A node for publishing joint velocity limits.
 * This node calculates the allowable joint velocity such that the joint position limit would not be violated.
 * 
 * @param robot_description the URDF description of the robot
 * @param control_frame the name of the control frame of the robot 
 * @param tool_frame the name of the tool/end effector frame
 * @param time_to_pos_limit the time to reach the position limit
 * @param position_limit_buffer the buffer for the position limit
 *  
 * Subscribers:	
 * 	- joint_states: The joint states of the robot
 * 
 * Publishers:
 * 	- joint_limits: The joint limits
 */

#include "trackit_robot_state/joint_velocity_limit_publisher.h"

/**
 * Constructor for the JointVelocityLimitPublisher class.
 *
 * @param n A pointer to the ros::NodeHandle object.
 *
 */
JointVelocityLimitPublisher::JointVelocityLimitPublisher(ros::NodeHandle* n):_n(*n) {
	// private nodehandle
	_nh = ros::NodeHandle("~");

	// node parameters
	_n.param("robot_description"	, _robot_description	, std::string());
	
	_nh.param<std::string>("control_frame"	, _control_frame	, std::string());
	_nh.param<std::string>("tool_frame"		, _tool_frame		, std::string());

	_nh.param<double>("time_to_pos_limit", _time_to_pos_limit, 5.0);
	_nh.param<double>("position_limit_buffer", _position_limit_buffer, 0.1);

	// subscribers
	_sub_joint_state = _n.subscribe("joint_states", 1, &JointVelocityLimitPublisher::jointStateCallback, this);

	// publishers
	_pub_joint_limits = _n.advertise<trackit_msgs::JointLimitsStamped>("joint_limits", 1);

	// set up kdl tree and chain	
	if(!kdl_parser::treeFromString(_robot_description, _kdl_tree)) {
		ROS_ERROR("KDL tree was not able to be constructed from the robot description, shutting down the joint velocity limit publisher node");
		ros::shutdown();
		return;
	}

	if(!_kdl_tree.getChain(_control_frame, _tool_frame, _kdl_chain)) {
		ROS_ERROR("KDL chain was not able to be obtained, shutting down the joint velocity limit publisher node");
		ros::shutdown();
		return;
	}

	// get the robot model
	if(!_robot_model.initString(_robot_description)) {
		ROS_ERROR("Robot urdf model was not able to be parsed from the robot description, shutting down the joint velocity limit publisher node");
		ros::shutdown();
		return;
	}

	// get the joint names
	int num_seg = _kdl_chain.getNrOfSegments();

	for (int i = 0; i < num_seg; i++) {
		std::string joint_name = _kdl_chain.getSegment(i).getJoint().getName();

		if(_kdl_chain.getSegment(i).getJoint().getType() != KDL::Joint::JointType::None) {
			_joint_names.push_back(joint_name);
			_joint_position_upper_limits.push_back(_robot_model.getJoint(joint_name)->limits->upper - _position_limit_buffer);
			_joint_position_lower_limits.push_back(_robot_model.getJoint(joint_name)->limits->lower + _position_limit_buffer);
			_msg_joint_limits.velocity_upper_limits.data.push_back(0.0);
			_msg_joint_limits.velocity_lower_limits.data.push_back(0.0);
		}
	}

	_msg_joint_limits.header.frame_id = _control_frame;
	_msg_joint_limits.joint_names = _joint_names;

}

/**
 * Callback function that receives joint state messages and computes the velocity limits for each joint.
 * The velocity limits are published as a JointLimitsStamped message.
 * 
 * @param msg A pointer to the sensor_msgs::JointState message that contains the joint positions.
 *
 * @throws None
 */
void JointVelocityLimitPublisher::jointStateCallback(const sensor_msgs::JointStateConstPtr& msg) {
	// compute velocity limits
	for (int i = 0; i < _joint_names.size(); i++) {
		_msg_joint_limits.velocity_upper_limits.data[i] = (_joint_position_upper_limits[i] - msg->position[i]) / _time_to_pos_limit;
		// Making sure that the positive joint velocity can't go below zero
		if(_msg_joint_limits.velocity_upper_limits.data[i] < 0.0) {
			_msg_joint_limits.velocity_upper_limits.data[i] = 0.0;
		}
		_msg_joint_limits.velocity_lower_limits.data[i] = (_joint_position_lower_limits[i] - msg->position[i]) / _time_to_pos_limit;
		// Making sure that the negative joint velocity can't go above zero
		if(_msg_joint_limits.velocity_lower_limits.data[i] > 0.0) {
			_msg_joint_limits.velocity_lower_limits.data[i] = 0.0;
		}
	}
	
	// publish velocity limits
	_msg_joint_limits.header.stamp = ros::Time::now();
	_pub_joint_limits.publish(_msg_joint_limits);
}

int main(int argc, char** argv) {
	ros::init(argc, argv, "trackit_joint_velocity_limit_publisher");
	ros::NodeHandle n;
	JointVelocityLimitPublisher joint_limit_publisher(&n);
	ros::spin();
	return 0;
}