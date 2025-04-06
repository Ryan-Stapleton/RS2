/**
 * @file static_joint_limit_publisher.cpp
 * @brief A node for publishing static joint limits from a URDF file.
 * 
 * @param robot_description the URDF description of the robot
 * @param control_frame the name of the control frame of the robot
 * @param tool_frame the name of the tool/end effector frame
 * @param rate the rate at which the node runs
 * 
 * Publishers:
 * 	- joint_limits: The joint limits of the robot
 */

#include "trackit_robot_state/static_joint_limit_publisher.h"

/**
 * Constructor for the StaticJointLimitPublisher class.
 *
 * @param n pointer to ros::NodeHandle object
 *
 * @return None
 *
 * @throws None
 */
StaticJointLimitPublisher::StaticJointLimitPublisher(ros::NodeHandle* n):_n(*n) {
	// private nodehandle
	_nh = ros::NodeHandle("~");

	// node parameters
	_n.param("robot_description"	, _robot_description	, std::string());
	
	_nh.param<std::string>("control_frame"	, _control_frame	, std::string());
	_nh.param<std::string>("tool_frame"		, _tool_frame		, std::string());

	_nh.param<double>("rate", _rate, 500.0);

	// publisher
	_pub_joint_limits = _n.advertise<trackit_msgs::JointLimitsStamped>("joint_limits", 1);

	// timer
	_timer = _n.createTimer(ros::Duration(1.0/_rate), &StaticJointLimitPublisher::timerCallback, this);

	// set up kdl tree and chain
	if(!kdl_parser::treeFromString(_robot_description, _kdl_tree)) {
		ROS_ERROR("KDL tree was not able to be constructed from the robot description, shutting down the joint limit uploader node");
		ros::shutdown();
		return;
	}

	if(!_kdl_tree.getChain(_control_frame, _tool_frame, _kdl_chain)) {
		ROS_ERROR("KDL chain was not able to be obtained, shutting down the joint limit uploader node");
		ros::shutdown();
		return;
	}

	// get the robot model
	if(!_robot_model.initString(_robot_description)) {
		ROS_ERROR("Robot urdf model was not able to be parsed from the robot description, shutting down the joint limit uploader node");
		ros::shutdown();
		return;
	}

	// get the joint names and limits
	int num_seg = _kdl_chain.getNrOfSegments();

	for (int i = 0; i < num_seg; i++) {
		std::string joint_name = _kdl_chain.getSegment(i).getJoint().getName();

		if(_kdl_chain.getSegment(i).getJoint().getType() != KDL::Joint::JointType::None) {
			_joint_names.push_back(joint_name);
			_joint_position_upper_limits.push_back(_robot_model.getJoint(joint_name)->limits->upper);
			_joint_position_lower_limits.push_back(_robot_model.getJoint(joint_name)->limits->lower);
			_joint_velocity_upper_limits.push_back(_robot_model.getJoint(joint_name)->limits->velocity);
			_joint_velocity_lower_limits.push_back(_robot_model.getJoint(joint_name)->limits->velocity * -1);
			_joint_effort_limits.push_back(_robot_model.getJoint(joint_name)->limits->effort);
		}
	}

	// setup the message
	_msg_joint_limits.header.frame_id = _control_frame;
	_msg_joint_limits.joint_names = _joint_names;

	_msg_joint_limits.position_upper_limits.data.resize(_joint_names.size());
	_msg_joint_limits.position_lower_limits.data.resize(_joint_names.size());
	_msg_joint_limits.velocity_upper_limits.data.resize(_joint_names.size());
	_msg_joint_limits.velocity_lower_limits.data.resize(_joint_names.size());
	_msg_joint_limits.effort_limits.data.resize(_joint_names.size());

	for (int i = 0; i < _joint_names.size(); i++) {
		_msg_joint_limits.position_upper_limits.data[i] = _joint_position_upper_limits[i];
		_msg_joint_limits.position_lower_limits.data[i] = _joint_position_lower_limits[i];
		_msg_joint_limits.velocity_upper_limits.data[i] = _joint_velocity_upper_limits[i];
		_msg_joint_limits.velocity_lower_limits.data[i] = _joint_velocity_lower_limits[i];
		_msg_joint_limits.effort_limits.data[i] = _joint_effort_limits[i];
	}
}

/**
 * Timer callback function for the StaticJointLimitPublisher class.
 *
 * @param event The timer event.
 *
 * @return None.
 *
 * @throws None.
 */
void StaticJointLimitPublisher::timerCallback(const ros::TimerEvent& event) {
	// update the stamp and publish the msg
	_msg_joint_limits.header.stamp = ros::Time::now();
	_pub_joint_limits.publish(_msg_joint_limits);
}

int main(int argc, char** argv) {
	ros::init(argc, argv, "trackit_joint_limit_publisher");
	ros::NodeHandle n;
	StaticJointLimitPublisher joint_limit_publisher(&n);
	ros::spin();
	return 0;
}