/**
 * @file joint_limit_aggregator.cpp
 * @brief Implementation of the JointLimitAggregator class functions.
 * 
 * This node combines the different joint limits messages recevied into a single message.
 * 
 * @param robot_description the URDF description of the robot
 * @param control_frame the name of the control frame
 * @param tool_frame the name of the tool frame
 * @param msg_timeout the timeout for the recency of the joint limits messages
 * @param rate the rate at which the node runs
 *  
 * Subscribers:
 * 	- joint_limits_in: The joint limits messages from other nodes.
 * 
 * Publishers:
 * 	- joint_limits_out: The aggregated joint limits
 * 
 */

#include "trackit_robot_state/joint_limit_aggregator.h"

/**
 * Constructor for the JointLimitAggregator class.
 *
 * @param n pointer to a NodeHandle object
 *
 * @return None
 *
 * @throws None
 */
JointLimitAggregator::JointLimitAggregator(ros::NodeHandle* n):_n(*n) {
	// private nodehandle
	_nh = ros::NodeHandle("~");

	// node parameters
	_n.param("robot_description"	, _robot_description	, std::string());

	_nh.param<std::string>("control_frame"		, _control_frame		, std::string());
	_nh.param<std::string>("tool_frame"			, _tool_frame			, std::string());

	_nh.param<double>("msg_timeout"	, _msg_timeout	, 0.1);
	_nh.param<double>("rate"		, _rate			, 500);

	// publishers
	_pub_joint_limits = _n.advertise<trackit_msgs::JointLimitsStamped>("joint_limits_out", 1);

	// subscribers
	_sub_joint_limits = _n.subscribe("joint_limits_in", 1, &JointLimitAggregator::jointLimitsCallback, this);

	// timer
	_timer = _n.createTimer(ros::Duration(1.0/_rate), &JointLimitAggregator::timerCallback, this);

	// set up kdl tree and chain
	if(!kdl_parser::treeFromString(_robot_description, _kdl_tree)) {
		ROS_ERROR("KDL tree was not able to be constructed from the robot description, shutting down the joint limit aggregator node");
		ros::shutdown();
		return;
	}

	if(!_kdl_tree.getChain(_control_frame, _tool_frame, _kdl_chain)) {
		ROS_ERROR("KDL chain was not able to be obtained, shutting down the joint limit aggregator node");
		ros::shutdown();
		return;
	}

	if(!_robot_model.initString(_robot_description)) {
		ROS_ERROR("Robot urdf model was not able to be parsed from the robot description, shutting down the joint limit aggregator node");
		ros::shutdown();
		return;
	}

	int num_seg = _kdl_chain.getNrOfSegments();

	for (int i = 0; i < num_seg; i++) {
		std::string joint_name = _kdl_chain.getSegment(i).getJoint().getName();

		// only include joints that are not fixed
		if(_kdl_chain.getSegment(i).getJoint().getType() != KDL::Joint::JointType::None) {
			_joint_names.push_back(joint_name);

			_joint_position_upper_urdf_limits.push_back(_robot_model.getJoint(joint_name)->limits->upper);
			_joint_position_lower_urdf_limits.push_back(_robot_model.getJoint(joint_name)->limits->lower);
			_joint_position_upper_limits.push_back(_robot_model.getJoint(joint_name)->limits->upper);
			_joint_position_lower_limits.push_back(_robot_model.getJoint(joint_name)->limits->lower);
			
			_joint_velocity_urdf_limits.push_back(_robot_model.getJoint(joint_name)->limits->velocity);
			_joint_velocity_upper_limits.push_back(_robot_model.getJoint(joint_name)->limits->velocity);
			_joint_velocity_lower_limits.push_back(-_robot_model.getJoint(joint_name)->limits->velocity);
			
			_joint_effort_urdf_limits.push_back(_robot_model.getJoint(joint_name)->limits->effort);
			_joint_effort_limits.push_back(_robot_model.getJoint(joint_name)->limits->effort);
		}
	}

	// set up joint limits message
	_msg_joint_limits.header.frame_id = _control_frame;
	_msg_joint_limits.joint_names = _joint_names;

	_msg_joint_limits.position_upper_limits.data = _joint_position_upper_limits;
	_msg_joint_limits.position_lower_limits.data = _joint_position_lower_limits;
	_msg_joint_limits.velocity_upper_limits.data = _joint_velocity_upper_limits;
	_msg_joint_limits.velocity_lower_limits.data = _joint_velocity_lower_limits;
	_msg_joint_limits.effort_limits.data = _joint_effort_limits;

	// initialize time stamps
	ros::Time init_time = ros::Time(0.0);	
	for (int i = 0; i < _joint_names.size(); i++) {
		_joint_position_upper_timestamps.push_back(init_time);
		_joint_position_lower_timestamps.push_back(init_time);
		_joint_velocity_upper_timestamps.push_back(init_time);
		_joint_velocity_lower_timestamps.push_back(init_time);
		_joint_effort_timestamps.push_back(init_time);
	}
}

void JointLimitAggregator::jointLimitsCallback(const trackit_msgs::JointLimitsStamped::ConstPtr& msg) {
	// Need to check if the joint limits that came in is more restrictive than what we already have
	// If so update the values and the time stamps
	for (int i= 0; i < _joint_names.size(); i++) {
		if(!msg->position_upper_limits.data.empty() && msg->position_upper_limits.data.size() == _joint_names.size()) {
			if(_joint_position_upper_limits[i] > msg->position_upper_limits.data[i]) {
				_joint_position_upper_limits[i] = msg->position_upper_limits.data[i];
				_joint_position_upper_timestamps[i] = msg->header.stamp;
			}
		}

		if(!msg->position_lower_limits.data.empty() && msg->position_lower_limits.data.size() == _joint_names.size()) {
			if(_joint_position_lower_limits[i] < msg->position_lower_limits.data[i]) {
				_joint_position_lower_limits[i] = msg->position_lower_limits.data[i];
				_joint_position_lower_timestamps[i] = msg->header.stamp;
			}
		}

		if(!msg->velocity_upper_limits.data.empty() && msg->velocity_upper_limits.data.size() == _joint_names.size()) {
			if(_joint_velocity_upper_limits[i] > msg->velocity_upper_limits.data[i]) {
				_joint_velocity_upper_limits[i] = msg->velocity_upper_limits.data[i];
				_joint_velocity_upper_timestamps[i] = msg->header.stamp;
			}
		}

		if(!msg->velocity_lower_limits.data.empty() && msg->velocity_lower_limits.data.size() == _joint_names.size()) {
			if(_joint_velocity_lower_limits[i] < msg->velocity_lower_limits.data[i]) {
				_joint_velocity_lower_limits[i] = msg->velocity_lower_limits.data[i];
				_joint_velocity_lower_timestamps[i] = msg->header.stamp;
			}
		}
		
		if(!msg->effort_limits.data.empty() && msg->effort_limits.data.size() == _joint_names.size()) {
			if(_joint_effort_limits[i] > msg->effort_limits.data[i]) {
				_joint_effort_limits[i] = msg->effort_limits.data[i];
				_joint_effort_timestamps[i] = msg->header.stamp;
			}
		}

	}
}

/**
 * Publishes joint limits based on current time and compares them with urdf values.
 * If the limits are recent enough, it uses the more restrictive one.
 * If the limits are not recent enough, it uses the urdf values.
 *
 * @param event The ros::TimerEvent object representing the timer event.
 *
 * @return void
 */
void JointLimitAggregator::timerCallback(const ros::TimerEvent& event) {
	// Publish joint limits
	// If the limits are recent enough compare it with the urdf values and use the more restrictive one
	// If the limits are not recent enough just use the urdf values

	ros::Time current_time = ros::Time::now();
	for(int i = 0; i < _joint_names.size(); i++) {
		
		if(isRecent(current_time, _joint_position_upper_timestamps[i])) {
			if(_joint_position_upper_urdf_limits[i] > _joint_position_upper_limits[i]) {
				_msg_joint_limits.position_upper_limits.data[i] = _joint_position_upper_limits[i];
			}
			else {
				_msg_joint_limits.position_upper_limits.data[i] = _joint_position_upper_urdf_limits[i];
			}	
		}
		else {
			_msg_joint_limits.position_upper_limits.data[i] = _joint_position_upper_urdf_limits[i];
			_joint_position_upper_limits[i] = _joint_position_upper_urdf_limits[i];
		}
		

		if(isRecent(current_time, _joint_position_lower_timestamps[i])) {
			if(_joint_position_lower_urdf_limits[i] < _joint_position_lower_limits[i]) {
				_msg_joint_limits.position_lower_limits.data[i] = _joint_position_lower_limits[i];
			}
			else {
				_msg_joint_limits.position_lower_limits.data[i] = _joint_position_lower_urdf_limits[i];
			}
		}
		else {
			_msg_joint_limits.position_lower_limits.data[i] = _joint_position_lower_urdf_limits[i];
			_joint_position_lower_limits[i] = _joint_position_lower_urdf_limits[i];
		}

		if(isRecent(current_time, _joint_velocity_upper_timestamps[i])) {
			if(_joint_velocity_urdf_limits[i] > _joint_velocity_upper_limits[i]) {
				_msg_joint_limits.velocity_upper_limits.data[i] = _joint_velocity_upper_limits[i];
			}
			else {
				_msg_joint_limits.velocity_upper_limits.data[i] = _joint_velocity_urdf_limits[i];
			}
		}
		else {
			_msg_joint_limits.velocity_upper_limits.data[i] = _joint_velocity_urdf_limits[i];
			_joint_velocity_upper_limits[i] = _joint_velocity_urdf_limits[i];
		}

		if(isRecent(current_time, _joint_velocity_lower_timestamps[i])) {
			if(-_joint_velocity_urdf_limits[i] < _joint_velocity_lower_limits[i]) {
				_msg_joint_limits.velocity_lower_limits.data[i] = _joint_velocity_lower_limits[i];
			}
			else {
				_msg_joint_limits.velocity_lower_limits.data[i] = -_joint_velocity_urdf_limits[i];
			}
		}
		else {
			_msg_joint_limits.velocity_lower_limits.data[i] = -_joint_velocity_urdf_limits[i];
			_joint_velocity_lower_limits[i] = -_joint_velocity_urdf_limits[i];
		}

		if(isRecent(current_time, _joint_effort_timestamps[i])) {
			if(_joint_effort_urdf_limits[i] < _joint_effort_limits[i]) {
				_msg_joint_limits.effort_limits.data[i] = _joint_effort_limits[i];
			}
			else {
				_msg_joint_limits.effort_limits.data[i] = _joint_effort_urdf_limits[i];
			}
		}
		else {
			_msg_joint_limits.effort_limits.data[i] = _joint_effort_urdf_limits[i];
			_joint_effort_limits[i] = _joint_effort_urdf_limits[i];
		}

		_msg_joint_limits.header.stamp = current_time;
		_pub_joint_limits.publish(_msg_joint_limits);
	}
}

/**
 * Determines if the given message time is recent based on the current time and a timeout value.
 *
 * @param current_time the current time
 * @param msg_time the message time to check
 *
 * @return true if the message time is recent, false otherwise
 *
 */
bool JointLimitAggregator::isRecent(const ros::Time current_time, const ros::Time msg_time) {
	if(std::fabs((current_time - msg_time).toSec()) > _msg_timeout) {
		return false;
	}
	return true;
}

int main(int argc, char** argv) {
	ros::init(argc, argv, "joint_limit_aggregator");
	ros::NodeHandle n;
	JointLimitAggregator joint_limit_aggregator(&n);
	ros::spin();
	return 0;
}