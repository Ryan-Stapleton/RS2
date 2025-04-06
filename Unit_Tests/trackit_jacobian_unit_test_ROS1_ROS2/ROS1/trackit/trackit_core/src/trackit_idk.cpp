/**
 * @file trackit_idk.cpp
 *
 * @brief Implementation of the IDK class.
 *
 * This file contains the implementation of the IDK class, which is responsible for performing inverse differential kinematics calculations.
 * The IDK class receives twist and joint velocity messages, as well as joint limit and Jacobian data, and computes the joint velocities
 * required to achieve the desired twist for the robot end-effector.
 *
 * @param control_rate The rate at which the IDK node runs.
 * @param max_linear_velocity The maximum linear velocity of the robot.
 * @param max_angular_velocity The maximum angular velocity of the robot.
 * @param time_difference_limit The allowable time difference for messages to be used in the current calculation.
 * @param control_frame The frame of the twist messages.
 * @param mask The mask is used to determine which dimensions of the twist should be used. The mask is a 6x1 vector where 0 indicates that the corresponding dimension would not be controlled.
 *
 * Subscribers
 * - twist_in: Main twist message in the control frame.
 * - twist_external: Additional twist message to be accounted for. Can be used for collision avoidance, etc.
 * - joint_vel_null: Joint velocity message from the null space task.
 * - joint_vel_external: Additional joint velocity message from another source.
 * - joint_limits: The joint velocity part of the joint limits msgs is used to clamp the joint velocity output.
 * - jacobian: The jacobian of the robot to convert twist to a joint velocity.
 *
 * Publishers
 * - joint_velocity_out: The calculated joint velocity to be sent to the robot.
 */

#include "trackit_core/trackit_idk.h"

/**
 * @brief Constructor for the IDK class.
 *
 * Initializes the IDK class and sets up the necessary ROS node handle.
 *
 * @param n Pointer to the ROS node handle.
 */
IDK::IDK(ros::NodeHandle* n):_n(*n) {
	_nh = ros::NodeHandle("~");

	// node parameters
	_nh.param<double>("control_rate"			, _control_rate				, 500);
	_nh.param<double>("max_linear_velocity"		, _max_lin_vel				, 0.1);
	_nh.param<double>("max_angular_velocity"	, _max_ang_vel				, 1.0);
	_nh.param<double>("time_difference_limit"	, _time_diff_threshold		, 0.1);

	_nh.param<std::string>("control_frame", _control_frame, "base");

	_nh.param<std::vector<int>>("mask", _mask, {1,1,1,1,1,1});
	
	// subscribers
	_sub_twist_in		= _n.subscribe("twist_in"				, 1, &IDK::twistInCallback					, this);
	_sub_twist_external = _n.subscribe("twist_external"			, 1, &IDK::twistExternalCallback			, this);
	_sub_joint_external = _n.subscribe("joint_velocity_external", 1, &IDK::jointVelocityExternalCallback	, this);
	_sub_joint_null 	= _n.subscribe("joint_velocity_null"	, 1, &IDK::jointVelocityNullCallback		, this);
	_sub_jacobian 		= _n.subscribe("jacobian_inverse"		, 1, &IDK::jacobianCallback				, this);
	_sub_joint_limits 	= _n.subscribe("joint_limits"			, 1, &IDK::jointLimitsCallback				, this);

	// publishers
	_pub_joint_velocity = _n.advertise<trackit_msgs::JointVelocityStamped>("joint_velocity_out", 1);

	// timer
	_timer = _n.createTimer(ros::Duration(1.0/_control_rate), &IDK::timerCallback, this);

	// initialise message headers
	_msg_twist_in.header.stamp			= ros::Time(0.0);
	_msg_twist_external.header.stamp	= ros::Time(0.0);
	_msg_joint_external.header.stamp 	= ros::Time(0.0);
	_msg_joint_null.header.stamp		= ros::Time(0.0);
	_msg_jacobian_inverse.header.stamp 	= ros::Time(0.0);

	_first_jacobian_msg = true;

	_joint_num = 0;
	
	_eig_joint_vel_limits.resize(0,0);

	// check the mask and initialise the maskss
	if(_mask.size() != 6) {
		ROS_ERROR("Mask size must be 6, setting to default");
		_mask = {1,1,1,1,1,1};
	}
	else {
		for(int i = 0; i < 6; i++) {
			if(_mask[i] != 0) {
				_mask[i] = 1;
			}
		}
	}


}

/**
 * Updates the _msg_twist_in member variable with the incoming twist message,
 * if it is in the same frame as the control frame. Otherwise, it logs a 
 * warning message.
 *
 * @param msg A pointer to the incoming twist message.
 *
 * @throws None
 */
void IDK::twistInCallback(const geometry_msgs::TwistStamped::ConstPtr& msg) {
	// Check if the incoming twist is in the same frame as the control frame. If not, don't include it
	if(msg->header.frame_id.compare(_control_frame) == 0){
		_msg_twist_in = *msg;
	}
	else{
		ROS_WARN_STREAM_THROTTLE(1,"Twist in has a different frame to the control frame of the velocity control node. Twist in frame: " << msg->header.frame_id << ", control frame: " << _control_frame << ". "<< "Twist Linear: " << msg->twist.linear.x << ", " << msg->twist.linear.y << ", " << msg->twist.linear.z << ". " << "Twist Angular: " << msg->twist.angular.x << ", " << msg->twist.angular.y << ", " << msg->twist.angular.z << ". ");
	}
}

/**
 * Updates the _msg_twist_extynal member variable with the incoming twist message,
 * if it is in the same frame as the control frame. Otherwise, it logs a 
 * warning message.
 *
 * @param msg A pointer to the incoming twist message.
 *
 * @throws None
 */
void IDK::twistExternalCallback(const geometry_msgs::TwistStamped::ConstPtr& msg) {
	// Check if the external twist is in the same frame as the control frame. If not, don't include it
	if(msg->header.frame_id.compare(_control_frame) == 0){
		_msg_twist_external = *msg;
	}
	else{
		ROS_WARN_STREAM_THROTTLE(1,"Twist external has a different frame to the control frame of the velocity control node. Twist external frame: " << msg->header.frame_id << ", control frame: " << _control_frame << ". ");
	}
}

/**
 * Callback function for receiving joint velocities from an external source.
 *
 * @param msg A pointer to a `trackit_msgs::JointVelocityStamped` message containing the joint velocities.
 *
 * @throws None
 */
void IDK::jointVelocityExternalCallback(const trackit_msgs::JointVelocityStamped::ConstPtr& msg) {
	// Check if the incoming joint velocities from the external source has the same amount of joints as expected. If not, don't include it
	if(msg->joint_velocity.data.size() == _joint_num) {
		_msg_joint_external = *msg;
	}
	else {
		ROS_WARN_STREAM_THROTTLE(1,"Joint velocity external has a different number of values. Number of joint velocities: " << msg->joint_velocity.data.size() << ", expected number of joint velocities: " << _joint_num << ".");
	}
}

/**
 * Callback function for received joint velocity from the nullspace node.
 *
 * @param msg A pointer to a `trackit_msgs::JointVelocityStamped` object that contains the incoming joint velocities from the null space task.
 *
 * @throws None
 */
void IDK::jointVelocityNullCallback(const trackit_msgs::JointVelocityStamped::ConstPtr& msg) {
	// Check if the incoming joint velocities from the null space task has the same amount of joints as expected. If not, don't include it
	if(msg->joint_velocity.data.size() == _joint_num) {
		_msg_joint_null = *msg;
	}
	else {
		ROS_WARN_STREAM_THROTTLE(1,"Joint velocity null has a different number of values. Number of joint velocities: " << msg->joint_velocity.data.size() << ", expected number of joint velocities: " << _joint_num << ".");
	}
}

/**
 * Callback function for joint limits.
 *
 * @param msg Pointer to the joint limits message.
 *
 * @throws None
 */
void IDK::jointLimitsCallback(const trackit_msgs::JointLimitsStamped::ConstPtr& msg) {
	// ROS_INFO_STREAM("Received new joint limits");
	// Need to make sure that the joint limits are in the same order as the jacobian
	if(!_first_jacobian_msg) {
		_eig_joint_vel_limits.resize(_joint_num,2);
		_eig_joint_vel_limits.setZero();
		for (int i = 0; i < _joint_names.size(); i++) {
			_eig_joint_vel_limits(i,0) = msg->velocity_lower_limits.data[std::find(msg->joint_names.begin(), msg->joint_names.end(), _joint_names[i]) - msg->joint_names.begin()];
			_eig_joint_vel_limits(i,1) = msg->velocity_upper_limits.data[std::find(msg->joint_names.begin(), msg->joint_names.end(), _joint_names[i]) - msg->joint_names.begin()];
		}
	}
}

/**
 * Callback function for the jacobian message.
 *
 * @param msg Pointer to the incoming jacobian message
 *
 * @throws None
 */
void IDK::jacobianCallback(const trackit_msgs::JacobianStamped::ConstPtr& msg) {
	_msg_jacobian_inverse = *msg;

	// if it is the first jacobian msg, set the number of joints and the joint names
	if(_first_jacobian_msg) {
		_first_jacobian_msg = false;
		_joint_num = _msg_jacobian_inverse.joint_names.size();
		_joint_names.resize(_joint_num);
		_joint_names = _msg_jacobian_inverse.joint_names;
	}

	// Insert the inverted jacobian data into an eigen matrix
	int r = _msg_jacobian_inverse.jacobian.layout.dim[0].size;
	int c = _msg_jacobian_inverse.jacobian.layout.dim[1].size;

	_eig_jacobian_inv.resize(r,c);

	for (int i = 0; i < r; i++)	{
		for (int j = 0; j < c; j++)	{
			_eig_jacobian_inv(i,j) = _msg_jacobian_inverse.jacobian.data[c*i + j];
		}
	}
	
	// Mask the jacobian matrix
	for (int i = 0; i < _joint_num; i++) {
		if(_mask[i] == 0) {
			_eig_jacobian_inv.col(i).setZero();
		}
	}
}

/**
 * Callback function for the timer. This is the main loop of IDK. It computes
 * the joint velocity command and publishes it
 *
 * @param event the timer event object
 *
 * @return void
 *
 * @throws None
 */
void IDK::timerCallback(const ros::TimerEvent&) {
	if(!_first_jacobian_msg) {
		// Skipping if the joint velocity limits is not available
		if(_eig_joint_vel_limits.cols() == 0) {
			return;
		}

		Eigen::Matrix<double, 6, 1> dx;
		dx.setZero();
		
		// Add twist_in if it is recent enough
		if(isMsgCurrent(_msg_twist_in.header.stamp)) {
			dx += twistMsgToEigen(_msg_twist_in);
		}

		// Add twist_external if it is recent enough
		if(isMsgCurrent(_msg_twist_external.header.stamp)) {
			dx += twistMsgToEigen(_msg_twist_external);
		}
		
		// clamp velocity using the linear maximum first
		double linear_pt = dx.head(3).norm();
		if(linear_pt > _max_lin_vel) {
			dx = (dx/linear_pt) * _max_lin_vel;
		}

		// clamp velocity using the angular maximum if necessary
		double angular_pt = dx.tail(3).norm();
		if(angular_pt > _max_ang_vel) {
			dx = (dx/angular_pt) * _max_ang_vel;
		}
		
		Eigen::MatrixXd dq;
		dq.resize(_joint_num,1);
		
		// Calculate dq if the jacobian is recent enough
		if(isMsgCurrent(_msg_jacobian_inverse.header.stamp)) {
			dq = _eig_jacobian_inv * dx;
		}
		else {
			dq.setZero();
			// ROS_WARN("Jacobian message too old");
		}

		// Add the external joint velocity
		if(isMsgCurrent(_msg_joint_external.header.stamp) && _msg_joint_external.joint_velocity.data.size() == _joint_num) {
			Eigen::MatrixXd dq_e = jointVelocityMsgToEigen(_msg_joint_external);
			if(dq_e.rows() == dq.rows() && dq_e.cols() == dq.cols()) {
				dq += dq_e;
			}
			else {
				ROS_WARN("External joint command size does not match the calculated joint command");
			}
		}

		// Add the null space joint velocity
		if((isMsgCurrent(_msg_joint_null.header.stamp)) && _msg_joint_null.joint_velocity.data.size() == _joint_num) {
			Eigen::MatrixXd dq_n = jointVelocityMsgToEigen(_msg_joint_null);
			if(dq_n.rows() == dq.rows() && (dq_n.cols() == dq.cols())) {
				dq += dq_n;
			}
			else {
				ROS_WARN("Null space joint command size does not match the calculated joint command");
			}
		} 

		// Clamp the velocity to within the joint velocity limits
		checkWithinLimit(dq);

		// Publish the joint velocity
		if(!_first_jacobian_msg) {
			trackit_msgs::JointVelocityStamped joint_vel_msg;
			joint_vel_msg.joint_names = _joint_names;
			joint_vel_msg.joint_velocity.data.resize(_joint_num);

			for(int i = 0; i < _joint_num; i++) {
				joint_vel_msg.joint_velocity.data[i] = dq(i);
			}

			_pub_joint_velocity.publish(joint_vel_msg);	
		}
	}
}

/**
 * Check if the current time and the given time is within the time difference threshold.
 *
 * @param time The time to compare with the current time.
 *
 * @return True if the time is within the threshold, false otherwise.
 *
 * @throws None
 */
bool IDK::isMsgCurrent(ros::Time time) {
	// check if the current time and the given time is within the time difference threshold
	ros::Time now = ros::Time::now();
	if ((now.toSec() - time.toSec()) > _time_diff_threshold) {
		return false;
	}
	else {
		return true;
	}
}

/**
 * Converts a geometry_msgs::TwistStamped message into an Eigen::Matrix<double, 6, 1>.
 *
 * @param msg The input geometry_msgs::TwistStamped message to be converted.
 *
 * @return An Eigen::Matrix<double, 6, 1> representing the converted twist.
 *
 * @throws None
 */
Eigen::Matrix<double, 6, 1> IDK::twistMsgToEigen(const geometry_msgs::TwistStamped msg) {
	// Convert the twist msg into an eigen matrix and apply the mask
	Eigen::Matrix<double, 6, 1> twist;
	twist(0,0) = msg.twist.linear.x * _mask[0];
	twist(1,0) = msg.twist.linear.y * _mask[1];
	twist(2,0) = msg.twist.linear.z * _mask[2];
	twist(3,0) = msg.twist.angular.x * _mask[3];
	twist(4,0) = msg.twist.angular.y * _mask[4];
	twist(5,0) = msg.twist.angular.z * _mask[5];
	return twist;
}

/**
 * Converts a trackit_msgs::JointVelocityStamped message to an Eigen::MatrixXd object.
 *
 * @param msg the trackit_msgs::JointVelocityStamped message to convert
 *
 * @return the Eigen::MatrixXd object representing the joint velocity
 */
Eigen::MatrixXd IDK::jointVelocityMsgToEigen(const trackit_msgs::JointVelocityStamped msg) {
	Eigen::MatrixXd joint_velocity;

	// Need to check and make sure that the order the joint velocity is given matches the order that the jacobian is made
	// If jacobian msg have not been received yet, skip this msg
	if(!_first_jacobian_msg) {
		joint_velocity.resize(_joint_num,1);
		joint_velocity.setZero();

		for (int i = 0; i < _joint_num; i++) {
			auto index = std::find(msg.joint_names.begin(), msg.joint_names.end(), _joint_names[i]) - msg.joint_names.begin();
			auto val = msg.joint_velocity.data[index];
			joint_velocity(i,0) = val; 
		}
	}
	else {
		joint_velocity.resize(1,1);
		joint_velocity.setZero();
	}
	return joint_velocity;
}

/**
 * Calculates the allowable joint velocity step and applies a scaling factor to the input joint velocity.
 *
 * @param joint_velocity the input joint velocity matrix
 *
 * @return void
 *
 * @throws None
 */
void IDK::checkWithinLimit(Eigen::MatrixXd& joint_velocity) {
	// calculate the allowable joint velocity step
	Eigen::MatrixXd min_space = joint_velocity - _eig_joint_vel_limits.col(0);
	Eigen::MatrixXd max_space = _eig_joint_vel_limits.col(1) - joint_velocity;
	
	// initialise the scale factors
	double min_scale = 1.0;
	double max_scale = 1.0;

	for (int i = 0; i < joint_velocity.rows(); i++) {
		// if the space is negative, then the joint velocity is higher than the joint velocity limit
		// so need to calculate the scale factor
		if(min_space(i,0) < 0) {
			double temp_min_scale = _eig_joint_vel_limits(i,0) / joint_velocity(i,0);
			min_scale = std::min(min_scale, temp_min_scale);
		}
		if(max_space(i,0) < 0) {
			double temp_max_scale = _eig_joint_vel_limits(i,1) / joint_velocity(i,0);
			max_scale = std::min(max_scale, temp_max_scale);
		}
	}

	// get the minimum scale factor from both the min scale and the max scale
	double scaling_factor = std::min(min_scale, max_scale);
	
	// as a safeguard to make sure the scale doesn't accidentally become negative
	// set the scaling factor to 0 if it is negative
	if(scaling_factor < 0 ) {
		scaling_factor = 0.0;
	}

	// apply the scaling factor to the joint velocity
	joint_velocity = scaling_factor * joint_velocity;
	return;
}

int main(int argc, char **argv) {
	//Initialise IDK node
	ros::init(argc, argv, "trackit_idk");
	ros::NodeHandle n;
	IDK IDK(&n);
	ros::spin();
	return 0;
}