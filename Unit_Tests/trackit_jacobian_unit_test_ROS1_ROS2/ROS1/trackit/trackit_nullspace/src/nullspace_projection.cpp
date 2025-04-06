/**
 * @file nullspace_projection.cpp
 * @brief Implementation of the NullSpace class functions.
 * 
 * This file contains the implementation of the NullSpace class functions.
 * This node uses the jacobian to calculate a nullspace projection matrix.
 * 
 * @param lambda a singular damping value
 * @param time_diff_threshold a threshold for the time difference between messages
 * 
 * Subscribers:
 * 	- jacobian_in: The jacobian message
 * 	- joint_velocity_in: The joint velocity for the projection to be applied.
 * 
 * Publishers:
 * 	- joint_velocity_out: The projected joint velocity
 * 
 */

#include "trackit_nullspace/nullspace_projection.h"

/**
 * Constructor for the NullSpace class.
 *
 * @param n Pointer to the ROS NodeHandle object.
 *
 * @return None
 *
 * @throws None
 */
NullSpace::NullSpace(ros::NodeHandle* n):_n(*n) {
	// private nodehandle
	_nh = ros::NodeHandle("~");

	// node parameters
	_nh.param<double>("lambda", _lambda, 0.05);
	_nh.param<double>("time_diff_threshold", _time_diff_threshold, 0.01);

	// subscribers
	_sub_joint_vel = _n.subscribe("joint_velocity_in", 1, &NullSpace::jointVelocityCallback, this);
	_sub_jacobian  = _n.subscribe("jacobian_in", 1, &NullSpace::jacobianCallback, this);

	// publishers
	_pub_projected_joint_vel = _n.advertise<trackit_msgs::JointVelocityStamped>("joint_velocity_out", 1);

	_joint_num = 0;
	_first_jacobian_msg = true;

}

/**
 * Checks if the given message time is current.
 *
 * @param time The time to compare against the current time.
 *
 * @return true if the message time is current, false otherwise.
 *
 * @throws None
 */
bool NullSpace::isMsgCurrent(ros::Time time) {
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
 * Sets the member variable _msg_joint_velocity to the value of the input parameter msg.
 *
 * @param msg a constant reference to a JointVelocityStamped message
 *
 * @throws None
 */
void NullSpace::jointVelocityCallback(const trackit_msgs::JointVelocityStamped::ConstPtr& msg) {
	_msg_joint_velocity = *msg;
}

/**
 * Callback function for the Jacobian message and is the main callback for the nullspace node.
 * This calculates the nullspace projection of the input Jacobian.
 * With the joint velocity message, the nullspace projected joint velovity is calculated published.
 * 
 * @param msg The input Jacobian message
 *
 * @return void
 *
 * @throws None
 */
void NullSpace::jacobianCallback(const trackit_msgs::JacobianStamped::ConstPtr& msg) {
	trackit_msgs::JacobianStamped jacobian_msg = *msg;

	// if it is the first jacobian msg, set the number of joints and the joint names
	if(_first_jacobian_msg) {
		_first_jacobian_msg = false;
		_joint_num = jacobian_msg.joint_names.size();
		_joint_names.resize(_joint_num);
		_joint_names = jacobian_msg.joint_names;
	}

	// Check if the message is current
	if(isMsgCurrent(_msg_joint_velocity.header.stamp)) {
		// Insert the joint velocity msg into an eigen matrix for calculation
		Eigen::MatrixXd eig_joint_velocity = jointVelocityMsgToEigen(_msg_joint_velocity);
		
		// Insert the inverted jacobian data into an eigen matrix
		int r = jacobian_msg.jacobian.layout.dim[0].size;
		int c = jacobian_msg.jacobian.layout.dim[1].size;

		Eigen::MatrixXd eig_jacobian;
		eig_jacobian.resize(r,c);

		for (int i = 0; i < r; i++)	{
			for (int j = 0; j < c; j++)	{
				eig_jacobian(i,j) = jacobian_msg.jacobian.data[c*i + j];
			}
		}

		// Invert the jacobian using a pseudoInverse
		Eigen::MatrixXd eig_jacobian_pinv;
		eig_jacobian_pinv.resize(c,r);
		eig_jacobian_pinv = pseudoInverse(eig_jacobian, _lambda);

		Eigen::MatrixXd eig_identity;
		eig_identity.resize(c,c);
		eig_identity.setIdentity();

		// Calculate the projected joint velocity
		Eigen::MatrixXd eig_projected_joint_velocity = (eig_identity - eig_jacobian_pinv * eig_jacobian) * eig_joint_velocity;

		// Publish the joint velocity
		trackit_msgs::JointVelocityStamped eig_projected_joint_velocity_msg;
		eig_projected_joint_velocity_msg.header = jacobian_msg.header;
		eig_projected_joint_velocity_msg.joint_names = _joint_names;
		eig_projected_joint_velocity_msg.joint_velocity = eigenToJointVelocityMsg(eig_projected_joint_velocity);
		_pub_projected_joint_vel.publish(eig_projected_joint_velocity_msg);
	}	
}

/**
 * Converts a `trackit_msgs::JointVelocityStamped` message to an `Eigen::MatrixXd` representing joint velocities.
 *
 * @param msg The `trackit_msgs::JointVelocityStamped` message to convert.
 * 
 * @return The `Eigen::MatrixXd` representing joint velocities.
 * 
 * @throws None
 */
Eigen::MatrixXd NullSpace::jointVelocityMsgToEigen(const trackit_msgs::JointVelocityStamped msg) {
	Eigen::MatrixXd joint_velocity;
	joint_velocity.resize(_joint_num,1);
	joint_velocity.setZero();

	// Need to check and make sure that the order the joint velocity is given matches the order that the jacobian is made
	if(!_first_jacobian_msg && msg.joint_velocity.data.size() == _joint_num) {
		for (int i = 0; i < _joint_num; i++) {
			joint_velocity(i,0) = msg.joint_velocity.data[std::find(msg.joint_names.begin(), msg.joint_names.end(), _joint_names[i]) - msg.joint_names.begin()];
		}
	}

	return joint_velocity;
}


/**
 * Convert an Eigen matrix to a ROS Float64MultiArray message containing joint velocities.
 *
 * @param eig_joint_velocity The Eigen matrix representing the joint velocities.
 *
 * @return The ROS Float64MultiArray message containing the joint velocities.
 *
 * @throws None
 */
std_msgs::Float64MultiArray NullSpace::eigenToJointVelocityMsg(const Eigen::MatrixXd& eig_joint_velocity) {
	std_msgs::Float64MultiArray msg;
	msg.data.resize(_joint_num);
	for (int i = 0; i < _joint_num; i++) {
		msg.data[i] = eig_joint_velocity(i,0);
	}
	return msg;
}
/**
 * Calculates the pseudo inverse of a given matrix using the Singular Value Decomposition (SVD) method.
 *
 * @param mat The input matrix to calculate the pseudo inverse for.
 * @param lambda The threshold value for the singular values. Singular values smaller than this value will be treated as zero.
 *
 * @return The pseudo inverse of the input matrix.
 *
 * @throws None.
 */
Eigen::MatrixXd NullSpace::pseudoInverse(const Eigen::MatrixXd& mat, const double lambda) {
	Eigen::JacobiSVD<Eigen::MatrixXd> svd(mat, Eigen::DecompositionOptions::ComputeThinU|Eigen::DecompositionOptions::ComputeThinV);
	return svd.matrixV()*(svd.singularValues().array().abs() > lambda).select(svd.singularValues().array().inverse(), 0).matrix().asDiagonal() * svd.matrixU().adjoint();
}

int main(int argc, char **argv) {
	ros::init(argc, argv, "trackit_nullspace");
	ros::NodeHandle n;
	NullSpace nullspace(&n);
	ros::spin();
	return 0;
}