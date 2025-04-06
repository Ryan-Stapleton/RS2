/**
 * @file admittance_twist_basic.cpp
 * @brief A node for a basic admittance control.
 * This node converts a wrench into a twist using the admittance gains.
 * 
 * @param Kf the gain for the force part of the wrench
 * @param Kt the gain for the torque part of the wrench
 *  
 * Subscribers:
 * 	- wrench_in: the wrench message to be converted
 * 
 * Publishers:
 * 	- twist_out: the converted twist
 * 
 */

#include "trackit_velocity_commands/admittance_twist_basic.h"

/**
 * Constructor for the AdmittanceBasic class.
 *
 * @param n A pointer to the ROS NodeHandle object.
 *
 * @throws None
 */
AdmittanceBasic::AdmittanceBasic(ros::NodeHandle* n):_n(*n) {
    _nh = ros::NodeHandle("~");

    _nh.param<double>("Kf", _Kf, 0.0);
    _nh.param<double>("Kt", _Kt, 0.0);
    
    _sub_wrench = _n.subscribe("wrench_in", 1, &AdmittanceBasic::wrenchCallback, this);
    _pub_twist = _n.advertise<geometry_msgs::TwistStamped>("twist_out", 1);

}

/**
 * Callback function for receiving wrench messages.
 * This callback converts the wrench msg into a twist using the admittance gains.
 * 
 * @param msg Pointer to the received wrench message.
 *
 * @return void
 *
 * @throws None
 */
void AdmittanceBasic::wrenchCallback(const geometry_msgs::WrenchStamped::ConstPtr& msg) {
    geometry_msgs::TwistStamped twist_out;
    twist_out.header = msg->header;

    // calculate the output twist
    twist_out.twist.linear.x = _Kf * msg->wrench.force.x;
    twist_out.twist.linear.y = _Kf * msg->wrench.force.y;
    twist_out.twist.linear.z = _Kf * msg->wrench.force.z;

    twist_out.twist.angular.x = _Kt * msg->wrench.torque.x;
    twist_out.twist.angular.y = _Kt * msg->wrench.torque.y;
    twist_out.twist.angular.z = _Kt * msg->wrench.torque.z;
    
    // publish the output twist
    _pub_twist.publish(twist_out);
}

int main(int argc, char **argv) {
	//Initialise twist iir lpf node
	ros::init(argc, argv, "admittance_basic_node");
	ros::NodeHandle n;
	AdmittanceBasic admittance_basic(&n);
	ros::spin();
	return 0;
}