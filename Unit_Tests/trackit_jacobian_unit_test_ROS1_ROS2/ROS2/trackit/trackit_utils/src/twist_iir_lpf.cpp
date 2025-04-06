/**
 * @file twist_iir_lpf.cpp
 * @brief This node applies an IIR low pass filter to the twist messages.
 * 
 * A low pass filter for twist messages. 
 * Implements the low pass filter from https://en.wikipedia.org/wiki/Low-pass_filter
 * 
 * @param  cutoff_frequency the cutoff frequency of the filter
 *  
 * Subscribers:
 * 	- twist_in: the twist message to be filtered
 * 
 * Publishers:
 * 	- twist_out: the filtered twist
 * 
 */

#include "trackit_utils/twist_iir_lpf.h"


/**
 * Constructor for the TwistLPF class.
 *
 * @param n Pointer to the ROS NodeHandle object.
 *
 * @throws None
 */
TwistLPF::TwistLPF(ros::NodeHandle* n):_n(*n) {

    // nodehandle
    _nh = ros::NodeHandle("~");

    // node parameters
    _nh.param<double>("cutoff_frequency", _cutoff_frequency, 1.0);
    
    // subscribers
    _sub_twist = _n.subscribe("twist_in", 1, &TwistLPF::twistCallback, this);
    
    // publishers
    _pub_twist = _n.advertise<geometry_msgs::TwistStamped>("twist_out", 1);

    // initialize previous twist
    _prev_twist.twist.linear.x = 0.0;
    _prev_twist.twist.linear.y = 0.0;
    _prev_twist.twist.linear.z = 0.0;
    _prev_twist.twist.angular.x = 0.0;
    _prev_twist.twist.angular.y = 0.0;
    _prev_twist.twist.angular.z = 0.0;

    _prev_twist.header.stamp = ros::Time::now();
}

/**
 * Callback function for twist messages.
 * Filtering is performed here.
 * 
 * @param msg Pointer to the received twist message.
 *
 * @throws None
 */
void TwistLPF::twistCallback(const geometry_msgs::TwistStamped::ConstPtr& msg) {
    geometry_msgs::TwistStamped twist_in = *msg;
    geometry_msgs::TwistStamped twist_out = twist_in;

    // calculate time difference and alpha
    double dt = (twist_out.header.stamp - _prev_twist.header.stamp).toSec();
    double alpha = (2*M_PI*_cutoff_frequency*dt)/((2*M_PI*_cutoff_frequency*dt + 1));

    // low pass filter
    twist_out.twist.linear.x = alpha * twist_in.twist.linear.x  + (1-alpha) * _prev_twist.twist.linear.x;
    twist_out.twist.linear.y = alpha * twist_in.twist.linear.y  + (1-alpha) * _prev_twist.twist.linear.y;
    twist_out.twist.linear.z = alpha * twist_in.twist.linear.z  + (1-alpha) * _prev_twist.twist.linear.z;

    twist_out.twist.angular.x = alpha * twist_in.twist.angular.x  + (1-alpha) * _prev_twist.twist.angular.x;
    twist_out.twist.angular.y = alpha * twist_in.twist.angular.y  + (1-alpha) * _prev_twist.twist.angular.y;
    twist_out.twist.angular.z = alpha * twist_in.twist.angular.z  + (1-alpha) * _prev_twist.twist.angular.z;

    _prev_twist = twist_out;
    
    // publish filtered twist
    _pub_twist.publish(twist_out);
}

int main(int argc, char **argv) {
	ros::init(argc, argv, "twist_lpf_node");
	ros::NodeHandle n;
	TwistLPF twist_lpf(&n);
	ros::spin();
	return 0;
}