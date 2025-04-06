/**
 * @file wrench_iir_lpf.cpp
 * @brief This node applies an IIR low pass filter to the wrench messages.
 * 
 *  A low pass filter for wrench messages. 
 * Implements the low pass filter from https://en.wikipedia.org/wiki/Low-pass_filter
 * 
 * @param  cutoff_frequency the cutoff frequency of the filter
 *  
 * Subscribers:
 * 	- wrench_in: the wrench message to be filtered
 * 
 * Publishers:
 * 	- wrench_out: the filtered wrench
 * 
 */

#include "trackit_utils/wrench_iir_lpf.h"


/**
 * Constructor for the WrenchLPF class.
 *
 * @param n Pointer to the ROS node handle.
 *
 * @throws None
 */
WrenchLPF::WrenchLPF(ros::NodeHandle* n):_n(*n) {
    // private nodehandle
    _nh = ros::NodeHandle("~");

    // node parameters
    _nh.param<double>("cutoff_frequency", _cutoff_frequency, 1.0);
    
    // subscriber
    _sub_wrench = _n.subscribe("wrench_in", 1, &WrenchLPF::wrenchCallback, this);
    
    // publisher
    _pub_wrench = _n.advertise<geometry_msgs::WrenchStamped>("wrench_out", 1);

    // initialize previous wrench
    _prev_wrench.wrench.force.x = 0.0;
    _prev_wrench.wrench.force.y = 0.0;
    _prev_wrench.wrench.force.z = 0.0;
    _prev_wrench.wrench.torque.x = 0.0;
    _prev_wrench.wrench.torque.y = 0.0;
    _prev_wrench.wrench.torque.z = 0.0;

    _prev_wrench.header.stamp = ros::Time::now();
}

/**
 * Updates the wrench value and publishes it.
 *
 * @param msg A pointer to the wrench message.
 *
 * @return None.
 *
 * @throws None.
 */
void WrenchLPF::wrenchCallback(const geometry_msgs::WrenchStamped::ConstPtr& msg) {
    geometry_msgs::WrenchStamped wrench_in = *msg;
    geometry_msgs::WrenchStamped wrench_out = wrench_in;

    // calculate time difference and alpha
    double dt = (wrench_out.header.stamp - _prev_wrench.header.stamp).toSec();
    double alpha = (2*M_PI*_cutoff_frequency*dt)/((2*M_PI*_cutoff_frequency*dt + 1));

    // low pass filter
    wrench_out.wrench.force.x = alpha * wrench_in.wrench.force.x  + (1-alpha) * _prev_wrench.wrench.force.x;
    wrench_out.wrench.force.y = alpha * wrench_in.wrench.force.y  + (1-alpha) * _prev_wrench.wrench.force.y;
    wrench_out.wrench.force.z = alpha * wrench_in.wrench.force.z  + (1-alpha) * _prev_wrench.wrench.force.z;

    wrench_out.wrench.torque.x = alpha * wrench_in.wrench.torque.x  + (1-alpha) * _prev_wrench.wrench.torque.x;
    wrench_out.wrench.torque.y = alpha * wrench_in.wrench.torque.y  + (1-alpha) * _prev_wrench.wrench.torque.y;
    wrench_out.wrench.torque.z = alpha * wrench_in.wrench.torque.z  + (1-alpha) * _prev_wrench.wrench.torque.z;

    // update previous wrench
    _prev_wrench = wrench_out;
    
    // publish wrench
    _pub_wrench.publish(wrench_out);
}

int main(int argc, char **argv) {
	//Initialise twist iir lpf node
	ros::init(argc, argv, "wrench_lpf_node");
	ros::NodeHandle n;
	WrenchLPF wrench_lpf(&n);
	ros::spin();
	return 0;
}