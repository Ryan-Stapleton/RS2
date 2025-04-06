/**
 * @file admittance_twist_spring_mass_damper.cpp
 * @brief An admittance control node simulating spring mass damper dynamics.
 * This node converts a wrench into a twist using the apring mass damper gains.
 * 
 * @param K a six element vector of gains for the spring component
 * @param M a six element vector of gains for the mass component
 * @param C a six element vector of gains for the damper component
 * @param control_rate the control rate in Hz
 *  
 * Subscribers:	
 * - wrench_in: the wrench message to be converted
 * - twist_spring: the twist message for the spring
 * 
 * Publishers:
 * - twist_out: the converted twist
 */

#include "trackit_velocity_commands/admittance_twist_spring_mass_damper.h"

/**
 * Constructor for the AdmittanceSpringMassDamper class.
 *
 * @param n Pointer to the ROS NodeHandle object.
 *
 * @throws None
 */
AdmittanceSpringMassDamper::AdmittanceSpringMassDamper(ros::NodeHandle* n):_n(*n) {
    // private nodehandle
    _nh = ros::NodeHandle("~");

    // node parameters
    _nh.param<std::vector<double>>("K", _K, {0.0, 0.0, 0.0, 0.0, 0.0, 0.0});
    _nh.param<std::vector<double>>("M", _M, {0.0, 0.0, 0.0, 0.0, 0.0, 0.0});
    _nh.param<std::vector<double>>("C", _C, {0.0, 0.0, 0.0, 0.0, 0.0, 0.0});

    _nh.param<double>("control_rate"    ,_control_rate  , 500.0);

    // subscribers
    _sub_wrench = _n.subscribe("wrench_in", 1, &AdmittanceSpringMassDamper::wrenchCallback, this);
    _sub_spring_twist = _n.subscribe("twist_spring", 1, &AdmittanceSpringMassDamper::springTwistCallback, this);
    
    // publisher
    _pub_twist = _n.advertise<geometry_msgs::TwistStamped>("twist_out", 1);

    // initialize twist
    _twist_prev.header.stamp = ros::Time::now();

    _twist_prev.twist.linear.x = 0.0;
    _twist_prev.twist.linear.y = 0.0;
    _twist_prev.twist.linear.z = 0.0;

    _twist_prev.twist.angular.x = 0.0;
    _twist_prev.twist.angular.y = 0.0;
    _twist_prev.twist.angular.z = 0.0;
    
    _twist_spring.header.stamp = ros::Time::now();

    _twist_spring.twist.linear.x = 0.0;
    _twist_spring.twist.linear.y = 0.0;
    _twist_spring.twist.linear.z = 0.0;

    _twist_spring.twist.angular.x = 0.0;
    _twist_spring.twist.angular.y = 0.0;
    _twist_spring.twist.angular.z = 0.0;

    // check parameters
    if(_K.size() !=6) {
        ROS_ERROR("K size must be 6");
    }
    if(_M.size() !=6) {
        ROS_ERROR("M size must be 6");
    }
    if(_C.size() !=6) {
        ROS_ERROR("D size must be 6");
    }
}

/**
 * Callback function that handles the wrench message from the AdmittanceSpringMassDamper class.
 *
 * @param msg The wrench message received.
 *
 * @return void
 *
 * @throws None
 */
void AdmittanceSpringMassDamper::wrenchCallback(const geometry_msgs::WrenchStamped::ConstPtr& msg) {
    geometry_msgs::TwistStamped twist_out;

    twist_out.header = msg->header;
    
    // calculate the output twist
    twist_out.twist.linear.x = calculateAdmittanceTwist(_K[0], _M[0], _C[0], msg->wrench.force.x, _twist_prev.twist.linear.x, _twist_spring.twist.linear.x);
    twist_out.twist.linear.y = calculateAdmittanceTwist(_K[1], _M[1], _C[1], msg->wrench.force.y, _twist_prev.twist.linear.y, _twist_spring.twist.linear.y);
    twist_out.twist.linear.z = calculateAdmittanceTwist(_K[2], _M[2], _C[2], msg->wrench.force.z, _twist_prev.twist.linear.z, _twist_spring.twist.linear.z);

    twist_out.twist.angular.x = calculateAdmittanceTwist(_K[3], _M[3], _C[3], msg->wrench.torque.x, _twist_prev.twist.angular.x, _twist_spring.twist.angular.x);
    twist_out.twist.angular.y = calculateAdmittanceTwist(_K[4], _M[4], _C[4], msg->wrench.torque.y, _twist_prev.twist.angular.y, _twist_spring.twist.angular.y);
    twist_out.twist.angular.z = calculateAdmittanceTwist(_K[5], _M[5], _C[5], msg->wrench.torque.z, _twist_prev.twist.angular.z, _twist_spring.twist.angular.z);
    
    // update the previous twist
    _twist_prev = twist_out;

    // publish the twist
    _pub_twist.publish(twist_out);
}

/**
 * Updates the spring twist data based on the received message.
 *
 * @param msg A pointer to a constant TwistStamped message containing the new spring twist data.
 *
 * @throws None
 */
void AdmittanceSpringMassDamper::springTwistCallback(const geometry_msgs::TwistStamped::ConstPtr& msg) {
    _twist_spring = *msg;
}

/**
 * Calculates the admittance twist for the given parameters.
 *
 * @param K the spring constant
 * @param M the mass
 * @param D the damping coefficient
 * @param F the force
 * @param V_prev the previous velocity
 * @param V_target the target velocity
 *
 * @return the calculated admittance twist
 *
 * @throws None
 */
double AdmittanceSpringMassDamper::calculateAdmittanceTwist(double K, double M, double D, double F, double V_prev, double V_target) {
    double V = (F/_control_rate + M*V_prev )/(M + D/_control_rate);
    double K_t = -(K*V_target/_control_rate)/(M + D/_control_rate);
    return V - K_t; 
}

int main(int argc, char **argv) {
	//Initialise twist iir lpf node
	ros::init(argc, argv, "admittance_mass_damper_node");
	ros::NodeHandle n;
	AdmittanceSpringMassDamper admittance_twist(&n);
	ros::spin();
	return 0;
}

