#include "trackit_velocity_commands/admittance_twist_mass_damper.h"

AdmittanceMassDamper::AdmittanceMassDamper(ros::NodeHandle* n):_n(*n) {
    _nh = ros::NodeHandle("~");

    _nh.param<double>("M_linear"        ,_M_linear      , 2.0);
    _nh.param<double>("M_angular"       ,_M_angular     , 2.0);
    _nh.param<double>("D_linear"        ,_D_linear      , 10.0);
    _nh.param<double>("D_angular"       ,_D_angular     , 20.0);
    _nh.param<double>("control_rate"    ,_control_rate  , 500.0);

    _sub_wrench = _n.subscribe("wrench_in", 1, &AdmittanceMassDamper::wrenchCallback, this);
    _pub_twist = _n.advertise<geometry_msgs::TwistStamped>("twist_out", 1);

    _prev_twist.header.stamp = ros::Time::now();

    _prev_twist.twist.linear.x = 0.0;
    _prev_twist.twist.linear.y = 0.0;
    _prev_twist.twist.linear.z = 0.0;

    _prev_twist.twist.angular.x = 0.0;
    _prev_twist.twist.angular.y = 0.0;
    _prev_twist.twist.angular.z = 0.0;
    
}

void AdmittanceMassDamper::wrenchCallback(const geometry_msgs::WrenchStamped::ConstPtr& msg) {
    geometry_msgs::TwistStamped twist_out;
    twist_out.header = msg->header;

    twist_out.twist.linear.x = calculateAdmittanceTwist(_M_linear, _D_linear, msg->wrench.force.x, _prev_twist.twist.linear.x);
    twist_out.twist.linear.y = calculateAdmittanceTwist(_M_linear, _D_linear, msg->wrench.force.y, _prev_twist.twist.linear.y);
    twist_out.twist.linear.z = calculateAdmittanceTwist(_M_linear, _D_linear, msg->wrench.force.z, _prev_twist.twist.linear.z);

    twist_out.twist.angular.x = calculateAdmittanceTwist(_M_angular, _D_angular, msg->wrench.torque.x, _prev_twist.twist.angular.x);
    twist_out.twist.angular.y = calculateAdmittanceTwist(_M_angular, _D_angular, msg->wrench.torque.y, _prev_twist.twist.angular.x);
    twist_out.twist.angular.z = calculateAdmittanceTwist(_M_angular, _D_angular, msg->wrench.torque.z, _prev_twist.twist.angular.x);
    
    _prev_twist = twist_out;

    // ROS_INFO_STREAM("admittance linear: "<< twist_out.twist.linear.x <<", "<< twist_out.twist.linear.y <<", "<< twist_out.twist.linear.z );
    // ROS_INFO_STREAM("admittance angular: "<< twist_out.twist.angular.x <<", "<< twist_out.twist.angular.y <<", "<< twist_out.twist.angular.z );

    _pub_twist.publish(twist_out);
}

double AdmittanceMassDamper::calculateAdmittanceTwist(double mass, double damping, double wrench, double prev_twist) {
    double dx = (wrench / _control_rate + prev_twist * mass) / (mass + damping / _control_rate);
    return dx; 
}

int main(int argc, char **argv) {
	//Initialise twist iir lpf node
	ros::init(argc, argv, "admittance_mass_damper_node");
	ros::NodeHandle n;
	AdmittanceMassDamper admittance_twist(&n);
	ros::spin();
	return 0;
}

