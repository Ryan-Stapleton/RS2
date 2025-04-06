#ifndef TRACKIT_VELOCITY_COMMANDS_ADMITTANCE_TWIST_MASS_DAMPER_H
#define TRACKIT_VELOCITY_COMMANDS_ADMITTANCE_TWIST_MASS_DAMPER_H

#include <ros/ros.h>
#include <geometry_msgs/WrenchStamped.h>
#include <geometry_msgs/TwistStamped.h>

class AdmittanceMassDamper {

public:
    AdmittanceMassDamper(ros::NodeHandle* n);
private:
    void wrenchCallback(const geometry_msgs::WrenchStamped::ConstPtr& msg);
    double calculateAdmittanceTwist(double mass, double damping, double wrench, double prev_twist);

    ros::NodeHandle _n;
    ros::NodeHandle _nh;
    ros::Subscriber _sub_wrench;
    ros::Publisher _pub_twist;

    double _M_linear;
    double _M_angular;
    double _D_linear;
    double _D_angular;
    double _control_rate;

    geometry_msgs::TwistStamped _prev_twist;
};

#endif