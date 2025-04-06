#ifndef TRACKIT_VELOCITY_COMMANDS_ADMITTANCE_TWIST_SPRING_MASS_DAMPER_H
#define TRACKIT_VELOCITY_COMMANDS_ADMITTANCE_TWIST_SPRING_MASS_DAMPER_H

#include <ros/ros.h>
#include <geometry_msgs/WrenchStamped.h>
#include <geometry_msgs/TwistStamped.h>

class AdmittanceSpringMassDamper {

public:
    AdmittanceSpringMassDamper(ros::NodeHandle* n);
private:
    void wrenchCallback(const geometry_msgs::WrenchStamped::ConstPtr& msg);
    void springTwistCallback(const geometry_msgs::TwistStamped::ConstPtr& msg);
    double calculateAdmittanceTwist(double K, double M, double D, double F, double V_prev, double V_target);

    ros::NodeHandle _n;
    ros::NodeHandle _nh;
    ros::Subscriber _sub_wrench;
    ros::Subscriber _sub_spring_twist;
    ros::Publisher _pub_twist;

    std::vector<double> _K;
    std::vector<double> _M;
    std::vector<double> _C;
    
    double _control_rate;
    
    geometry_msgs::TwistStamped _twist_prev;
    geometry_msgs::TwistStamped _twist_spring;
};

#endif