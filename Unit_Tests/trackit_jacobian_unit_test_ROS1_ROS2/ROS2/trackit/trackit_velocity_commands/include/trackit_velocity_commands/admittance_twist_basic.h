#ifndef TRACKIT_VELOCITY_COMMANDS_ADMITTANCE_TWIST_BASIC_H
#define TRACKIT_VELOCITY_COMMANDS_ADMITTANCE_TWIST_BASIC_H

#include <ros/ros.h>
#include <geometry_msgs/WrenchStamped.h>
#include <geometry_msgs/TwistStamped.h>

class AdmittanceBasic {

public:
    AdmittanceBasic(ros::NodeHandle* n);
private:
    void wrenchCallback(const geometry_msgs::WrenchStamped::ConstPtr& msg);

    ros::NodeHandle _n;
    ros::NodeHandle _nh;
    ros::Subscriber _sub_wrench;
    ros::Publisher _pub_twist;

    double _Kf;
    double _Kt;
};

#endif