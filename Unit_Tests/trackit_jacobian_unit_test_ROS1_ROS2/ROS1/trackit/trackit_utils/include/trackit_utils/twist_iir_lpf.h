#ifndef TRACKIT_UTILS_TWIST_IIR_LPF_H
#define TRACKIT_UTILS_TWIST_IIR_LPF_H

#include <ros/ros.h>
#include <geometry_msgs/TwistStamped.h>

class TwistLPF {

public:
    TwistLPF(ros::NodeHandle* n);
private:
    void twistCallback(const geometry_msgs::TwistStamped::ConstPtr& msg);

    ros::NodeHandle _n;
    ros::NodeHandle _nh;
    ros::Publisher _pub_twist;
    ros::Subscriber _sub_twist;

    geometry_msgs::TwistStamped _prev_twist;

    double _cutoff_frequency;
};

#endif