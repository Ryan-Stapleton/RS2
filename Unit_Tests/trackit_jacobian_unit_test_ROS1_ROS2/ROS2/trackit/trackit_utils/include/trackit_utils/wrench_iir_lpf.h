#ifndef TRACKIT_UTILS_WRENCH_IIR_LPF_H
#define TRACKIT_UTILS_WRENCH_IIR_LPF_H

#include <ros/ros.h>
#include <geometry_msgs/WrenchStamped.h>

class WrenchLPF {

public:
    WrenchLPF(ros::NodeHandle* n);
private:
    void wrenchCallback(const geometry_msgs::WrenchStamped::ConstPtr& msg);

    ros::NodeHandle _n;
    ros::NodeHandle _nh;
    ros::Publisher _pub_wrench;
    ros::Subscriber _sub_wrench;

    geometry_msgs::WrenchStamped _prev_wrench;

    double _cutoff_frequency;
};

#endif