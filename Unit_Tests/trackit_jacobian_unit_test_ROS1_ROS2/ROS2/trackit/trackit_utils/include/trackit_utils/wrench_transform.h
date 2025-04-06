#ifndef TRACKIT_UTILS_WRENCH_TRANSFORM_H
#define TRACKIT_UTILS0_WRENCH_TRANSFORM_H

#include <ros/ros.h>
#include <geometry_msgs/WrenchStamped.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

class WrenchTransform {

public:
    WrenchTransform(ros::NodeHandle* n);

private:
    void wrenchCallback(const geometry_msgs::WrenchStamped::ConstPtr& msg);

    // vector operations
    geometry_msgs::Vector3 crossProduct(const geometry_msgs::Vector3 &vec1, const geometry_msgs::Vector3 &vec2);
    geometry_msgs::Vector3 add(const geometry_msgs::Vector3 &vec1, const geometry_msgs::Vector3 &vec2);

    // ros nodehandles
    ros::NodeHandle _n;
    ros::NodeHandle _nh;

    // ros subscriber and publisher
    ros::Subscriber _sub_wrench;
    ros::Publisher _pub_wrench;

    // tf listener
    tf2_ros::Buffer _tf_buffer;
    tf2_ros::TransformListener _tf_listen;

    //Frames
    std::string _target_frame;

    // ignore additional torque from transforming the force vector
    bool _rotate_only;
};

#endif