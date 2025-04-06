#ifndef TRACKIT_UTILS_TWIST_TRANSFORM_H
#define TRACKIT_UTILS_TWIST_TRANSFORM_H

#include <ros/ros.h>
#include <geometry_msgs/TwistStamped.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

class TwistTransform {

public:
    TwistTransform(ros::NodeHandle* n);

private:
    void twistCallback(const geometry_msgs::TwistStamped::ConstPtr& msg);
    
    // vector oprations
    geometry_msgs::Vector3 crossProduct(const geometry_msgs::Vector3 &vec1, const geometry_msgs::Vector3 &vec2);
    geometry_msgs::Vector3 add(const geometry_msgs::Vector3 &vec1, const geometry_msgs::Vector3 &vec2);

    // ros nodehandles
    ros::NodeHandle _n;
    ros::NodeHandle _nh;

    // ros subscriber and publisher
    ros::Subscriber _sub_twist;
    ros::Publisher _pub_twist;

    // TF Listener
    tf2_ros::Buffer _tf_buffer;
    tf2_ros::TransformListener _tf_listen;

    // Frames
    std::string _target_frame;
    
    // Ignore additional angular velocity from the linear velocity part
    bool _rotate_only;
};

#endif