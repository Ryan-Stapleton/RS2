/**
 * @file twist_transform.cpp
 * @brief A node to transform twist messages between frames.
 * This node calculates the transform between two frames and applies it to a twist message.
 * There is an option to remove the "moment" aspect of the transformation.
 * 
 * @param target_frame the frame to transform the twist message to.
 * @param rotate_only if true, only the rotation part of the transformation is applied.
 *  
 * Subscribers:
 * 	- twist_in: the twist message to be transformed
 * 
 * Publishers:
 * 	- twist_out: the transformed twist
 * 
 */

#include "trackit_utils/twist_transform.h"

/**
 * Constructor for the TwistTransform class.
 *
 * @param n A pointer to the NodeHandle object.
 *
 * @return None.
 *
 * @throws None.
 */
TwistTransform::TwistTransform(ros::NodeHandle *n):_n(*n), _tf_listen(_tf_buffer) {
    // private nodehandle
    _nh = ros::NodeHandle("~");

    // node parameters
    _nh.param<std::string>("target_frame", _target_frame, "target_frame");
    _nh.param<bool>("rotate_only", _rotate_only, false);
    
    // subscriber
    _sub_twist = _n.subscribe("twist_in", 1, &TwistTransform::twistCallback, this);
    
    // publisher
    _pub_twist = _n.advertise<geometry_msgs::TwistStamped>("twist_out", 1);
}

/**
 * This function is the callback for the twist message from the ROS topic. 
 * It transforms the twist message from the source frame to the target frame using the provided transform 
 * and publishes the transformed twist message to another ROS topic.
 *
 * @param msg The twist message received from the ROS topic.
 *
 * @throws tf2::TransformException If there is an error in transforming the twist message.
 */
void TwistTransform::twistCallback(const geometry_msgs::TwistStamped::ConstPtr& msg) {
    geometry_msgs::TwistStamped twist_in = *msg;
    geometry_msgs::TwistStamped twist_out = twist_in;
    geometry_msgs::TransformStamped transform_stamped;

    // try to get the transform between the two frames
    try {
        transform_stamped = _tf_buffer.lookupTransform(_target_frame, twist_in.header.frame_id, ros::Time(0), ros::Duration(1.0));
        
        geometry_msgs::Vector3 linear_in;
        geometry_msgs::Vector3 linear_out;
        geometry_msgs::Vector3 angular_in;
        geometry_msgs::Vector3 angular_out;
        
        linear_in = twist_in.twist.linear;
        angular_in = twist_in.twist.angular;

        // apply the transform
        tf2::doTransform(linear_in, linear_out, transform_stamped);
        tf2::doTransform(angular_in, angular_out, transform_stamped);

        // if rotate only is false, add the moment
        if(!_rotate_only) {
            geometry_msgs::Vector3 moment_arm;
            moment_arm = transform_stamped.transform.translation;
            angular_out = add(crossProduct(moment_arm, linear_out), angular_out);    
        }

        // publish the transformed twist
        twist_out.twist.linear = linear_out;
        twist_out.twist.angular = angular_out;
        twist_out.header.frame_id = _target_frame;
        _pub_twist.publish(twist_out);
    }
    catch(tf2::TransformException &ex) {
        ROS_ERROR_THROTTLE(0.5, "%s", ex.what());
    }
}

/**
 * Calculates the cross product of two 3D vectors.
 *
 * @param vec1 The first vector.
 * @param vec2 The second vector.
 *
 * @return The cross product of the two input vectors.
 *
 * @throws std::exception if an error occurs during the calculation.
 */
geometry_msgs::Vector3 TwistTransform::crossProduct(const geometry_msgs::Vector3 &vec1, const geometry_msgs::Vector3 &vec2) {
    geometry_msgs::Vector3 vec_out;
    
    try {
        // calculate the cross product
        vec_out.x = vec1.y*vec2.z - vec1.z*vec2.y;
        vec_out.y = vec1.z*vec2.x - vec1.x*vec2.z;
        vec_out.z = vec1.x*vec2.y - vec1.y*vec2.x;
    }
    catch(std::exception& ex) {
        ROS_WARN("Exception detected: %s", ex.what());
    }

    return vec_out;
}

/**
 * Adds two `geometry_msgs::Vector3` objects and returns the result.
 *
 * @param vec1 The first `geometry_msgs::Vector3` object to be added.
 * @param vec2 The second `geometry_msgs::Vector3` object to be added.
 *
 * @return The sum of `vec1` and `vec2` as a `geometry_msgs::Vector3` object.
 *
 * @throws std::exception If an exception occurs during the addition process.
 */
geometry_msgs::Vector3 TwistTransform::add(const geometry_msgs::Vector3 &vec1, const geometry_msgs::Vector3 &vec2) {
    geometry_msgs::Vector3 vec_out;
    
    try {
        // calculate the sum
        vec_out.x = vec1.x + vec2.x;
        vec_out.y = vec1.y + vec2.y;
        vec_out.z = vec1.z + vec2.z;
    }
    catch(std::exception& ex) {
        ROS_WARN("Exception detected: %s", ex.what());
    }
    
    return vec_out;
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "twist_transform");
    ros::NodeHandle n;
    TwistTransform twist_transform(&n);
    ros::spin();
    return 0;
}