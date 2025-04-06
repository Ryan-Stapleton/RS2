/**
 * @file wrench_transform.cpp
 * @brief A node to transform wrench messages between frames.
 * This node calculates the transform between two frames and applies it to a wrench message.
 * There is an option to remove the "moment" aspect of the transformation.
 * 
 * @param target_frame the frame to transform the wrench message to.
 * @param rotate_only if true, only the rotation part of the transformation is applied.
 *  
 * Subscribers:
 * 	- wrench_in: the wrench message to be transformed
 * 
 * Publishers:
 * 	- wrench_out: the transformed wrench
 * 
 */

#include "trackit_utils/wrench_transform.h"

/**
 * Constructor for the WrenchTransform class.
 *
 * @param n Pointer to the ROS NodeHandle object.
 *
 * @throws None
 */
WrenchTransform::WrenchTransform(ros::NodeHandle* n):_n(*n), _tf_listen(_tf_buffer) {
    // private nodehandle
    _nh = ros::NodeHandle("~");

    // node parameters
    _nh.param<std::string>("target_frame", _target_frame, "target_frame");
    _nh.param<bool>("rotate_only", _rotate_only, false);

    // subscriber
    _sub_wrench = _n.subscribe("wrench_in", 1, &WrenchTransform::wrenchCallback, this);
    
    // publisher
    _pub_wrench = _n.advertise<geometry_msgs::WrenchStamped>("wrench_out", 1);
}

/**
 * Callback function for the wrench message.
 *
 * @param msg The wrench message received.
 *
 * @throws tf2::TransformException if there is an error in transforming the wrench.
 */
void WrenchTransform::wrenchCallback(const geometry_msgs::WrenchStamped::ConstPtr& msg) {
    geometry_msgs::WrenchStamped wrench_in = *msg;
    geometry_msgs::WrenchStamped wrench_out = wrench_in;
    geometry_msgs::TransformStamped transform_stamped;

    // try to get the transform between the two frames
    try {
        transform_stamped = _tf_buffer.lookupTransform(_target_frame, wrench_in.header.frame_id, ros::Time(0), ros::Duration(1.0));

        geometry_msgs::Vector3 force_in;
        geometry_msgs::Vector3 force_out;
        geometry_msgs::Vector3 torque_in;
        geometry_msgs::Vector3 torque_out;

        force_in = wrench_in.wrench.force;
        torque_in = wrench_in.wrench.torque;

        // transform the wrench
        tf2::doTransform(force_in, force_out, transform_stamped);
        tf2::doTransform(torque_in, torque_out, transform_stamped);
        
        // if rotate only is false, add the moment
        if(!_rotate_only) {
            geometry_msgs::Vector3 moment_arm;
            moment_arm = transform_stamped.transform.translation;
            torque_out = add(crossProduct(moment_arm, force_out), torque_out);
        }

        // publish the transformed wrench
        wrench_out.wrench.force = force_out;
        wrench_out.wrench.torque = torque_out;
        wrench_out.header.frame_id = _target_frame;
        _pub_wrench.publish(wrench_out);
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
 * @return The resulting vector.
 *
 * @throws std::exception Exception thrown if an error occurs during the calculation.
 */
geometry_msgs::Vector3 WrenchTransform::crossProduct(const geometry_msgs::Vector3 &vec1, const geometry_msgs::Vector3 &vec2) {
    geometry_msgs::Vector3 vec_out;
    
    try {
        // calculate the cross product
        vec_out.x = vec1.y*vec2.z - vec1.z*vec2.y;
        vec_out.y = vec1.z*vec2.x - vec1.x*vec2.z;
        vec_out.z = vec1.x*vec2.y - vec1.y*vec2.x;
    }
    catch(std::exception& ex) {
        ROS_WARN("crossProduct Exception detected: %s", ex.what());
    }

    return vec_out;
}

/**
 * Adds two geometry_msgs::Vector3 objects.
 *
 * @param vec1 The first geometry_msgs::Vector3 object to be added.
 * @param vec2 The second geometry_msgs::Vector3 object to be added.
 *
 * @return A geometry_msgs::Vector3 object representing the sum of vec1 and vec2.
 *
 * @throws std::exception If an exception occurs during the addition.
 */
geometry_msgs::Vector3 WrenchTransform::add(const geometry_msgs::Vector3 &vec1, const geometry_msgs::Vector3 &vec2) {
    geometry_msgs::Vector3 vec_out;
    
    try {
        // add the vectors
        vec_out.x = vec1.x + vec2.x;
        vec_out.y = vec1.y + vec2.y;
        vec_out.z = vec1.z + vec2.z;
    }
    catch(std::exception& ex) {
        ROS_WARN("add Exception detected: %s", ex.what());
    }
    
    return vec_out;
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "wrench_transform");
    ros::NodeHandle n;
    WrenchTransform wrench_transform(&n);
    ros::spin();
    return 0;
}