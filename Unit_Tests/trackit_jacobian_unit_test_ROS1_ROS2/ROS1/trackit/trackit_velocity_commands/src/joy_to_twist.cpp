/**
 * @file joy_to_twist.cpp
 * @brief a node to convert joystick messages into twist messages
 * Layout of the buttons is based on a ps4 controller running ros joy node.
 * 
 * @param max_linear_velocity the maximum linear velocity
 * @param max_angular_velocity the maximum angular velocity
 * @param twist_msg_rate the rate at which the twist messages are published
 * @param twist_frame the frame of the twist message
 *  
 * Subscribers:	
 *  - joy: the joystick message
 * 
 * Publishers:
 *  - twist: the twist message
 * 	
 */

#include "trackit_velocity_commands/joy_to_twist.h"

/**
 * Initializes a new instance of the JoyTwist class.
 *
 * @param n a pointer to the ROS NodeHandle object
 *
 * @throws None
 */
JoyTwist::JoyTwist(ros::NodeHandle* n):_n(*n) {
    // private nodehandle
    _nh = ros::NodeHandle("~");

    // node parameters
    _nh.param<double>("max_linear_velocity", _max_linear_velocity, 0.25);
    _nh.param<double>("max_angular_velocity", _max_angular_velocity, 0.785);
    _nh.param<int>("twist_msg_rate", _twist_msg_rate, 500);
    _nh.param<std::string>("twist_frame", _twist_frame, "base");

    // subscriber
    _sub_joy = _n.subscribe("joy", 1, &JoyTwist::joyCallback, this);
    
    // publisher
    _pub_twist = _n.advertise<geometry_msgs::TwistStamped>("twist",1);
    
    // timer
    _timer = _n.createTimer(ros::Duration(1.0/_twist_msg_rate), &JoyTwist::timerCallback, this);

    // publish zeros if msg received is false
    _msg_received = false;
}

/**
 * This function is the callback function for the joy topic.
 * Sets the msg_joy variable to the received message.
 * Msg_received is set to true.
 * 
 * @param msg A constant pointer to the sensor_msgs::Joy message.
 *
 * @return void
 *
 * @throws None
 */
void JoyTwist::joyCallback(const sensor_msgs::Joy::ConstPtr &msg) {
    _msg_joy = *msg;
    _msg_received = true;
}

/**
 * Timer callback function that is called periodically.
 * If msg_received is true, then the twist message is set based on the joystick input.
 * Otherwise the twist is set to zeros.
 * 
 * @param event The timer event.
 *
 * @return void
 *
 * @throws None
 */
void JoyTwist::timerCallback(const ros::TimerEvent&) {
    geometry_msgs::TwistStamped twist_msg;
    
    // initialize the twist message
    twist_msg.header.frame_id = _twist_frame;
    twist_msg.header.stamp = ros::Time::now();
    
    twist_msg.twist.linear.x = 0.0;
    twist_msg.twist.linear.y = 0.0;
    twist_msg.twist.linear.z = 0.0;

    twist_msg.twist.angular.x = 0.0;
    twist_msg.twist.angular.y = 0.0;
    twist_msg.twist.angular.z = 0.0;

    // set twist if msg_received
    if(_msg_received) {        
        _msg_received = false;
        if(_msg_joy.buttons[L1] == 1) {
            double dx = _msg_joy.axes[JOY_LEFT_HORIZONTAL] * -1; // flipping the axis value so it makes better sense
            double dy = _msg_joy.axes[JOY_LEFT_VERTICAL];
            double dz = _msg_joy.axes[JOY_RIGHT_VERTICAL];

            // changing between linear and angular velocity based on R! button press
            if(_msg_joy.buttons[R1] == 1) {
                twist_msg.twist.angular.x = dx * _max_angular_velocity;
                twist_msg.twist.angular.y = dy * _max_angular_velocity;
                twist_msg.twist.angular.z = dz * _max_angular_velocity;
            }
            else {
                twist_msg.twist.linear.x = dx * _max_linear_velocity;
                twist_msg.twist.linear.y = dy * _max_linear_velocity;
                twist_msg.twist.linear.z = dz * _max_linear_velocity;
            }
        }
    }

    _pub_twist.publish(twist_msg);
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "joy_to_twist_node");
    ros::NodeHandle n;
    JoyTwist joy_twist(&n);
    ros::spin();
    return 0;
}