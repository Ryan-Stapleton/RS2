#ifndef TRACKIT_VELOCITY_COMMANDS_JOY_TO_TWIST_H
#define TRACKIT_VELOCITY_COMMANDS_JOY_TO_TWIST_H

#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <geometry_msgs/TwistStamped.h>

// PS4 controller button mapping
// #define BUTTON_X 0
// #define BUTTON_CIRCLE 1
// #define BUTTON_TRIANGLE 2
// #define BUTTON_SQUARE 3
// #define BUTTON_L1 4
// #define BUTTON_R1 5
// #define BUTTON_L2 6
// #define BUTTON_R2 7
// #define BUTTON_SHARE 8
// #define BUTTON_OPTIONS 9
// #define BUTTON_PS_LOGO 10
// #define BUTTON_JOY_L3 11
// #define BUTTON_JOY_R3 12

enum button {
    X = 0,
    CIRCLE = 1,
    TRIANGLE = 2,
    SQUARE = 3,
    L1 = 4,
    R1 = 5,
    L2 = 6,
    R2 = 7,
    SHARE = 8,
    OPTIONS = 9,
    PS_LOGO = 10,
    JOY_L3 = 11,
    JOY_R3 = 12
};

// PS4 controller axes mapping (default left and up is +1, right and down is -1)
// #define AXIS_JOY_LEFT_HORIZONTAL 0
// #define AXIS_JOY_LEFT_VERTICAL 1
// #define AXIS_L2 2
// #define AXIS_JOY_RIGHT_HORIZONTAL 3
// #define AXIS_JOY_RIGHT_VERTICAL 4
// #define AXIS_R2 5
// #define AXIS_PAD_HORIZONTAL 6
// #define AXIS_PAD_VERTICAL 7

enum axis {
    JOY_LEFT_HORIZONTAL = 0,
    JOY_LEFT_VERTICAL = 1,
    TRIGGER_L2 = 2,
    JOY_RIGHT_HORIZONTAL = 3,
    JOY_RIGHT_VERTICAL = 4,
    TRIGGER_R2 = 5,
    PAD_HORIZONTAL = 6,
    PAD_VERTICAL = 7
};

class JoyTwist {
public:
    JoyTwist(ros::NodeHandle* n);

private:
    void joyCallback(const sensor_msgs::Joy::ConstPtr& msg);
    void timerCallback(const ros::TimerEvent&);

    ros::NodeHandle _n;
    ros::NodeHandle _nh;

    ros::Subscriber _sub_joy;
    ros::Publisher _pub_twist;
    ros::Timer _timer;

    sensor_msgs::Joy _msg_joy;

    double _max_linear_velocity;
    double _max_angular_velocity;
    
    int _twist_msg_rate;

    std::string _twist_frame;

    bool _msg_received;
};
#endif