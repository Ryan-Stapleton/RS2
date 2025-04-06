/**
 * @file robot_command.cpp
 * @brief A generic node to publish joint velocity commands to the robot.
 * This node is a generic node
 * 
 * @param use_deadman if true the message only publish if a true boolean msg is received
 *  
 * Subscribers:	
 *  - joint_velocity: the trackit joint velocity message to be sent to the robot
 * 
 * Publishers:
 *  - velocity_command: the joint velocity command to be sent to the robot
 * 
 */

#include "trackit_velocity_commands/robot_command.h"

/**
 * Constructor for the RobotCommand class.
 *
 * @param n pointer to the ROS NodeHandle
 *
 * @throws None
 */
RobotCommand::RobotCommand(ros::NodeHandle* n):_n(*n) {
    // private nodehandle
    _nh = ros::NodeHandle("~");

    // node parameters
    _nh.param<bool>("use_deadman", _use_deadman, false);

	_sub_trackit_msg = _n.subscribe("joint_velocity", 1, &RobotCommand::trackit_msg_callback, this);

    if(_use_deadman) {
        _sub_deadman_msg = _n.subscribe("/deadman", 1, &RobotCommand::deadman_callback, this);
    }
	_pub_joint_command = _n.advertise<std_msgs::Float64MultiArray>("velocity_command", 1);
    
}

/**
 * Callback function for the deadman switch.
 *
 * @param msg Pointer to the ROS message containing the deadman switch state.
 *
 * @throws None
 */
void RobotCommand::deadman_callback(const std_msgs::Bool::ConstPtr& msg) {
    _enable = msg->data;
}

void RobotCommand::trackit_msg_callback(const trackit_msgs::JointVelocityStamped::ConstPtr& msg) {
    trackit_msgs::JointVelocityStamped ti_msg = *msg;
    std_msgs::Float64MultiArray cmd;

    for (int i = 0; i < 6; i++) {
        cmd.data.push_back(ti_msg.joint_velocity.data[i]);
        if(_use_deadman == true && _enable == false) {
            cmd.data[i] = 0;
        }    
    }
    
    _pub_joint_command.publish(cmd);
}

int main(int argc, char **argv) {
	//Initialise UR velocity command node
	ros::init(argc, argv, "velocity_controller_command_node");
	ros::NodeHandle n;
	RobotCommand ur_command(&n);
	ros::spin();
	return 0;
}