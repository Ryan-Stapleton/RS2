/**
 * @file task_joint_middle.cpp
 * @brief Implementation of the JointMiddle class functions.
 * 
 * This file contains the implementation of the JointMiddle class functions.
 * This node calculates a joint velocity task to move the robot towards the middle of the joint limits
 * 
 * @param max_velocity the maximum velocity for the joint velocity task
 * 
 * Subscribers:
 * 	- joint_limits: The joint limits of the robot.
 * 	- joint_states: The current joint state of the robot.
 * 
 * Publishers:
 * 	- nullspace_joint_velocity: The calculated joint velocity to be sent to the nullspace projection.
 * 
 */

 #include "trackit_nullspace/task_joint_middle.h"	

 /**
  * Constructor for the JointMiddle class.
  *
  * @param n A pointer to a `ros::NodeHandle` object.
  *
  * @throws None
  */
 JointMiddle::JointMiddle() : Node("JointMiddle") {
	 // private nodehandle
	 // _nh = ros::NodeHandle("~");
	 
	 // node parameters
	 _nh->declare_parameter("max_velocity", 0.1);
	 _nh->get_parameter("max_velocity", _max_velocity);
 
	 // publishers
	 // _pub_joint_vel = _n.advertise<trackit_msgs::JointVelocityStamped>("nullspace_joint_velocity", 1);
	 _pub_joint_vel = this->create_publisher<trackit_msgs::msg::JointVelocityStamped>("nullspace_joint_velocity", 1);
 
	 // subscribers
	 // _sub_joint_limits = _n.subscribe("joint_limits", 1, &JointMiddoile::jointLimitsCallback, this);
	 _sub_joint_limits = this->create_subscription<trackit_msgs::msg::JointLimitsStamped>(
		 "joint_limits", 1, std::bind(&JointMiddle::jointLimitsCallback,this,std::placeholders::_1));
	 // _sub_joint_states = _n.subscribe("joint_states", 1, &JointMiddle::jntStateCallback, this);
	 _sub_joint_states = this->create_subscription<sensor_msgs::msg::JointState>(
		 "joint_states", 1, std::bind(&JointMiddle::jointStateCallback,this,std::placeholders::_1));
	 
	 // initialize joint names
	 _joint_names.resize(0);
	 _joint_names.clear();
 
 }
 
 /**
  * Main callback function for the joint state.
  * This callback calculates the joint velocity from the latest joint limits .
  * 
  * @param msg Pointer to the joint state message.
  *
  * @throws None
  */
 void JointMiddle::jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg) {
 
	 if(_joint_names.size() > 0) {
 
		 Eigen::MatrixXd current_joint_position;
		 current_joint_position.resize(_joint_names.size(),1);
 
		 for (int i = 0; i < _joint_names.size(); i++) {
			 current_joint_position(i,0) = msg->position[std::find(msg->name.begin(), msg->name.end(), _joint_names[i]) - msg->name.begin()];	
		 }
 
		 Eigen::MatrixXd difference = _midpoint - current_joint_position;
		 Eigen::MatrixXd magnitude = difference/_max_velocity;
		 magnitude = magnitude.cwiseAbs();
 
		 if(magnitude.maxCoeff() > 1) {
			 difference = difference/magnitude.maxCoeff();
		 }
 
		 trackit_msgs::msg::JointVelocityStamped msg_joint_velocity;
		 msg_joint_velocity.header = msg->header;
		 msg_joint_velocity.joint_names = _joint_names;
 
		 msg_joint_velocity.joint_velocity.data.resize(_joint_names.size());
		 for (int i = 0; i < _joint_names.size(); i++) {
			 msg_joint_velocity.joint_velocity.data[i] = difference(i,0);
		 }
		 
		 _pub_joint_vel->publish(msg_joint_velocity);
	 }
	 
 }
 
 /**
  * Callback function for joint limits.
  *
  * @param msg Pointer to the joint limits message
  *
  * @return void
  *
  * @throws None
  */
 void JointMiddle::jointLimitsCallback(const trackit_msgs::msg::JointLimitsStamped::SharedPtr msg) {
 
	 _joint_names = msg->joint_names;
	 msgToMatrix(msg->position_upper_limits, _joint_upper_limits);
	 msgToMatrix(msg->position_lower_limits, _joint_lower_limits);
 
	 _midpoint.resize(_joint_names.size(),1);
	 _midpoint.setZero();
 
	 _midpoint = (_joint_upper_limits + _joint_lower_limits)/2;
 }
 
 /**
  * Insert the values from the Float64MultiArray message into the Eigen matrix
  *
  * @param msg The input Float64MultiArray message
  * @param mat The Eigen matrix to be filled
  *
  * @throws None
  */
 void JointMiddle::msgToMatrix(const std_msgs::msg::Float64MultiArray& msg, Eigen::MatrixXd& mat) {
	 mat.resize(msg.data.size(),1);
	 mat.setZero();
	 for (int i = 0; i < msg.data.size(); i++) {
		 mat(i,0) = msg.data[i];
	 }
 }
 
 int main(int argc, char** argv) {
	 rclcpp::init(argc, argv);
	 auto node = std::make_shared<JointMiddle>();
	 // JointMiddle task_joint_middle(&n);
	 // rclcpp::executors::SingleThreadedExecutor executor;
	 rclcpp::spin(node);
	 return 0;
 }