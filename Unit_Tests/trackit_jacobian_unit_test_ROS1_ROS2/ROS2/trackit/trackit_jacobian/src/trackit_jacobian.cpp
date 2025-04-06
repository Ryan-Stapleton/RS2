/**
 * @file trackit_jacobian.cpp
 *
 * This file contains the implementation of the TrackItJacobian class, which uses the kdl library
 * for calculating and publishing the Jacobian matrix for a robot.
 * The Jacobian matrix relates the end-effector's velocity to the joint velocities of the robot.
 *
 * @param robot_description: urdf of the robot in the ros parameter server
 * @param control_frame: the base frame of the robot
 * @param tool_frame: the end effector frame of the robot
 * @param damped_inverse: if true, apply damping to the jacobian inverse
 * @param damping: the damping factor for the jacobian inverse, used if damped_inverse is true
 *
 * Subscribers:
 *     joint_states: the joint_state of the robot
 *
 * Publishers:
 *     jacobian: the jacobian of the robot for a given joint state
 *     jacobian_inverse: the inverse jacobian
 */

#include "trackit_jacobian/trackit_jacobian.hpp"
#include <fstream>
#include <string>

/**
 * Constructor for the TrackItJacobian class.
 *
 * @param n Pointer to the ROS NodeHandle object.
 *
 * @return None.
 *
 * @throws None.
 */
TrackItJacobian::TrackItJacobian(const rclcpp::NodeOptions &options) : Node ("trackit_jacobian", options) {
	
	//Node Parameters
	this->declare_parameter("control_frame", "base_link");
	this->declare_parameter("tool_frame", "tool0");
	this->declare_parameter("damped_inverse", true);
	this->declare_parameter("damping", 0.1);

	this->get_parameter("control_frame", _control_frame);
;	this->get_parameter("tool_frame", _tool_frame);
	this->get_parameter("damped_inverse", _damped_inverse);
	this->get_parameter("damping", _damping);

	// Node Subscriber
	_sub_joint_state = this->create_subscription<sensor_msgs::msg::JointState>("joint_states", 10, std::bind(&TrackItJacobian::jointStateCallback, this, std::placeholders::_1));
	
	// Node Publishers
	_pub_jacobian = this->create_publisher<trackit_msgs::msg::JacobianStamped>("jacobian", 10);
	_pub_jacobian_inverse = this->create_publisher<trackit_msgs::msg::JacobianStamped>("jacobian_inverse", 10);

	_num_joints = 0;

	std::ifstream urdf_file("/home/ryan/ros2_ws/src/trackit/trackit_jacobian/urdf/ur3e.urdf");
    std::string _robot_description((std::istreambuf_iterator<char>(urdf_file)), std::istreambuf_iterator<char>());
    this->declare_parameter("robot_description", _robot_description);

	// this->get_parameter("robot_description", _robot_description);

	// Check if robot_description is available
	if (this->get_parameter("robot_description", _robot_description)) {
		RCLCPP_INFO_STREAM(this->get_logger(), "[Node] robot_description param found and is " << _robot_description.length() << " characters long.");
	} else {
		RCLCPP_ERROR(this->get_logger(), "[Node] robot_description param NOT found!");
	}

	// construct the kdl tree, if not shutting down the jacobian node
	if (!kdl_parser::treeFromString(_robot_description, _kdl_tree)) {
		RCLCPP_ERROR(this->get_logger(), "KDL tree was not able to be constructed from the robot description, shutting down the jacobian node");
		rclcpp::shutdown();
		return;
	}

	// construct the kdl chain, if not shutting down the jacobian node
	if (!_kdl_tree.getChain(_control_frame, _tool_frame, _kdl_chain)) {
		RCLCPP_ERROR(this->get_logger(), "KDL chain was not able to be obtained, shutting down the jacobian node");
		rclcpp::shutdown();
		return;
	}

	// construct the jacobian solver
	_jacobian_solver = std::make_unique<KDL::ChainJntToJacSolver>(_kdl_chain);

	int num_seg = _kdl_chain.getNrOfSegments();

	// get and print the joint names being added to the jacobian
	for (int i = 0; i < num_seg; i++) {
		std::string joint_name = _kdl_chain.getSegment(i).getJoint().getName();

		if (_kdl_chain.getSegment(i).getJoint().getType() != KDL::Joint::JointType::None) {
			RCLCPP_INFO(this->get_logger(), "[ADDING] Joint %d Name: %s type: %s", i, joint_name.c_str(), _kdl_chain.getSegment(i).getJoint().getTypeName().c_str());//"[ADDING] Joint " << i << " Name: " << joint_name << " type: " << _kdl_chain.getSegment(i).getJoint().getTypeName());
			_joint_names.push_back(joint_name);
		}
		else {
			RCLCPP_INFO(this->get_logger(), "[IGNORE] Joint %d Name: %s type: %s", i, joint_name.c_str(), _kdl_chain.getSegment(i).getJoint().getTypeName().c_str()); //"[IGNORE] Joint " << i << " Name: " << joint_name << " type: " << _kdl_chain.getSegment(i).getJoint().getTypeName());
		}
	}

	_num_joints = _joint_names.size();

	// if all the joints are fixed or there are no joints, shut down the jacobian node
	if (_num_joints == 0)
	{
		RCLCPP_ERROR(this->get_logger(), "There is no joints in the KDL chain, shutting down the jacobian node");
		rclcpp::shutdown();
		return;
	}
}

/**
 * Callback function for the joint state message. This is the main callback for the jacobian node.
 * The joint state message is parsed into a KDL joint array and the jacobian is calculated.
 * The  inverse jacobian is also calculated.
 *
 * @param msg Pointer to the joint state message
 *
 * @throws None
 */
void TrackItJacobian::jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg) {
	
	// This checks if the number of joints in the joint state message at least matches the number of joints in the KDL chain
	if (_num_joints <= msg->position.size()) {
		
		// Create the KDL joint array based on the number of joints
		KDL::JntArray kdl_joint_array;
		kdl_joint_array.resize(_num_joints);

		// Sort the joint state message into the KDL joint array based on the joint names
		for (int i = 0; i < _num_joints; i++) {
			kdl_joint_array.data[i] = msg->position[std::find(msg->name.begin(), msg->name.end(), _joint_names[i]) - msg->name.begin()];
		}

		// Calculate the jacobian
		KDL::Jacobian kdl_jacobian;
		kdl_jacobian.resize(_num_joints);

		if (_jacobian_solver->JntToJac(kdl_joint_array, kdl_jacobian) < 0) {
			RCLCPP_ERROR(this->get_logger(), "Could not calculate Jacobian");
			return;
		}

		// Convert the KDL jacobian to an eigen matrix
		const Eigen::MatrixXd &eig_jacobian = kdl_jacobian.data;

		// Setup the Jacobian msg
		trackit_msgs::msg::JacobianStamped jacobian_msg;

		jacobian_msg.header.frame_id = _control_frame;
		jacobian_msg.header.stamp = msg->header.stamp;

		// joint order set by the jacobian is passed on for other nodes using this message
		jacobian_msg.joint_names = _joint_names;

		jacobian_msg.jacobian.data.clear();
		jacobian_msg.jacobian.layout.dim.resize(2);
		jacobian_msg.jacobian.layout.dim[0].label = "rows";
		jacobian_msg.jacobian.layout.dim[0].size = 6; // Assuming that the jacobian will always be for a 6 DOF Cartesian
		jacobian_msg.jacobian.layout.dim[0].stride = 1;
		jacobian_msg.jacobian.layout.dim[1].label = "columns";
		jacobian_msg.jacobian.layout.dim[1].size = _num_joints;
		jacobian_msg.jacobian.layout.dim[1].stride = 1;

		for (int r = 0; r < 6; r++) {
			for (int c = 0; c < _num_joints; c++) {
				jacobian_msg.jacobian.data.push_back(eig_jacobian(r, c));
			}
		}

		// Invert the jacobian and put it into a msg
		trackit_msgs::msg::JacobianStamped inv_jacobian_msg;

		inv_jacobian_msg.header.frame_id = _control_frame;
		inv_jacobian_msg.header.stamp = msg->header.stamp;

		inv_jacobian_msg.joint_names = _joint_names;

		inv_jacobian_msg.jacobian.data.clear();
		inv_jacobian_msg.jacobian.layout.dim.resize(2);
		inv_jacobian_msg.jacobian.layout.dim[0].label = "rows";
		inv_jacobian_msg.jacobian.layout.dim[0].size = _num_joints;
		inv_jacobian_msg.jacobian.layout.dim[0].stride = 1;
		inv_jacobian_msg.jacobian.layout.dim[1].label = "columns";
		inv_jacobian_msg.jacobian.layout.dim[1].size = 6; // Since it is inverted, the column should now be 6
		inv_jacobian_msg.jacobian.layout.dim[1].stride = 1;

		Eigen::MatrixXd eig_inv_jacobian;
		eig_inv_jacobian.resize(_num_joints, 6);

		// Calculate the inverse jacobian
		if (_damped_inverse) {
			Eigen::MatrixXd eig_i;
			eig_i.resize(6, 6);
			eig_i.setIdentity();
			eig_inv_jacobian = eig_jacobian.transpose() * (eig_jacobian * eig_jacobian.transpose() + _damping * eig_i).inverse();
		}
		else {
			if (_num_joints == 6) {
				eig_inv_jacobian = eig_jacobian.inverse();
			}
			else {
				// Moore Penrose pseudoinverse for 7 or more joints since the matrix isn't square
				eig_inv_jacobian = eig_jacobian.transpose() * (eig_jacobian * eig_jacobian.transpose()).inverse();
			}
		}

		// Move the data into the msg
		for (int r = 0; r < _num_joints; r++) {
			for (int c = 0; c < 6; c++) {
				inv_jacobian_msg.jacobian.data.push_back(eig_inv_jacobian(r, c));
			}
		}

		// Publish the messages
		_pub_jacobian->publish(jacobian_msg);
		_pub_jacobian_inverse->publish(inv_jacobian_msg);
	}
}

int main(int argc, char **argv) {
	// Initialise jacobian node
	rclcpp::init(argc, argv);
	rclcpp::NodeOptions options;
	auto jacobian_node = std::make_shared<TrackItJacobian>(options);
	rclcpp::spin(jacobian_node);
	rclcpp::shutdown();
	return 0;
}