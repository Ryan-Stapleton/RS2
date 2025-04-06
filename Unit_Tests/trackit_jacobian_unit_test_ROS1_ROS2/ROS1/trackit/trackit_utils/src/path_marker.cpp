/**
 * @file path_marker.cpp
 * @brief Implementation of the PathMarker class.
 * 
 * This node tracks a tf frame and creates a trail of points along the path.
 * 
 * @param marker_point_count number of markers in the trail
 * @param line_color color of the line
 * @param rate the rate at which the node runs
 * @param line_width the width of the line
 * @param control_frame the frame that the line marker will be published in
 * @param tracked_frame the frame that the node will track
 * @param marker_ns the frame that the line marker will be published in
 *  
 * Subscribers:
 * 	- joint_limits_in: The joint limits messages from other nodes.
 * 
 * Publishers:
 * 	- joint_limits_out: The aggregated joint limits
 * 
 */

#include "trackit_utils/path_marker.h"

/**
 * Constructor for the PathMarker class.
 *
 * @param n pointer to the ROS NodeHandle
 *
 * @return none
 *
 * @throws none
 */
PathMarker::PathMarker(ros::NodeHandle* n):_n(*n), _tf_listen(_tf_buffer) {
	// private nodehandle
	_nh = ros::NodeHandle("~");
	
	// node parameters
	int color;
	_nh.param<int>("marker_point_count", _marker_point_count, 100);
	_nh.param<int>("line_color", color, 1);
	_line_color = (line_color)color;

	_nh.param<double>("rate"			, _rate				, 50);
	_nh.param<double>("line_width"		, _line_width		, 0.01);
	_nh.param<double>("marker_lifetime"	, _marker_lifetime	, 5.0);

	_nh.param<std::string>("control_frame"		, _control_frame	, "base_link");
	_nh.param<std::string>("tracked_frame"		, _tracked_frame	, "ee_link");
	_nh.param<std::string>("marker_ns"			, _marker_ns		, "path_marker");

	// timer
	_timer = _n.createTimer(ros::Duration(1.0/_rate), &PathMarker::timerCallback, this);
	
	// publishers
	_pub_marker = _n.advertise<visualization_msgs::Marker>("path_marker", 1);

	// set up the marker
	_marker.header.frame_id = _control_frame;
	_marker.ns = _marker_ns;
	_marker.id = color;
	_marker.type = visualization_msgs::Marker::LINE_STRIP;
	_marker.action = visualization_msgs::Marker::ADD;
	
	_marker.pose.position.x = 0.0;
	_marker.pose.position.y = 0.0;
	_marker.pose.position.z = 0.0;
	_marker.pose.orientation.x = 0.0;
	_marker.pose.orientation.y = 0.0;
	_marker.pose.orientation.z = 0.0;
	_marker.pose.orientation.w = 1.0;
	
	_marker.scale.x = _line_width;
	
	_marker.color.a = 1.0;

	if(_line_color == BLACK) {
		_marker.color.r = 0.0;
		_marker.color.g = 0.0;
		_marker.color.b = 0.0;
	}
	else if(_line_color == RED) {
		_marker.color.r = 225.0/255.0;
		_marker.color.g = 35.0/255.0;
		_marker.color.b = 35.0/255.0;
	}
	else if(_line_color == GREEN) {
		_marker.color.r = 35.0/255.0;
		_marker.color.g = 225.0/255.0;
		_marker.color.b = 35.0/255.0;
	}
	else if(_line_color == BLUE) {
		_marker.color.r = 35.0/255.0;
		_marker.color.g = 35.0/255.0;
		_marker.color.b = 225.0/255.0;
	}
	else if(_line_color == CYAN) {
		_marker.color.r = 0.0;
		_marker.color.g = 230.0/255.0;
		_marker.color.b = 230.0/255.0;
	}
	else if(_line_color == YELLOW) {
		_marker.color.r = 255.0/255.0;
		_marker.color.g = 255.0/255.0;
		_marker.color.b = 87.0/255.0;
	}
	else if(_line_color == MAGENTA) {
		_marker.color.r = 255.0/255.0;
		_marker.color.g = 0.0;
		_marker.color.b = 255.0/255.0;
	}
	else if(_line_color == ORANGE) {
		_marker.color.r = 255.0/255.0;
		_marker.color.g = 139.0/255.0;
		_marker.color.b = 43.0/255.0;
	}
	else if(_line_color == PURPLE) {
		_marker.color.r = 209.0/255.0;
		_marker.color.g = 54.0/255.0;
		_marker.color.b = 255.0/255.0;
	}
	else if(_line_color == BROWN) {
		_marker.color.r = 131.0/255.0;
		_marker.color.g = 79.0/255.0;
		_marker.color.b = 23.0/255.0;
	}
	else if(_line_color == GREY) {
		_marker.color.r = 128.0/255.0;
		_marker.color.g = 128.0/255.0;
		_marker.color.b = 128.0/255.0;
	}
	else if(_line_color == WHITE) {
		_marker.color.r = 255.0/255.0;
		_marker.color.g = 255.0/255.0;
		_marker.color.b = 255.0/255.0;
	}

	// set the lifetime of the marker
	_marker.lifetime = ros::Duration(_marker_lifetime);
	_tf_buffer.setUsingDedicatedThread(true);
}

/**
 * Timer callback function that retrieves the current transform between
 * the control frame and the tracked frame. It adds the current tracked
 * position to the marker and publishes the updated marker.
 *
 * @param event the ROS timer event triggering the callback
 *
 * @throws tf2::TransformException if there is an error looking up the transform
 */
void PathMarker::timerCallback(const ros::TimerEvent&) {	
	
	geometry_msgs::TransformStamped tf_tracked;
	// get the transform between the tracked frame and the control frame
	try {
		tf_tracked = _tf_buffer.lookupTransform(_control_frame, _tracked_frame, ros::Time(0), ros::Duration(1.0));
	}
	catch (tf2::TransformException& ex) {
		ROS_ERROR("%s", ex.what());
		return;
	}

	// add the current position to the marker
	geometry_msgs::Point current_point;
	current_point.x = tf_tracked.transform.translation.x;
	current_point.y = tf_tracked.transform.translation.y;
	current_point.z = tf_tracked.transform.translation.z;

	_marker.points.push_back(current_point);

	// remove points that are more than _marker_point_count
	if(_marker.points.size() > _marker_point_count) {
		_marker.points.erase(_marker.points.begin());
	}

	// publish the marker
	_marker.header.stamp = ros::Time::now();
	_pub_marker.publish(_marker);
}

int main(int argc, char** argv) {
	ros::init(argc, argv, "path_marker");
	ros::NodeHandle n;
	PathMarker path_marker(&n);
	ros::spin();
	return 0;
}