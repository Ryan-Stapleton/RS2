#ifndef TRACKIT_UTILS_PATH_MARKER_H
#define TRACKIT_UTILS_PATH_MARKER_H

#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Point.h>
#include <visualization_msgs/Marker.h>

enum line_color {BLACK = 0, RED = 1, GREEN = 2, BLUE = 3, CYAN = 4, YELLOW = 5, MAGENTA = 6, ORANGE = 7, PURPLE = 8, BROWN = 9, GREY = 10, WHITE = 11};

class PathMarker {

public:
	PathMarker(ros::NodeHandle* n);

private:
	void timerCallback(const ros::TimerEvent&);

	ros::NodeHandle _n;
	ros::NodeHandle _nh;

	ros::Timer _timer;
	ros::Publisher _pub_marker;

	tf2_ros::Buffer _tf_buffer;
	tf2_ros::TransformListener _tf_listen;

	int _marker_point_count;
	line_color _line_color;

	double _rate;
	double _line_width;
	double _marker_lifetime;

	std::string _tracked_frame;
	std::string _control_frame;
	std::string _marker_ns;
	
	visualization_msgs::Marker _marker;
};

#endif // TRACKIT_UTILS_PATH_MARKER_H