cmake_minimum_required(VERSION 3.5.0)
project(trackit_robot_state)

find_package(catkin REQUIRED COMPONENTS
  kdl_parser
  roscpp
  rospy
  std_msgs
  urdf
  trackit_msgs
)

catkin_package(
  INCLUDE_DIRS include
)


include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(joint_velocity_limit_publisher src/joint_velocity_limit_publisher.cpp)
add_dependencies(joint_velocity_limit_publisher ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(joint_velocity_limit_publisher ${catkin_LIBRARIES})

add_executable(joint_limit_aggregator src/joint_limit_aggregator.cpp)
add_dependencies(joint_limit_aggregator ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(joint_limit_aggregator ${catkin_LIBRARIES})
