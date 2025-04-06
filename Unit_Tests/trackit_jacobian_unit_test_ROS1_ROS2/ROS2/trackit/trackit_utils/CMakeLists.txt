cmake_minimum_required(VERSION 3.0.2)
project(trackit_utils)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  rospy
  sensor_msgs
  std_msgs
  tf2_geometry_msgs
  tf2_ros
  std_srvs
  trackit_msgs
)

catkin_package(
  INCLUDE_DIRS include
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(twist_iir_lpf src/twist_iir_lpf.cpp)
add_dependencies(twist_iir_lpf ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(twist_iir_lpf ${catkin_LIBRARIES})

add_executable(twist_transform src/twist_transform.cpp)
add_dependencies(twist_transform ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(twist_transform ${catkin_LIBRARIES})

add_executable(wrench_iir_lpf src/wrench_iir_lpf.cpp)
add_dependencies(wrench_iir_lpf ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(wrench_iir_lpf ${catkin_LIBRARIES})

add_executable(wrench_transform src/wrench_transform.cpp)
add_dependencies(wrench_transform ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(wrench_transform ${catkin_LIBRARIES})

add_executable(tf_tracking_error src/tf_tracking_error.cpp)
add_dependencies(tf_tracking_error ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(tf_tracking_error ${catkin_LIBRARIES})

add_executable(tf_velocity src/tf_velocity.cpp)
add_dependencies(tf_velocity ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(tf_velocity ${catkin_LIBRARIES})

add_executable(path_marker src/path_marker.cpp)
add_dependencies(path_marker ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(path_marker ${catkin_LIBRARIES})