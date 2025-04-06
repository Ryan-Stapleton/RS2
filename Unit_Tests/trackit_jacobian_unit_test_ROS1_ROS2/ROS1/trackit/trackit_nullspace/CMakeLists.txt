cmake_minimum_required(VERSION 3.5.0)
project(trackit_nullspace)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  trackit_msgs
)

find_package(Eigen3 REQUIRED)

catkin_package(
  INCLUDE_DIRS include
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(nullspace_projection src/nullspace_projection.cpp)
add_dependencies(nullspace_projection ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(nullspace_projection ${catkin_LIBRARIES})

add_executable(task_joint_middle src/task_joint_middle.cpp)
add_dependencies(task_joint_middle ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(task_joint_middle ${catkin_LIBRARIES})