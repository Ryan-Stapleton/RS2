cmake_minimum_required(VERSION 3.0.2)
project(trackit_jacobian)

find_package(catkin REQUIRED COMPONENTS
  kdl_parser
  roscpp
  rospy
  sensor_msgs
  std_msgs
  trackit_msgs
)

find_package(Boost REQUIRED COMPONENTS system)
find_package(orocos_kdl REQUIRED)
find_package(Eigen3 REQUIRED)

catkin_package(
  INCLUDE_DIRS include
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIRS}
)

add_executable(trackit_jacobian src/trackit_jacobian.cpp)
add_dependencies(trackit_jacobian ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(trackit_jacobian ${catkin_LIBRARIES})

add_executable(test_trackit_jacobian test/test_trackit_jacobian.cpp)
target_link_libraries(test_trackit_jacobian ${catkin_LIBRARIES} ${GTEST_LIBRARIES})

# if(CATKIN_ENABLE_TESTING)
#   catkin_add_gtest(test_trackit_jacobian
#     test/test_trackit_jacobian.cpp
#   )
#   # Link against the aggregator library (not the node)
#   target_link_libraries(test_trackit_jacobian
#     ${catkin_LIBRARIES}
#     ${GTEST_LIBRARIES}
#   )
# endif()