cmake_minimum_required(VERSION 3.5.0)
project(trackit_core)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
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

# add_executable(trackit_velocity_control src/trackit_velocity_control.cpp)
# add_dependencies(trackit_velocity_control ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
# target_link_libraries(trackit_velocity_control ${catkin_LIBRARIES})

# add_executable(trackit_velocity_control_limit src/trackit_velocity_control_limit.cpp)
# add_dependencies(trackit_velocity_control_limit ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
# target_link_libraries(trackit_velocity_control_limit ${catkin_LIBRARIES})

# add_executable(trackit_velocity_control_limit_mask src/trackit_velocity_control_limit_mask.cpp)
# add_dependencies(trackit_velocity_control_limit_mask ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
# target_link_libraries(trackit_velocity_control_limit_mask ${catkin_LIBRARIES})

add_executable(trackit_idk src/trackit_idk.cpp)
add_dependencies(trackit_idk ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(trackit_idk ${catkin_LIBRARIES})

if(CATKIN_ENABLE_TESTING)
  catkin_add_gtest(test_trackit_idk
    test/test_trackit_idk.cpp
  )
  # Link against the aggregator library (not the node)
  target_link_libraries(test_trackit_idk
    ${catkin_LIBRARIES}
    ${GTEST_LIBRARIES}
  )
endif()