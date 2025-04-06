cmake_minimum_required(VERSION 3.5.0)
project(trackit_velocity_commands)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  rospy
  std_msgs
  sensor_msgs
  tf2_ros
  tf2_geometry_msgs
  trackit_msgs
)


catkin_package(
  INCLUDE_DIRS include
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(joy_to_twist src/joy_to_twist.cpp)
add_dependencies(joy_to_twist ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(joy_to_twist ${catkin_LIBRARIES})

add_executable(admittance_twist_basic src/admittance_twist_basic.cpp)
add_dependencies(admittance_twist_basic ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(admittance_twist_basic ${catkin_LIBRARIES})

add_executable(admittance_twist_mass_damper src/admittance_twist_mass_damper.cpp)
add_dependencies(admittance_twist_mass_damper ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(admittance_twist_mass_damper ${catkin_LIBRARIES})

add_executable(admittance_twist_spring_mass_damper src/admittance_twist_spring_mass_damper.cpp)
add_dependencies(admittance_twist_spring_mass_damper ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(admittance_twist_spring_mass_damper ${catkin_LIBRARIES})

add_executable(pid_ff src/pid_ff.cpp)
add_dependencies(pid_ff ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(pid_ff ${catkin_LIBRARIES})

add_executable(robot_command src/robot_command.cpp)
add_dependencies(robot_command ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(robot_command ${catkin_LIBRARIES})
