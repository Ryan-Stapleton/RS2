cmake_minimum_required(VERSION 3.5)
project(trackit_jacobian)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(trackit_msgs REQUIRED)
find_package(kdl_parser REQUIRED)

find_package(Boost REQUIRED COMPONENTS system)
find_package(orocos_kdl REQUIRED)
find_package(Eigen3 REQUIRED)

find_package(ament_cmake_gtest REQUIRED) 

# Include directories
include_directories(
  include
  ${Boost_INCLUDE_DIRS}
  ${Eigen3_INCLUDE_DIRS}
)

# Add executable
add_executable(trackit_jacobian src/trackit_jacobian.cpp)

# Link dependencies
ament_target_dependencies(trackit_jacobian rclcpp sensor_msgs std_msgs trackit_msgs kdl_parser orocos_kdl Eigen3 Boost)

# add_executable(trackit_jacobian_elds src/trackit_jacobian_elds.cpp)
# ament_target_dependencies(trackit_jacobian_elds
#   rclcpp
#   sensor_msgs
#   std_msgs
#   trackit_msgs
#   kdl_parser
#   orocos_kdl
#   Eigen3
#   Boost
# )
# target_link_libraries(trackit_jacobian_edls)

ament_add_gtest(test_trackit_jacobian test/test_trackit_jacobian.cpp)

if(TARGET test_trackit_jacobian)
  target_include_directories(test_trackit_jacobian PRIVATE include)
  ament_target_dependencies(test_trackit_jacobian rclcpp sensor_msgs trackit_msgs)

  install(TARGETS test_trackit_jacobian
    DESTINATION lib/${PROJECT_NAME}
  )
endif()

# Install target
install(TARGETS trackit_jacobian #trackit_jacobian_edls
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()