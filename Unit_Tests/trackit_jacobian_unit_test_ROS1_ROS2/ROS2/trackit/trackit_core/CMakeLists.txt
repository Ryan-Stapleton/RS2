cmake_minimum_required(VERSION 3.5)
project(trackit_core)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(trackit_msgs REQUIRED)
find_package(Eigen3 REQUIRED)

find_package(ament_cmake_gtest REQUIRED)

# Include directories
include_directories(include)

# Add executable
add_executable(trackit_idk src/trackit_idk.cpp)

# Link dependencies
ament_target_dependencies(trackit_idk rclcpp geometry_msgs sensor_msgs std_msgs trackit_msgs Eigen3)

# Add executable for the test
ament_add_gtest(test_trackit_idk test/test_trackit_idk.cpp)

# Link dependencies for the test
ament_target_dependencies(test_trackit_idk rclcpp sensor_msgs trackit_msgs)

# Install test executable
install(TARGETS test_trackit_idk
  DESTINATION lib/${PROJECT_NAME}
)

# Install target
install(TARGETS trackit_idk
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()