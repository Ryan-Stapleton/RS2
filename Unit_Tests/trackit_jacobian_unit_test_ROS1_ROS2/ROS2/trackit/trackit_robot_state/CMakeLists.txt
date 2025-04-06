cmake_minimum_required(VERSION 3.8)
project(trackit_robot_state)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(kdl_parser REQUIRED)
find_package(urdf REQUIRED)
find_package(trackit_msgs REQUIRED)
find_package(ament_cmake_gtest REQUIRED)

# Include directories
include_directories(include)

# ----------------------------------------------------------------
# Build the library (without main functions)
# ----------------------------------------------------------------
add_library(${PROJECT_NAME}_lib
  src/joint_limit_aggregator.cpp
  src/joint_velocity_limit_publisher.cpp
  src/static_joint_limit_publisher.cpp
)
ament_target_dependencies(${PROJECT_NAME}_lib
  rclcpp
  std_msgs
  sensor_msgs
  kdl_parser
  urdf
  trackit_msgs
)

# ----------------------------------------------------------------
# Build ROS2 Node executables that include the main() functions.
# (These are in separate source files so that when you build tests you avoid duplicate main()s.)
# ----------------------------------------------------------------

# Joint Limit Aggregator Node
add_executable(joint_limit_aggregator_node
  src/joint_limit_aggregator_node.cpp
)
target_link_libraries(joint_limit_aggregator_node ${PROJECT_NAME}_lib)
ament_target_dependencies(joint_limit_aggregator_node rclcpp)

# Joint Velocity Limit Publisher Node
add_executable(joint_velocity_limit_publisher_node
  src/joint_velocity_limit_publisher_node.cpp
)
target_link_libraries(joint_velocity_limit_publisher_node ${PROJECT_NAME}_lib)
ament_target_dependencies(joint_velocity_limit_publisher_node rclcpp)

# Static Joint Limit Publisher Node
add_executable(static_joint_limit_publisher_node
  src/static_joint_limit_publisher_node.cpp
)
target_link_libraries(static_joint_limit_publisher_node ${PROJECT_NAME}_lib)
ament_target_dependencies(static_joint_limit_publisher_node rclcpp)

# Install executables
install(
  TARGETS
    joint_limit_aggregator_node
    joint_velocity_limit_publisher_node
    static_joint_limit_publisher_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install header files
install(
  DIRECTORY include/
  DESTINATION include/
)

# ----------------------------------------------------------------
# Testing Section
# ----------------------------------------------------------------
if(BUILD_TESTING)
  # Optionally run lint tests
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  # Test for Joint Limit Aggregator
  ament_add_gtest(test_joint_limit_aggregator
    unit_tests/test_joint_limit_aggregator.cpp
  )
  if(TARGET test_joint_limit_aggregator)
    target_link_libraries(test_joint_limit_aggregator ${PROJECT_NAME}_lib)
    ament_target_dependencies(test_joint_limit_aggregator
      rclcpp
      std_msgs
      sensor_msgs
      kdl_parser
      urdf
      trackit_msgs
    )
  endif()

  # Test for Joint Velocity Limit Publisher
  ament_add_gtest(test_joint_velocity_limit_publisher
    unit_tests/test_joint_velocity_limit_publisher.cpp
  )
  if(TARGET test_joint_velocity_limit_publisher)
    target_link_libraries(test_joint_velocity_limit_publisher ${PROJECT_NAME}_lib)
    ament_target_dependencies(test_joint_velocity_limit_publisher
      rclcpp
      std_msgs
      sensor_msgs
      kdl_parser
      urdf
      trackit_msgs
    )
  endif()

  # Test for Static Joint Limit Publisher
  ament_add_gtest(test_static_joint_limit_publisher
    unit_tests/test_static_joint_limit_publisher.cpp
  )
  if(TARGET test_static_joint_limit_publisher)
    target_link_libraries(test_static_joint_limit_publisher ${PROJECT_NAME}_lib)
    ament_target_dependencies(test_static_joint_limit_publisher
      rclcpp
      std_msgs
      sensor_msgs
      kdl_parser
      urdf
      trackit_msgs
    )
  endif()
endif()

ament_package()
