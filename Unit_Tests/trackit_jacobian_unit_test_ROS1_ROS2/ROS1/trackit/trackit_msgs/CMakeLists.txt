cmake_minimum_required(VERSION 3.5.0)
project(trackit_msgs)

find_package(catkin REQUIRED COMPONENTS
  cmake_modules
  message_generation
  geometry_msgs
  std_msgs
)

add_message_files(DIRECTORY msg)

add_service_files(DIRECTORY srv)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)

catkin_package(
  CATKIN_DEPENDS message_runtime geometry_msgs std_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)