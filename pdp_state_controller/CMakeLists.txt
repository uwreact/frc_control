cmake_minimum_required(VERSION 2.8.3)
project(pdp_state_controller)

# Load catkin and package dependencies
find_package(catkin REQUIRED COMPONENTS
  controller_interface
  hardware_interface
  message_generation
  message_runtime
  pluginlib
  realtime_tools
  roscpp
  std_msgs
)

add_message_files(
   FILES
   PDPData.msg
)

generate_messages(
   DEPENDENCIES
   std_msgs
)

# Declare a catkin package
catkin_package(
   INCLUDE_DIRS include
   LIBRARIES ${PROJECT_NAME}
   CATKIN_DEPENDS
    controller_interface
    hardware_interface
    message_generation
    message_runtime
    pluginlib
    realtime_tools
    roscpp
    std_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME} src/pdp_state_controller.cpp)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})
