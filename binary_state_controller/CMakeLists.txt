cmake_minimum_required(VERSION 2.8.3)
project(binary_state_controller)

# Load catkin and package dependencies
find_package(catkin REQUIRED COMPONENTS
  hardware_interface
  roscpp
)

# Declare a catkin package
catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS
    hardware_interface
    roscpp
)