cmake_minimum_required(VERSION 2.8.3)
project(frc_robot_sim)

add_compile_options(-std=c++11)

# Load catkin and package dependencies
find_package(catkin REQUIRED COMPONENTS
  roscpp
  gazebo_ros
  frc_robot_hw
)

# System dependencies are found with CMake's conventions
find_package(GAZEBO REQUIRED)

# Declare a catkin package
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS
    gazebo_ros
    roscpp
    frc_robot_hw
  DEPENDS GAZEBO
)

# Specify additional locations of header files
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${GAZEBO_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME} SHARED
  src/gazebo_hw_plugin.cpp
  src/frc_robot_hw_sim.cpp
)

target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${GAZEBO_LIBRARIES}
)