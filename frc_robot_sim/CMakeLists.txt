cmake_minimum_required(VERSION 2.8.3)
project(frc_robot_sim)

# Optionally enable external vendor-specific libraries.
option(CTRE "Use CTRE Phoenix library (eg. CAN-enabled TalonSRX and VictorSPX)" OFF)
if (CTRE)
  add_definitions(-DUSE_CTRE=1)
endif()

option(KAUAI "Use Kauai Labs NavX library" OFF)
if (KAUAI)
  add_definitions(-DUSE_KAUAI=1)
endif()

# Load catkin and package dependencies
find_package(catkin REQUIRED COMPONENTS
  frc_robot_hw
  gazebo_ros
  roscpp
)

# System dependencies are found with CMake's conventions
find_package(GAZEBO REQUIRED)

# Declare a catkin package
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS
    frc_robot_hw
    gazebo_ros
    roscpp
  DEPENDS GAZEBO
)

# Specify additional locations of header files
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${GAZEBO_INCLUDE_DIRS}
)

# Compile the Gazebo plugin
add_library(${PROJECT_NAME} SHARED
  src/frc_robot_hw_sim.cpp
  src/gazebo_hw_plugin.cpp
)

target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${GAZEBO_LIBRARIES}
)

# Install headers
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)

# Install libraries
install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Install launch files
install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  FILES_MATCHING PATTERN "*.launch"
)
