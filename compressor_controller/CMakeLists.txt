cmake_minimum_required(VERSION 2.8.3)
project(compressor_controller)

# Load catkin and package dependencies
find_package(catkin REQUIRED COMPONENTS
  controller_interface
  hardware_interface
  message_generation
  pluginlib
  realtime_tools
  roscpp
  std_msgs
)

add_message_files(
   FILES
   CompressorData.msg
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
    message_runtime
    pluginlib
    realtime_tools
    roscpp
    std_msgs
)

# Specify additional locations of header files
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

# Compile the plugin
add_library(${PROJECT_NAME}
  src/compressor_command_controller.cpp
  src/compressor_state_controller.cpp
)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})
add_dependencies(${PROJECT_NAME} ${PROJECT_NAME}_generate_messages_cpp)

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

# Install plugin file
install(FILES ${PROJECT_NAME}_plugins.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
