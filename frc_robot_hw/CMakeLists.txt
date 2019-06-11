cmake_minimum_required(VERSION 3.6) # We require 3.6 for list(FILTER)
project(frc_robot_hw)

# Load catkin and package dependencies
find_package(catkin REQUIRED COMPONENTS
  controller_manager
  hardware_interface
  joint_limits_interface
  realtime_tools
  roscpp
  #tinyxml2 #TODO: cmake_modules
  transmission_interface
  urdf

  analog_controller
  binary_controller
  frc_msgs
  ternary_controller
  pdp_state_controller
)

# Declare a catkin package
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS
    controller_manager
    hardware_interface
    joint_limits_interface
    realtime_tools
    roscpp
    #tinyxml2
    transmission_interface
    urdf

    analog_controller
    binary_controller
    frc_msgs
    ternary_controller
    pdp_state_controller
)

# Specify additional locations of header files
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

# Optionally enable external vendor-specific libraries.
option(CTRE "Use CTRE Phoenix library (eg. CAN-enabled TalonSRX and VictorSPX)" OFF)
if (CTRE)
  add_definitions(-DUSE_CTRE=1)
endif()

option(KAUAI "Use Kauai Labs NavX library" OFF)
if (KAUAI)
  add_definitions(-DUSE_KAUAI=1)
endif()

option(MINDSENSORS "DEPRECATED! Use Mindsensors library (eg. CANSD450 and CANLIGHTS)" OFF)
if (MINDSENSORS)
  message(WARNING "Mindsensors libraries are not supported since 2019!")
  # add_definitions(-DUSE_MINDSENSORS=1)
endif()

# Compile the standard, generic base classes for both sim and real
add_library(${PROJECT_NAME}
  src/frc_robot_hw.cpp
  src/robot_control_loop.cpp
)

target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
)

# If we're compiling for the RoboRIO, compile the robot_hw_real and link the WPILib libraries
if (DEFINED CMAKE_TOOLCHAIN_FILE)

  message(STATUS "Performing cross-compile, linking WPILib")

  # Find OpenCV
  set(OpenCV_DIR ${CMAKE_FIND_ROOT_PATH}/share/OpenCV/)
  find_package(OpenCV)

  # Extract WPI libraries and headers from maven archives
  add_custom_target(extract_wpilib COMMAND ${CMAKE_CURRENT_SOURCE_DIR}/../extract_archives.py)
  add_dependencies(${PROJECT_NAME} extract_wpilib)

  # Include WPILib and vendor headers
  file(GLOB WPILIB_HEADERS $ENV{HOME}/frc2019/extracted/include/*)
  include_directories(
   ${WPILIB_HEADERS}
  )

  # Compile WPILib libraries from source
  file(GLOB uncompiled_artifacts $ENV{HOME}/frc2019/extracted/src/*/)
  foreach(artifact_path ${uncompiled_artifacts})
    get_filename_component(artifact ${artifact_path} NAME [CACHE])

    # Find all source files
    file(GLOB_RECURSE ${artifact}_SRC ${artifact_path}/*.cpp ${artifact_path}/*.cc)
    list(FILTER ${artifact}_SRC EXCLUDE REGEX "jni") # Ignore anything with "jni" in it
    list(FILTER ${artifact}_SRC EXCLUDE REGEX "sim") # Ignore anything with "sim" in it

    add_library(${artifact} ${${artifact}_SRC})
    target_link_libraries(${artifact} ${OpenCV_LIBS})
    list(APPEND build_wpilibs ${artifact})
  endforeach()

  # Setup linker to link WPILib libraries
  set(WPILIB_LIB_DIR $ENV{HOME}/frc2019/extracted/libathena)
  file(GLOB WPILIB_LIBS ${WPILIB_LIB_DIR}/*/*.so
                        ${WPILIB_LIB_DIR}/*/*.so.[0-9]*) #TODO: This is super hackey, but it filters out so.debugs

  add_executable(robot_base_node
    src/frc_robot_hw_real.cpp
    src/robot_base_node.cpp
  )

  # Link ROS, WPILib, and vendor libraries
  target_link_libraries(robot_base_node
    ${PROJECT_NAME}
    ${catkin_LIBRARIES}
    ${build_wpilibs}
    ${WPILIB_LIBS}
  )

  set(extra_targets robot_base_node)

# If we're compiling for a normal Ubuntu image, do not compile real-robot WPILib code
else()
  message(STATUS "Performing native compile, NOT linking WPILib")
endif()

# Install headers
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)

# Install libraries
install(TARGETS ${PROJECT_NAME} ${extra_targets}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Install config files
install(DIRECTORY config/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
