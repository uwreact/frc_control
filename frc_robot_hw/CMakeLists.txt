cmake_minimum_required(VERSION 2.8.3)
project(frc_robot_hw)

# Extract WPI libraries and headers from maven archives
add_custom_target(extract_wpilib COMMAND ${CMAKE_CURRENT_SOURCE_DIR}/../extract_archives.sh)

# Load catkin and package dependencies
find_package(catkin REQUIRED COMPONENTS
  controller_manager
  hardware_interface
  joint_limits_interface
  transmission_interface
  realtime_tools
  roscpp
  #tinyxml2 #TODO: cmake_modules
  urdf

  analog_state_controller
  binary_state_controller
  ternary_state_controller
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
    transmission_interface
    realtime_tools
    roscpp
    #tinyxml2
    urdf

    analog_state_controller
    binary_state_controller
    ternary_state_controller
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

option(NAVX "Use Kauai Labs NavX library" OFF)
if (NAVX)
  add_definitions(-DUSE_NAVX=1)
endif()

option(MINDSENSORS "Use Mindsensors library (eg. CANSD450 and CANLIGHTS)" OFF)
if (MINDSENSORS)
  add_definitions(-DUSE_MINDSENSORS=1)
endif()

# Compile the standard, generic base classes for both sim and real
add_library(${PROJECT_NAME}
  src/frc_robot_hw.cpp
  src/robot_control_loop.cpp
)
add_dependencies(${PROJECT_NAME} extract_wpilib)

target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
)

# If we're compiling for the RoboRIO, compile the robot_hw_real and link the WPILib libraries
if (DEFINED CMAKE_TOOLCHAIN_FILE)

  message(STATUS "Performing cross-compile, linking WPILib")

  # Include WPILib and vendor headers
  file(GLOB WPILIB_HEADERS $ENV{HOME}/frc2019/extracted/include/*)
  include_directories(
   ${WPILIB_HEADERS}
  )

  # Setup linker to link WPILib libraries
  set (WPILIB_LIB_DIR $ENV{HOME}/frc2019/extracted/lib_athena)
  file(GLOB WPILIB_LIBS ${WPILIB_LIB_DIR}/*/*.so
                        ${WPILIB_LIB_DIR}/*/*.so.[0-9]*) #TODO: This is super hackey, but it filters out so.debugs
  link_directories(
    ${WPILIB_LIB_DIR}
  )

  # List all user libraries for linking
  # NOTE: This will link ALL libraries in the directory, not just the ones enabled by the options
  # file(GLOB USER_LIBS ${WPI_USER_LIB_DIR}/*.a ${WPI_USER_LIB_DIR}/*.so)

  # NOTE: Hack for CTRE. libCTRE_Phoenix.a must be linked before libCTRE_PhoenixCCI.a
  # list(REVERSE USER_LIBS)

  add_executable(robot_base_node
    src/frc_robot_hw_real.cpp
    src/robot_base_node.cpp
  )

  # Link ROS, WPILib, and vendor libraries
  target_link_libraries(robot_base_node
    ${PROJECT_NAME}
    ${catkin_LIBRARIES}
    ${WPILIB_LIBS}
  )

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
install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Install config files
install(DIRECTORY config/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
