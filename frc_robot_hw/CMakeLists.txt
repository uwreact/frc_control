cmake_minimum_required(VERSION 2.8.3)
project(frc_robot_hw)

# Enable C++11
# Note: We are unable to use C++03 for Kinetic releases because the WPILib requires at least C++11
add_compile_options(-std=c++11)

# Load catkin and package dependencies
find_package(catkin REQUIRED COMPONENTS
  controller_manager
  hardware_interface
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
    realtime_tools
    roscpp
    #tinyxml2
    urdf

    analog_state_controller
    binary_state_controller
    ternary_state_controller
    pdp_state_controller
)

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

target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
)

# If we're compiling for the RoboRIO, compile the robot_hw_real and link the WPILib libraries
if (DEFINED CMAKE_TOOLCHAIN_FILE)

  message(STATUS "Performing cross-compile, linking WPILib")

  # Include WPILib and vendor headers
  include_directories(
    $ENV{HOME}/wpilib/user/cpp/include
    $ENV{HOME}/wpilib/cpp/current/include
  )

  # Setup linker to link WPILib libraries
  set (WPI_LIB_DIR        $ENV{HOME}/wpilib/cpp/current/reflib/linux/athena/shared)
  set (WPI_COMMON_LIB_DIR $ENV{HOME}/wpilib/common/current/lib/linux/athena/shared)
  set (WPI_USER_LIB_DIR   $ENV{HOME}/wpilib/user/cpp/lib)

  link_directories(
    ${WPI_LIB_DIR}
    ${WPI_COMMON_LIB_DIR}
    ${WPI_USER_LIB_DIR}
  )

  # List all user libraries for linking
  # NOTE: This will link ALL libraries in the directory, not just the ones enabled by the options
  file(GLOB USER_LIBS ${WPI_USER_LIB_DIR}/*.a ${WPI_USER_LIB_DIR}/*.so)

  # NOTE: Hack for CTRE. libCTRE_Phoenix.a must be linked before libCTRE_PhoenixCCI.a
  list(REVERSE USER_LIBS)

  add_executable(robot_base_node
    src/frc_robot_hw_real.cpp
    src/robot_base_node.cpp
  )

  # Link ROS, WPILib, and vendor libraries
  target_link_libraries(robot_base_node
    ${PROJECT_NAME}
    ${catkin_LIBRARIES}
    ${USER_LIBS}
    ${WPI_LIB_DIR}/libwpi.so
    ${WPI_LIB_DIR}/libFRC_NetworkCommunication.so.18.0.0
    ${WPI_LIB_DIR}/libRoboRIO_FRC_ChipObject.so.18.0.0
  )

# If we're compiling for a normal Ubuntu image, do not compile real-robot WPILib code
else()
  message(STATUS "Performing native compile, NOT linking WPILib")
endif()
