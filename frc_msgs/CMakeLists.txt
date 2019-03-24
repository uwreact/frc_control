cmake_minimum_required(VERSION 2.8.3)
project(frc_msgs)

# Load catkin and package dependencies
find_package(catkin REQUIRED COMPONENTS
  message_generation
  sensor_msgs
  std_msgs
)

# List messages for generation
add_message_files(
  FILES
    CANStatus.msg
    DriverStationMode.msg
    JoyArray.msg
    JoyFeedback.msg
    MatchData.msg
    MatchTime.msg
    RobotState.msg
)

# Generate messages with the specified dependencies
generate_messages(
  DEPENDENCIES
    sensor_msgs
    std_msgs
)

# Declare a catkin package
catkin_package(
  CATKIN_DEPENDS
    message_runtime
    sensor_msgs
    std_msgs
)
