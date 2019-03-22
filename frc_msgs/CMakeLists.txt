cmake_minimum_required(VERSION 2.8.3)
project(frc_msgs)

# Load catkin and package dependencies
find_package(catkin REQUIRED COMPONENTS
  std_msgs
  sensor_msgs
  message_generation
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
    std_msgs
    sensor_msgs
)

# Declare a catkin package
catkin_package(
  CATKIN_DEPENDS
    std_msgs
    sensor_msgs
    message_runtime
)
