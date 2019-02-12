cmake_minimum_required(VERSION 2.8.3)
project(frc_msgs)

# Load catkin and package dependencies
find_package(catkin REQUIRED COMPONENTS
  std_msgs
  sensor_msgs
  message_generation
)

# Generate messages in the 'msg' folder
add_message_files(
  FILES
    CANStatus.msg
    DriverStationMode.msg
    JoyArray.msg
    JoyFeedback.msg
    MatchData.msg
    RobotState.msg
)

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#     Service1.srv
#     Service2.srv
# )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

# Generate added messages and services with any dependencies listed here
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
