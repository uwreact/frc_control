#!/bin/bash

if [ $# -ne 1 ]; then
  echo "Please specify the ROS workspace"
  exit 1
fi

SCRIPT=`realpath "$0"`
SCRIPTPATH=`dirname "$SCRIPT"`
WSPATH=`realpath "$1"`

cd "${WSPATH}"
if [[ $(catkin locate 2>/dev/null) == "" ]]; then
  echo "Please specify a valid ROS workspace"
  exit 1
fi

cp "${SCRIPTPATH}/rostoolchain.cmake" "${WSPATH}"

# Setup the cross-compile profile
catkin profile add cross
catkin profile set cross
catkin config --extend ~/frc2019/roborio/arm-frc2019-linux-gnueabi/opt/ros/melodic\
              --space-suffix _cross\
              --blacklist frc_robot_sim\
              -DCMAKE_TOOLCHAIN_FILE="${WSPATH}/rostoolchain.cmake"

# Setup the native compile profile
catkin profile add native
catkin profile set native
catkin config --extend /opt/ros/melodic\
              --space-suffix _native\
