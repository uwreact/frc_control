#!/bin/bash

if [ $# -ne 1 ]; then
  echo "Please specify the ROS workspace"
  exit 1
fi

if [ ! -d "$1" ]; then
  echo "Please specify a valid ROS workspace"
  exit 1
fi

SCRIPT=`realpath "$0"`
SCRIPTPATH=`dirname "$SCRIPT"`
WSPATH=`realpath "$1"`

cp "${SCRIPTPATH}/rostoolchain.cmake" "${WSPATH}"

cd "${WSPATH}"

# Setup the cross-compile profile
catkin profile add cross
catkin profile set cross
catkin config --extend /usr/arm-frc-linux-gnueabi/opt/ros/kinetic\
              --space-suffix _cross\
              --blacklist frc_robot_sim\
              -DCMAKE_TOOLCHAIN_FILE="${WSPATH}/rostoolchain.cmake"

# Setup the native compile profile
catkin profile add native
catkin profile set native
catkin config --extend /opt/ros/kinetic\
              --space-suffix _native\
