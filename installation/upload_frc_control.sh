#!/bin/sh

if [ $# -ne 1 ]; then
  echo "Please specify the team number"
  exit 1
fi

remote="admin@roborio-$1-frc.local"
localroot=~/frc2019/roborio/arm-frc2019-linux-gnueabi

run_remote() {
  ssh $remote -q "$@"
}

write_remote() {
  run_remote "echo $1 > $2"
}

append_remote() {
  run_remote "echo $1 >> $2"
}

# Set the date on the RIO to match the local machine
run_remote date --set \"$(date +%y%m%d%H%M.%S)\"


##### Copy over dependencies #####
# Headers and cmake stuff - Not required to run code
# rsync -e 'ssh -q' -rqaH --no-i-r $localroot/include/urdf* $remote:/usr/local/ros/include
# rsync -e 'ssh -q' -rqaH --no-i-r $localroot/usr/local/include/Poco* $remote:/usr/local/ros/include
# rsync -e 'ssh -q' -rqaH --no-i-r $localroot/lib/urdf* $remote:/usr/local/ros/lib

# Libraries
rsync -e 'ssh -q' -rqaH --no-i-r $localroot/lib/liburdf* $remote:/usr/local/ros/lib
rsync -e 'ssh -q' -rqaH --no-i-r $localroot/usr/local/lib/libPocoFoundation.so.60 $remote:/usr/local/ros/lib


##### Copy over WPILib #####
rsync -e 'ssh -q' -rqaH --no-i-r ~/frc2019/extracted/libathena/*/*d.so $remote:/usr/local/frc/third-party/lib
rsync -e 'ssh -q' -rqaH --no-i-r ~/frc2019/extracted/libathena/opencv/libopencv_*d.so.3.4 $remote:/usr/local/frc/third-party/lib


##### (Somewhat hackily) Install frc_control to the RIO #####
# TODO: Delete old files
ws=$(catkin locate)
rsync -e 'ssh -q' -rqaH --no-i-r $ws/install_cross/include $remote:/opt/ros/melodic
rsync -e 'ssh -q' -rqaH --no-i-r $ws/install_cross/lib     $remote:/opt/ros/melodic
rsync -e 'ssh -q' -rqaH --no-i-r $ws/install_cross/share   $remote:/opt/ros/melodic
