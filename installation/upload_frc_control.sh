#!/bin/sh

# TODO(matt.reynolds): Better method to specify the bringup launch file.
# Maybe save team number and launchfile to a config file somwhere to read as default

if [ $# -lt 1 ]; then
  echo "Please specify the team number"
  exit 1
fi

if [ $# -lt 2 ]; then
  echo "Using default launch file frc_robot_hw bringup.launch"
  launchfile="frc_robot_hw bringup.launch"
fi

remote="admin@roborio-$1-frc.local"
localroot=~/frc2019/roborio/arm-frc2019-linux-gnueabi
launchfile="$2 $3"

blue=$(tput setaf 4)
normal=$(tput sgr0)

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


##### Kill and remove old robot code #####
printf "\n${blue}>>>>> Stopping previous robot code${normal}\n"
run_remote ". /etc/profile.d/natinst-path.sh; /usr/local/frc/bin/frcKillRobot.sh -t"
run_remote rm -f /home/lvuser/frcUserProgram


##### Copy over dependencies #####
printf "\n${blue}>>>>> Uploading dependencies${normal}\n"
# Headers and cmake stuff - Not required to run code
# rsync -e 'ssh -q' -rqaH --no-i-r $localroot/include/urdf* $remote:/usr/local/ros/include
# rsync -e 'ssh -q' -rqaH --no-i-r $localroot/usr/local/include/Poco* $remote:/usr/local/ros/include
# rsync -e 'ssh -q' -rqaH --no-i-r $localroot/lib/urdf* $remote:/usr/local/ros/lib

# Libraries
rsync -e 'ssh -q' -rqaH --no-i-r $localroot/lib/liburdf* $remote:/usr/local/ros/lib
rsync -e 'ssh -q' -rqaH --no-i-r $localroot/usr/local/lib/libPocoFoundation.so.60 $remote:/usr/local/ros/lib


##### Copy over WPILib libraries #####
printf "\n${blue}>>>>> Uploading WPILib${normal}\n"
rsync -e 'ssh -q' -rqaH --no-i-r ~/frc2019/extracted/libathena/*/*d.so $remote:/usr/local/frc/third-party/lib
rsync -e 'ssh -q' -rqaH --no-i-r ~/frc2019/extracted/libathena/opencv/libopencv_*d.so.3.4 $remote:/usr/local/frc/third-party/lib


##### Copy over vendor libraries #####
# NOTE: Disabled since vendor libs are linked statically
#
# printf "\n${blue}>>>>> Uploading vendor libs${normal}\n"
# for dir in ~/frc2019/extracted/libathena/*; do
#   case $(basename $dir) in
#     cameraserver|chipobject|cscore|hal|netcomm|ntcore|opencv|wpilibc|wpiutil) continue;;
#     *) ;;
#   esac
#
#   rsync -e 'ssh -q' -rqaH --no-i-r $dir/*.so $remote:/usr/local/frc/third-party/lib
# done


##### (Somewhat hackily) Install frc_control to the RIO #####
# TODO: Delete old files
printf "\n${blue}>>>>> Uploading frc_control${normal}\n"
ws=$(catkin locate)
rsync -e 'ssh -q' -rqaH --no-i-r $ws/install_cross/include $remote:/opt/ros/melodic
rsync -e 'ssh -q' -rqaH --no-i-r $ws/install_cross/lib     $remote:/opt/ros/melodic
rsync -e 'ssh -q' -rqaH --no-i-r $ws/install_cross/share   $remote:/opt/ros/melodic


##### Set file permissions on uploaded files #####
printf "\n${blue}>>>>> Setting permissions\n"
run_remote chmod -R 777 /usr/local/frc/third-party/lib
run_remote chown -R lvuser:ni /usr/local/frc/third-party/lib
run_remote ldconfig


##### Start robot code #####
printf "\n${blue}>>>>> Starting robot code${normal}\n"
write_remote "roslaunch ${launchfile}" /home/lvuser/robotCommand
run_remote chmod +x /home/lvuser/robotCommand
run_remote chown lvuser /home/lvuser/robotCommand
run_remote ". /etc/profile.d/natinst-path.sh; /usr/local/frc/bin/frcKillRobot.sh -t -r"


printf "\n${blue}>>>>> Cleaning up\n"
run_remote sync


printf "\n${blue}>>>>> Done\n\n"
