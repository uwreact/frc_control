#!/bin/sh
#
# Install a locally cross-compiled roscore onto the roboRIO.
#

if [ $# -ne 1 ]; then
  echo "Please specify the team number"
  exit 1
fi

remote="admin@roborio-$1-frc.local"
localroot=~/frc2019/roborio/arm-frc2019-linux-gnueabi

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


##### Kill robot code #####
printf "\n${blue}>>>>> Stopping previous robot code${normal}\n"
run_remote ". /etc/profile.d/natinst-path.sh; /usr/local/frc/bin/frcKillRobot.sh -t"


##### Install online packages #####
update=1
if [ $update -gt 0 ]; then
  printf "\n${blue}>>>>> Updating opkg${normal}\n"
  run_remote opkg update
  printf "\n${blue}>>>>> Upgrading opkg packages${normal}\n"
  run_remote opkg upgrade

  # Install rsync to facilitate copying files
  printf "\n${blue}>>>>> Installing rsync${normal}\n"
  run_remote opkg install --volatile-cache rsync # Not required to run code, but installed so we don't have to use scp

  # Install dependencies
  printf "\n${blue}>>>>> Installing dependencies${normal}\n"
  run_remote opkg install --volatile-cache gcc g++ gcc-symlinks g++-symlinks binutils # Is this really needed? Kinda big
  run_remote opkg install --volatile-cache libboost-atomic1.63.0 libboost-chrono1.63.0 libboost-date-time1.63.0\
                                           libboost-filesystem1.63.0 libboost-graph1.63.0 libboost-iostreams1.63.0\
                                           libboost-program-options1.63.0 libboost-random1.63.0 libboost-regex1.63.0\
                                           libboost-signals1.63.0 libboost-system1.63.0 libboost-thread1.63.0\
                                           libboost-timer1.63.0
  run_remote opkg install --volatile-cache python-pip python-dev
  run_remote pip install --no-cache-dir --upgrade pip
  run_remote pip install --no-cache-dir catkin_pkg rosdep rospkg # ROS
  run_remote pip install --no-cache-dir defusedxml pycrypto python-gnupg netifaces # ROS deps
fi


##### Copy over the ROS environment #####
printf "\n${blue}>>>>> Uploading ROS${normal}\n"
run_remote mkdir -p /opt/ros/
rsync -e 'ssh -q' -rqaH --no-i-r $localroot/opt/ros/melodic/ $remote:/opt/ros/melodic/ --delete

if ! run_remote '[ -d /usr/local/ros ]'; then
  run_remote mkdir /usr/local/ros
  write_remote "/usr/local/ros/lib" /etc/ld.so.conf.d/ros_libs.conf
  run_remote ldconfig
fi

# Headers and cmake stuff - Not required to run code, only required for local builds
# rsync -e 'ssh -q' -rqaH --no-i-r $localroot/include/console_bridge* $remote:/usr/local/ros/include
# rsync -e 'ssh -q' -rqaH --no-i-r $localroot/include/tinyxml* $remote:/usr/local/ros/include
# rsync -e 'ssh -q' -rqaH --no-i-r $localroot/lib/console_bridge* $remote:/usr/local/ros/lib
# rsync -e 'ssh -q' -rqaH --no-i-r $localroot/lib/cmake/tinyxml* $remote:/usr/local/ros/lib/cmake

# Libraries
printf "\n${blue}>>>>> Uploading ROS dependencies${normal}\n"
rsync -e 'ssh -q' -rqaH --no-i-r $localroot/lib/libconsole_bridge.so* $remote:/usr/local/ros/lib
rsync -e 'ssh -q' -rqaH --no-i-r $localroot/lib/libtinyxml2.so* $remote:/usr/local/ros/lib # Needed for rospack


##### Edit os_detect to work on nilrt #####
# Add `OS_NILRT = 'nilrt'`
# Add `OsDetect.register_default(OS_NILRT, FdoDetect("nilrt"))`
printf "\n${blue}>>>>> Modifying os_detect to support nilrt${normal}\n"
f=/usr/lib/python2.7/site-packages/rospkg/os_detect.py
if ! run_remote grep nilrt $f > /dev/null; then

  # TODO(matt.reynolds): I'm bad at awk so using sed for now.
  run_remote sed -i "\"/^OS_UBUNTU = 'ubuntu'/a OS_NILRT = 'nilrt'\"" $f
  run_remote sed -i "\"/^OsDetect.register_default(OS_UBUNTU, LsbDetect(\\\"Ubuntu\\\"))/a OsDetect.register_default(OS_NILRT, FdoDetect('nilrt'))\"" $f

  # run_remote awk -i inplace "\"FNR==NR{ if (/OS_*/) p=NR; next} 1; FNR==p{ print \\\"OS_NILRT = 'nilrt'\\\" }; { print }\"" $f
  # run_remote awk -i inplace "\"FNR==NR{ if (/OsDetect.register_default.*/) p=NR; next} 1; FNR==p{ print \\\"OsDetect.register_default(OS_NILRT, FdoDetect('nilrt'))\\\" }; print\"" $f
fi


##### Source ROS #####
printf "\n${blue}>>>>> Modifying /etc/profile.d to source the ROS environment${normal}\n"
f=/etc/profile.d/ros-env.sh
write_remote "" $f
append_remote "\# Source ROS" $f
append_remote "source /opt/ros/user/setup.bash" $f
append_remote "export ROS_HOSTNAME=\$(hostname).local" $f
run_remote chmod +x $f


printf "\n${blue}>>>>> Cleaning up\n"
run_remote ldconfig
run_remote sync


printf "\n${blue}>>>>> Done\n\n"
