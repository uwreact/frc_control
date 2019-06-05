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


##### Install online packages #####
update=1
if [ $update -gt 0 ]; then
  run_remote opkg update
  run_remote opkg upgrade

  # Install rsync to facilitate copying files
  run_remote opkg install --volatile-cache rsync # Not required to run code, but installed so we don't have to use scp

  # Install dependencies
  run_remote opkg install --volatile-cache gcc g++ gcc-symlinks g++-symlinks binutils # Is this really needed? Kinda big
  run_remote opkg install --volatile-cache libboost-atomic1.63.0 libboost-chrono1.63.0 libboost-date-time1.63.0 libboost-filesystem1.63.0 libboost-graph1.63.0 libboost-iostreams1.63.0 libboost-program-options1.63.0 libboost-random1.63.0 libboost-regex1.63.0 libboost-signals1.63.0 libboost-system1.63.0 libboost-thread1.63.0 libboost-timer1.63.0
  run_remote opkg install --volatile-cache python-pip python-dev
  run_remote pip install --no-cache-dir --upgrade pip
  run_remote pip install --no-cache-dir catkin_pkg rosdep rospkg # ROS
  run_remote pip install --no-cache-dir defusedxml pycrypto python-gnupg netifaces # ROS deps
fi


##### Copy over the ROS environment #####
rsync -e 'ssh -q' -rqaH --no-i-r $localroot/opt $remote:/

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
rsync -e 'ssh -q' -rqaH --no-i-r $localroot/lib/libconsole_bridge.so* $remote:/usr/local/ros/lib
rsync -e 'ssh -q' -rqaH --no-i-r $localroot/lib/libtinyxml2.so* $remote:/usr/local/ros/lib # Needed for rospack


##### Edit os_detect to work on nilrt #####
# Add `OS_NILRT = 'nilrt'`
# Add `OsDetect.register_default(OS_NILRT, FdoDetect("nilrt"))`
f=/usr/lib/python2.7/site-packages/rospkg/os_detect.py
if ! run_remote grep nilrt $f > /dev/null; then

  # TODO: I'm bad at awk so using sed for now.
  run_remote sed -i "\"/^OS_UBUNTU = 'ubuntu'/a OS_NILRT = 'nilrt'\"" $f
  run_remote sed -i "\"/^OsDetect.register_default(OS_UBUNTU, LsbDetect(\\\"Ubuntu\\\"))/a OsDetect.register_default(OS_NILRT, FdoDetect('nilrt'))\"" $f

  # run_remote awk -i inplace "\"FNR==NR{ if (/OS_*/) p=NR; next} 1; FNR==p{ print \\\"OS_NILRT = 'nilrt'\\\" }; { print }\"" $f
  # run_remote awk -i inplace "\"FNR==NR{ if (/OsDetect.register_default.*/) p=NR; next} 1; FNR==p{ print \\\"OsDetect.register_default(OS_NILRT, FdoDetect('nilrt'))\\\" }; print\"" $f
fi


##### Source ROS #####
if ! run_remote grep "\"source /opt/ros/melodic/setup.bash\"" .bashrc > /dev/null; then
  append_remote "" .bashrc
  append_remote "\# Source ROS" .bashrc
  append_remote "source /opt/ros/melodic/setup.bash" .bashrc
  append_remote "export ROS_HOSTNAME=roborio-$1-frc.local" .bashrc
fi

# Make sure team number is right
run_remote sed -i "\"s/^export ROS_HOSTNAME=.*/export ROS_HOSTNAME=roborio-$1-frc.local/\"" .bashrc
