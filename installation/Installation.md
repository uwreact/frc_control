**Note** All the commands in this file are run in a command shell. We've therefore excluded the traditional `$` prompt on all lines to make copy-paste a bit easier.

# The basics - Setup ROS Kinetic and clone frc_control

### 1. Install ROS Kinetic. See the [wiki](http://wiki.ros.org/kinetic/Installation/Ubuntu) for more details

a) Add ROS to the apt sources list:

    sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'

    sudo apt-key adv --keyserver hkp://ha.pool.sks-keyservers.net:80 --recv-key 421C365BD9FF1F717815A3895523BAEEB01FA116

b) Install ROS

    sudo apt update
    sudo apt install ros-kinetic-desktop-full

c) Initialize rosdep

    sudo rosdep init
    rosdep update

d) Source the ROS environment

    echo "source /opt/ros/kinetic/setup.bash" >> ~/.bashrc

### 2. Install [catkin_tools](https://catkin-tools.readthedocs.io)
We recommend using `catkin_tools` rather than `catkin_make`, as it makes switching between native and cross compilation easier.
The instructions and scripts discussed below are all for `catkin_tools`.

    sudo apt install python-catkin-tools

### 3. Create a workspace, for example `~/robot_workspace/`

    mkdir -p ~/robot_workspace/src
    cd ~/robot_workspace
    catkin init

### 4. Clone the repository from GitHub into `<workspace>/src/`

    cd ~/robot_workspace/src
    git clone https://github.com/uwreact/frc_control.git

# Setup cross-compilation

This is a tricky procedure, and these instructions aren't guaranteed to work forever. However, the general procedure will largely remain the same. See [the ROS wiki](http://wiki.ros.org/kinetic/Installation/Source) for more information.

### 1. Install the FRC Toolchain

    sudo add-apt-repository ppa:wpilib/toolchain
    sudo apt update
    sudo apt install frc-toolchain

### 2. (Hopefully temporary) Install the Eclipse plugins

Hopefully this will not be needed once the VS Code integration is more complete, but for now, we must either compile the wpilib from source or pull the existing binaries out of the Eclipse plugins. This procedure below basically mimics the installation procedure the Eclipse plugins perform to install the necessary libraries and tools.

     mkdir ~/wpilib
     cd wpilib
     wget http://first.wpi.edu/FRC/roborio/release/EclipsePluginsV2018.4.1.zip
     unzip EclipsePluginsV2018.4.1.zip
     cd eclipse/plugins

     # Install common (eg libraries, etc)
     mkdir ~/wpilib/common
     unzip edu.wpi.first.wpilib.plugins.core_2018.4.1.jar resources/common.zip resources/tools.zip
     unzip resources/common.zip -d ~/wpilib/common/current
     unzip resources/tools.zip -d ~/wpilib/tools

     # Install cpp stuff
     mkdir ~/wpilib/cpp
     unzip edu.wpi.first.wpilib.plugins.cpp_2018.4.1.jar resources/cpp.zip
     unzip resources/cpp.zip -d ~/wpilib/cpp/current

     cd ~/wpilib
     rm -rf EclipsePluginsV2018.4.1.zip eclipse

### 3. Create a workspace for ROS source

    mkdir ~/ros_arm_cross_ws
    cd ~/ros_arm_cross_ws

### 4. Select the packages to install

This list of packages ~~may~~**will** change as time goes on. Right now, the standard robot set of packages contains many unneeded packages and adds a lot of unnecessary dependencies (I'm lookin at you, Collada). Once development of this project proceeds further, we can further customize this as required. In fact, we might consider only installing the dependencies of frc_control with no additional packages. This does mean that teams who wish to run their whole ROS stack on the RIO (Instead of the recommended config; frc_control on the RIO, everything else on other machines) will be required to either manually install their extra dependencies, or clone them into their workspace. Needs some consideration.

    sudo apt install python-rosinstall-generator python-wstool
    rosinstall_generator robot ros_control realtime_tools --rosdistro kinetic --deps --wet-only --tar > kinetic-roborio-wet.rosinstall #Install robot plus any other dependencies
    wstool init -j8 src kinetic-roborio-wet.rosinstall

### 5. Manually resolve dependencies

When possible, we install ipks that NI has distributed for the RoboRIO. If these aren't available, we instead compile libraries from source.

    cd ~/robot_workspace/src/frc_control
    sudo ./installation/install_cross_deps_ipks.bash
    sudo ./installation/install_cross_deps_src.bash

### 6. Build ROS with the FRC toolchain

    cd ~/ros_arm_cross_ws
    sudo rm -rf /usr/arm-frc-linux-gnueabi/opt/ros/kinetic devel_isolated build_isolated
    sudo ./src/catkin/bin/catkin_make_isolated --install -DCMAKE_TOOLCHAIN_FILE=~/robot_workspace/rostoolchain.cmake -DCMAKE_INSTALL_PREFIX=/usr/arm-frc-linux-gnueabi/opt/ros/kinetic -DCMAKE_MODULE_PATH=/usr/arm-frc-linux-gnueabi/usr/share/cmake/Modules

# Configuring and compiling frc_control

### 1. Run the installer script to set up catkin profiles for native and cross compilation

    cd ~/robot_workspace
    ./src/frc_control/setup_catkin.bash .

### 2. To perform a native compilation:

    catkin profile set native
    catkin build

### 3. To perform a cross compilation:

    catkin profile set cross
    catkin build
