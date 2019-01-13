**Note** All the commands in this file are run in a command shell. We've therefore excluded the traditional `$` prompt on all lines to make copy-paste a bit easier.

# The basics - Setup ROS Kinetic and clone frc_control

## 1. Install ROS Kinetic. See the [wiki](http://wiki.ros.org/kinetic/Installation/Ubuntu) for more details

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

## 2. Install [catkin_tools](https://catkin-tools.readthedocs.io)

We recommend using `catkin_tools` rather than `catkin_make`, as it makes switching between native and cross compilation easier.
The instructions and scripts discussed below are all for `catkin_tools`.

    sudo apt install python-catkin-tools

## 3. Create a workspace, for example `~/robot_workspace/`

    mkdir -p ~/robot_workspace/src
    cd ~/robot_workspace
    catkin init

## 4. Clone the repository from GitHub into `<workspace>/src/`

    cd ~/robot_workspace/src
    git clone https://github.com/uwreact/frc_control.git

# Setup cross-compilation

This is a tricky procedure, and these instructions aren't guaranteed to work forever. However, the general procedure will largely remain the same. See [the ROS wiki](http://wiki.ros.org/kinetic/Installation/Source) for more information.

## 1. Install the WPILib and associated tools
Follow the instructions on [the ScreenStepsLive](https://wpilib.screenstepslive.com/s/currentCS/m/getting_started/l/999999-installing-c-and-java-development-tools-for-frc). These have been summarized below:

### 1.1 Download and extract the latest WPILib release

Download the latest Linux release from [the WPILib GitHub](https://github.com/wpilibsuite/allwpilib/releases) (At the time of writing, the latest is [2019.1.1](https://github.com/wpilibsuite/allwpilib/releases/download/v2019.1.1/WPILib_Linux-2019.1.1.tar.gz)). Extract the archive to `~/frc2019`.

    wget https://github.com/wpilibsuite/allwpilib/releases/download/v2019.1.1/WPILib_Linux-2019.1.1.tar.gz

    mkdir ~/frc2019
    tar xzf WPILib_Linux-2019.1.1.tar.gz -C ~/frc2019
    rm WPILib_Linux-2019.1.1.tar.gz

### 1.2 Install the VSCode extensions

**TODO**: Not sure which, if any, of the `Java*.vsix` extensions are required. For now, install everything. Once we know, unnecessary extensions will be removed from these instructions.

    cd ~/frc2019/vsCodeExtensions
    code --install-extension Cpp.vsix
    code --install-extension JavaLang.vsix
    code --install-extension JavaDeps.vsix
    code --install-extension JavaDebug.vsix
    code --install-extension WPILib.vsix

### 1.3 Set up VSCode to use Java 11

Press `ctrl+shift+P` to open the Command Palette. Run the `Set VS Code Java Home to FRC Home` command.

## 2. Create a workspace for ROS source

    mkdir ~/ros_arm_cross_ws
    cd ~/ros_arm_cross_ws

## 3. Select the packages to install

This list of packages ~~may~~**will** change as time goes on. Right now, the standard robot set of packages contains many unneeded packages and adds a lot of unnecessary dependencies (I'm lookin at you, Collada). Once development of this project proceeds further, we can further customize this as required. In fact, we might consider only installing the dependencies of frc_control with no additional packages. This does mean that teams who wish to run their whole ROS stack on the RIO (Instead of the recommended config; frc_control on the RIO, everything else on other machines) will be required to either manually install their extra dependencies, or clone them into their workspace. Needs some consideration.

    sudo apt install python-rosinstall-generator python-wstool
    rosinstall_generator robot ros_control realtime_tools --rosdistro kinetic --deps --wet-only --tar > kinetic-roborio-wet.rosinstall #Install robot plus any other dependencies
    wstool init -j8 src kinetic-roborio-wet.rosinstall

## 4. Manually resolve dependencies

When possible, we install ipks that NI has distributed for the RoboRIO. If these aren't available, we instead compile libraries from source.

    cd ~/robot_workspace/src/frc_control
    sudo ./installation/install_cross_deps_ipks.bash
    sudo ./installation/install_cross_deps_src.bash

## 5. Build ROS with the FRC toolchain

    cd ~/ros_arm_cross_ws
    sudo rm -rf /usr/arm-frc-linux-gnueabi/opt/ros/kinetic devel_isolated build_isolated
    sudo ./src/catkin/bin/catkin_make_isolated --install -DCMAKE_TOOLCHAIN_FILE=~/robot_workspace/rostoolchain.cmake -DCMAKE_INSTALL_PREFIX=/usr/arm-frc-linux-gnueabi/opt/ros/kinetic -DCMAKE_MODULE_PATH=/usr/arm-frc-linux-gnueabi/usr/share/cmake/Modules

# Configuring and compiling frc_control

## 1. Run the installer script to set up catkin profiles for native and cross compilation

    cd ~/robot_workspace
    ./src/frc_control/setup_catkin.bash .

## 2. To perform a native compilation:

    catkin build --profile native

## 3. To perform a cross compilation:

    catkin build --profile cross

## 4. Enable 3rd party libraries

frc_control has built-in support for the most common 3rd party libraries; [CTRE Toolsuite](http://www.ctr-electronics.com/control-system/hro.html#product_tabs_technical_resources), [Kauai Labs](https://pdocs.kauailabs.com/navx-mxp/software/), and [Mindsensors](http://www.mindsensors.com/blog/how-to/how-to-use-sd540c-and-canlight-with-roborio). However, since we know not all teams will be using all of these libraries, they are all **disabled** by default. To download and enable these libraries, use the `install_3rd_party_libs.bash` script.

    cd ~/robot_workspace/src/frc_control/installation
    ./install_3rd_party_libs.bash < --ctre | --kauai | --mindsensors | --all >
