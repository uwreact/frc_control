# frc_control

[![Build Status](https://travis-ci.com/uwreact/frc_control.svg?branch=melodic-devel)](https://travis-ci.com/uwreact/frc_control)

A set of packages enabling FRC teams to use ROS on their robots.

The project primary consists of tools to interface [ros_control](http://wiki.ros.org/ros_control) with the [WPILib](https://github.com/wpilibsuite/allwpilib) tools required to develop FRC-legal robots, as well as compiling and running ROS on the NI roboRIO.
We also provide tools to enable running the robot in [Gazebo](http://gazebosim.org/) simulation.

Read the project's design goals in [DESIGN.md](DESIGN.md) for details on the direction and design of the project.

## Compiling

Full instructions for cross-compiling ROS for the roboRIO and linking the WPI libraries can be found in [installation/INSTALLATION.md](installation/INSTALLATION.md).

Once the workspace has been setup for cross-compilation, you can native or cross compile the code using:

    $ catkin profile set <native|cross>
    $ catkin build

## Contributing

We facilitate a completely open source environment for all of our projects, and are always welcoming contributors.

### Contributing Guide

Before opening your editor, read this project's [contributing guide](CONTRIBUTING.md) to learn about its development and contribution process.

### License

The `frc_control` project is [BSD 3-Clause licensed](LICENSE).
