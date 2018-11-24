# frc_control

The `frc_control` project is a ROS metapackage enabling FRC teams to use ROS on their robots.

The project primary consists of tools to interface [ros_control](http://wiki.ros.org/ros_control) with the WPILib tools required to develop FRC-legal robots.
We also provide tools to enable running the robot in [Gazebo](http://gazebosim.org/) simulation.

Read the project's design goals in [DESIGN.md](https://github.com/uwreact/frc_control/blob/kinetic-devel/DESIGN.md) for details on the direction and design of the project.

## Compiling

Full instructions for cross-compiling ROS for the roboRIO and linking the WPI libraries can be found in [installation/installation.md](https://github.com/uwreact/frc_control/blob/kinetic-devel/installation/installation.md).

Once the workspace has been setup for cross-compilation, you can native or cross compile the code using:

    $ catkin profile set <native|cross>
    $ catkin build

## Contributing

We facilitate a completely open source environment for all of our projects, and are always welcoming contributors.

### Contributing Guide

Before opening your editor, read this project's [contributing guide](https://github.com/uwreact/frc_control/blob/kinetic-devel/CONTRIBUTING.md) to learn about its development and contribution process.

### License

The `frc_control` project is [BSD 3-Clause licensed](https://github.com/uwreact/frc_control/blob/kinetic-devel/LICENSE).
