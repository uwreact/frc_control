# frc_control

The `frc_control` project is a ROS metapackage enabling FRC teams to use ROS on their robots.

The project primary consists of tools to interface [ros_control](http://wiki.ros.org/ros_control) with the WPILib tools required to develop FRC-legal robots.
We also provide tools to enable running the robot in [Gazebo](http://gazebosim.org/) simulation.

Read the project's [full description](https://github.com/uwreact/frc_control/blob/kinetic-devel/DESIGN.md) for details on the direction and design of the project.

## Compiling

We recommend using [catkin_tools](https://catkin-tools.readthedocs.io) rather than `catkin_make`.
The instructions and scripts discussed below are all for `catkin_tools`.

Full instructions for cross-compiling ROS for the roboRIO and linking the WPI libraries are currently in development.
For the time being, the [setup_catkin.bash](https://github.com/uwreact/frc_control/blob/kinetic-devel/setup_catkin.bash) script can be used to setup the catkin profiles for native and cross-compilation.
Run `setup_catkin.bash` with the path to your ROS workspace, for example:

    $ ./setup_catkin.bash ~/catkin_ws

Once the workspace has been setup for cross-compilation, you can switch between native and cross compilation profiles using:

    $ catkin profile set <native|cross>

## Contributing

We facilitate a completely open source environment for all of our projects, and are always welcoming contributors.

### Contributing Guide

Before opening your editor, read this project's [contributing guide](https://github.com/uwreact/frc_control/blob/kinetic-devel/CONTRIBUTING.md) to learn about its development and contribution process.

### License

The `frc_control` project is [BSD 3-Clause licensed](https://github.com/uwreact/frc_control/blob/kinetic-devel/LICENSE).
