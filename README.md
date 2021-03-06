# frc_control

[![Build Status](https://travis-ci.com/uwreact/frc_control.svg?branch=melodic-devel)](https://travis-ci.com/uwreact/frc_control)
[![License](https://img.shields.io/github/license/uwreact/frc_control.svg)](LICENSE)

A set of packages enabling FRC teams to use ROS on their robots.

The project primary consists of tools to interface [ros_control](http://wiki.ros.org/ros_control) with the [WPILib](https://github.com/wpilibsuite/allwpilib) tools required to develop FRC-legal robots, as well as compiling and running ROS on the NI roboRIO.
We also provide tools to enable running the robot in [Gazebo](http://gazebosim.org/) simulation.

Read the project's design goals in [DESIGN.md](DESIGN.md) for details on the direction and design of the project.

## Compiling

Full instructions for cross-compiling ROS for the roboRIO and linking the WPI libraries can be found in [INSTALLATION.md](INSTALLATION.md).

Once the workspace has been setup for cross-compilation, you can native or cross compile the code using:

```
catkin profile set <native|cross>
catkin build
```

## Code Style

We largely follow the ROS coding guidelines, with a few noteable exceptions. To make development easy, we provide configuration files for standard linting and static analysis tools such as [clang-format](https://clang.llvm.org/docs/ClangFormat.html), [clang-tidy](https://clang.llvm.org/extra/clang-tidy) and [yapf](https://github.com/google/yapf).

Install the linters:

```
sudo apt install clang-format-7 clang-tidy-7
pip install yapf pylint --user

# If ~/.local/bin is not on your path:
echo "PATH=\"$PATH:$HOME/.local/bin\"" >> ~/.bashrc
```

To run the formatters:

```
find . -name "*.h" -o -name "*.cpp" | xargs clang-format-7 -i -style=file
yapf -ir .
```

To run the linters:

```
./scripts/run_clang_tidy.py uwreact_robot
find . -iname "*.py" -o -iregex ".*/scripts/.*" | xargs pylint
```

## Contributing

We facilitate a completely open source environment for all of our projects, and are always welcoming contributors.

### Contributing Guide

Before opening your editor, read this project's [contributing guide](CONTRIBUTING.md) to learn about its development and contribution process.

### License

The `frc_control` project is [BSD 3-Clause licensed](LICENSE).
