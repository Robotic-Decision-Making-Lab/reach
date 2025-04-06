# Reach Robotics Driver

Reach provides a C++ driver and ROS 2 interface for integrating Reach
Robotics devices, including the Reach Alpha 5 and Reach Bravo 7 manipulators.

## Main features

The main features of this project include:

- Integration of the Reach system communication protocol for hardware
  communication
- ros2_control integration for manipulator position, velocity, and torque
  control
- ROS 2 support for the Reach Robotics IP cameras
- Simulation support using Gazebo
- Integration of the dynamic parameters obtained by Reach Robotics

## Installation

Reach is currently supported on Linux, and is available for the ROS
distributions Jazzy and Rolling. To install the project, first clone this
project to the `src/` directory of your ROS 2 workspace, replacing `$ROS_DISTRO`
with the desired ROS 2 distribution or `main` for Rolling:

```bash
git clone -b $ROS_DISTRO git@github.com:Robotic-Decision-Making-Lab/reach.git
```

After cloning the project, install all ROS dependencies using rosdep, again,
replacing $ROS_DISTRO with the desired ROS distribution:

```bash
rosdep update && \
rosdep install -y --from-paths src --ignore-src --rosdistro $ROS_DISTRO
```

## Usage

Run the following command to use the Reach Alpha 5 interface:

```bash
ros2 launch reach_bringup alpha_5.launch.yaml
```

A full description of the available arguments can be retrieved using

```bash
ros2 launch reach_bringup <launch-file-name> --show-args
```

## Disclaimer

This is an independent project, and is not affiliated with or maintained by
Reach Robotics. Please refer to the [Reach Robotics SDK](https://github.com/Reach-Robotics/reach_robotics_sdk/tree/master)
for all official software.

## License

This project has been released under the [Reach Robotics Software License](https://github.com/Reach-Robotics/reach_robotics_sdk/tree/master?tab=readme-ov-file#license).
