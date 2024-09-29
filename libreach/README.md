# libreach

libreach is a C++ library designed to interface with [Reach Robotics](https://reachrobotics.com/)
devices. Get started with libreach by installing the project or by exploring
the implemented [examples](https://github.com/Robotic-Decision-Making-Lab/libreach/tree/main/examples).

> :warning: This project is not affiliated with or maintained by Reach Robotics.
> Please refer to the [Reach Robotics SDK](https://github.com/Reach-Robotics/reach_robotics_sdk/tree/master)
> for all official software.

## Installation

To install libreach, first clone the `ros2` branch of the repository to the
`src/` directory of your ROS 2 workspace

```bash
git clone -b ros2 git@github.com:Robotic-Decision-Making-Lab/libreach.git
```

Then install the project dependencies using rosdep

```bash
rosdep install --from paths src -y --ignore-src
```

Finally, build the package using colcon

```bash
colcon build --packages-select libreach
```

## Getting help

If you have questions regarding usage of libreach or regarding contributing to
this project, please ask a question on our [Discussions](https://github.com/Robotic-Decision-Making-Lab/libreach/discussions)
board.

## License

libreach is released under the Reach Robotics [software license](https://github.com/Reach-Robotics/reach_robotics_sdk/blob/master/LICENSE.txt).
