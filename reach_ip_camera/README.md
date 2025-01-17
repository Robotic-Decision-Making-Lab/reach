# Micro IP Camera

This package provides configurations and a ROS 2 interface for streaming video
from the Reach Robotics [Micro IP cameras](https://reachrobotics.com/products/underwater-rov-camera/).
The implementation in this package is a port of our work on Barlus UW-S5-3PBX10
underwater camera [here](https://github.com/Robotic-Decision-Making-Lab/barlus_underwater_camera).

## Network Configuration

The default network configurations for the Micro IP camera are as follows:

```bash
Address: 192.168.2.10 # You may need to modify this for multi-arm configurations
Netmask: 255.255.255.0
Gateway: 0.0.0.0
```

In order to connect to the camera, add a new IPv4 network configuration using
the same sub-domain, netmask, and gateway as the camera. For example,

```bash
Address: 192.168.2.12
Netmask: 255.255.255.0
Gateway: 0.0.0.0
```

After establishing a connection, verify the RTSP stream using FFmpeg

```bash
ffplay -fflags nobuffer -flags low_delay -probesize 32 -rtsp_transport tcp -i "rtsp://admin:@192.168.2.10:554/stream=1"
```

The stream can also be recorded using

```bash
ffmpeg -i "rtsp://admin:@192.168.2.10:554/stream=1" -c:v copy -an recording.mp4
```

> [!NOTE]
> Reach Robotics recommends using UDP instead of TCP. In our experience, UDP
> tends to introduce artifacting without much performance gain so we use TCP.

## ROS 2 interface

A simple wrapper node has been implemented to convert frames received from the
camera into ROS 2 `sensor_msgs/Image` messages using GStreamer. After building
the `reach_ip_camera` package, the node can be launched with [image_pipeline](https://github.com/ros-perception/image_pipeline)
using

```bash
ros2 launch reach_ip_camera gstreamer_proxy.launch.py
```

## Camera calibration

The camera intrinsics can be retrieved using the [camera_calibration](https://docs.ros.org/en/rolling/p/camera_calibration/index.html)
node. For example, to calibrate the camera using an [8x6 chessboard](https://wiki.ros.org/camera_calibration/Tutorials/MonocularCalibration?action=AttachFile&do=view&target=check-108.pdf),
run the following:

 ```bash
 ros2 run camera_calibration cameracalibrator --size 8x6 --square 0.108 image:=/micro_ip/image_mono camera:=/micro_ip --no-service-check
```
