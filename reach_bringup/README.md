# Reach Bringup

This package provides launch files for the ROS 2 interfaces implemented for
each of the Reach Robotics devices.

## Reach Alpha 5 Launch

* alpha_5.launch.yaml

### Examples

* Launch the Reach Alpha 5 interface using *real* hardware

  ```bash
  ros2 launch reach_bringup alpha_5.launch.yaml use_mock_hardware:=false use_rviz:=true
  ```

* Launch the Reach Alpha 5 interface in Gazebo

  ```bash
  ros2 launch reach_bringup alpha_5.launch.yaml use_sim:=true
  ```

## Micro IP Camera Launch

* micro_ip.launch.py

### Examples

* Launch the Micro IP camera interface using the image_pipeline image
rectification and debayer nodes

  ```bash
  ros2 launch reach_bringup micro_ip.launch.py
  ```

> [!NOTE]
> We generally do not recommend using the Micro IP ROS 2 interface unless you
> need it for control purposes. The pipeline will flood the ROS 2 network with
> image messages and can introduce latency. If you are simply wanting to view
> the camera feed, we recommend using FFmpeg. If you want to view the feed from
> a topside device, we recommend configuring a [MediaMTX](https://github.com/bluenviron/mediamtx) proxy.
