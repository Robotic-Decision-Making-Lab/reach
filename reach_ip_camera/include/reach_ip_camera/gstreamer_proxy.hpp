// Copyright 2025, Evan Palmer
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
// THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
// THE SOFTWARE.

#pragma once

#include <gst/gst.h>

#include <camera_info_manager/camera_info_manager.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include <image_transport/image_transport.hpp>
#include <memory>
#include <rclcpp/node.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>

// auto-generated by generate_parameter_library
#include <reach_ip_camera/gstreamer_proxy_parameters.hpp>

namespace reach
{

class GStreamerProxy : public rclcpp::Node
{
public:
  explicit GStreamerProxy(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

  ~GStreamerProxy();

private:
  auto configure_stream() -> bool;

  auto shutdown_stream() -> void;

  std::shared_ptr<gstreamer_proxy::ParamListener> param_listener_;
  gstreamer_proxy::Params params_;

  std::unique_ptr<sensor_msgs::msg::CameraInfo> camera_info_;
  std::shared_ptr<camera_info_manager::CameraInfoManager> camera_info_manager_;
  std::shared_ptr<image_transport::CameraPublisher> camera_pub_;

  GstElement * pipeline_;
};

}  // namespace reach
