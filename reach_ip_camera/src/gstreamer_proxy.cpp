// Copyright 2024, Evan Palmer
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

#include "reach_ip_camera/gstreamer_proxy.hpp"

#include <gst/app/app.h>

namespace reach
{

namespace
{

/// Convert a GStreamer sample to a cv::Mat image
auto gst_to_cv_mat(GstSample * sample) -> cv::Mat
{
  GstCaps * caps = gst_sample_get_caps(sample);
  GstBuffer * buffer = gst_sample_get_buffer(sample);
  GstStructure * structure = gst_caps_get_structure(caps, 0);

  int width;
  int height;
  gst_structure_get_int(structure, "width", &width);
  gst_structure_get_int(structure, "height", &height);

  GstMapInfo map;
  gst_buffer_map(buffer, &map, GST_MAP_READ);

  cv::Mat image(height, width, CV_8UC3, map.data);
  gst_buffer_unmap(buffer, &map);

  return image;
}

/// Convert a cv::Mat to a sensor_msgs::msg::Image
auto cv_mat_to_ros_image(const cv::Mat & image) -> sensor_msgs::msg::Image
{
  cv_bridge::CvImage cv_image;
  cv_image.image = image;
  cv_image.encoding = "bgr8";
  return *cv_image.toImageMsg();
}

}  // namespace

GStreamerProxy::GStreamerProxy(const rclcpp::NodeOptions & options)
: Node("gstreamer_proxy", options)
{
  try {
    param_listener_ = std::make_shared<gstreamer_proxy::ParamListener>(get_node_parameters_interface());
    params_ = param_listener_->get_params();
  }
  catch (const std::exception & e) {
    RCLCPP_ERROR(get_logger(), "Failed to get GStreamerProxy parameters: %s", e.what());  // NOLINT
    rclcpp::shutdown();
    return;
  }

  camera_info_ = std::make_unique<sensor_msgs::msg::CameraInfo>();
  camera_info_manager_ =
    std::make_shared<camera_info_manager::CameraInfoManager>(this, params_.camera_name, params_.camera_info_url);

  if (!camera_info_manager_->isCalibrated()) {
    RCLCPP_WARN(get_logger(), "%s is not calibrated", params_.camera_name.c_str());  // NOLINT
    camera_info_->header.frame_id = params_.frame_id;
    camera_info_->width = params_.image_width;
    camera_info_->height = params_.image_height;
    camera_info_manager_->setCameraInfo(*camera_info_);
  }

  camera_pub_ = std::make_shared<image_transport::CameraPublisher>(
    image_transport::create_camera_publisher(this, "image_raw", rclcpp::QoS{10}.get_rmw_qos_profile()));

  if (!configure_stream()) {
    shutdown_stream();
    rclcpp::shutdown();
    return;
  }
}

GStreamerProxy::~GStreamerProxy() { shutdown_stream(); }

auto GStreamerProxy::configure_stream() -> bool
{
  gst_init(nullptr, nullptr);

  RCLCPP_INFO(get_logger(), "Starting GStreamer pipeline with address: %s", params_.stream_address.c_str());  // NOLINT

  // The following configurations are intended for low-latency transport
  const std::string video_source = "rtspsrc location=" + params_.stream_address + " protocols=tcp";
  const std::string video_codec = "! rtph264depay ! h264parse ! avdec_h264";
  const std::string video_decode = "! decodebin ! videoconvert ! video/x-raw,format=(string)BGR";
  const std::string video_sink_conf = "! queue ! appsink emit-signals=true sync=false max-buffers=1 drop=true";
  const std::string command = video_source + " " + video_codec + " " + video_decode + " " + video_sink_conf;

  // Create GStreamer pipeline
  GError * error = nullptr;
  pipeline_ = gst_parse_launch(command.c_str(), &error);

  if (error != nullptr) {
    RCLCPP_ERROR(get_logger(), "Failed to create GStreamer pipeline. %s", error->message);  // NOLINT
    g_error_free(error);
    return false;
  }

  // Configure appsink
  GstElement * sink = gst_bin_get_by_name(GST_BIN(pipeline_), "appsink0");  // NOLINT(bugprone-casting-through-void)

  if (sink == nullptr) {
    RCLCPP_ERROR(get_logger(), "Failed to get appsink from the pipeline");
    return false;
  }

  gst_app_sink_set_emit_signals(GST_APP_SINK_CAST(sink), static_cast<gboolean>(true));
  gst_app_sink_set_drop(GST_APP_SINK_CAST(sink), static_cast<gboolean>(true));
  gst_app_sink_set_max_buffers(GST_APP_SINK_CAST(sink), 1);

  // Set the callbacks for the appsink
  GstAppSinkCallbacks callbacks = {
    nullptr,                                                                                     // eos
    [](GstAppSink * /*sink*/, gpointer /*user_data*/) -> GstFlowReturn { return GST_FLOW_OK; },  // new_preroll
    [](GstAppSink * sink, gpointer user_data) -> GstFlowReturn {                                 // new_sample
      auto * self = reinterpret_cast<GStreamerProxy *>(user_data);

      GstSample * sample = gst_app_sink_pull_sample(GST_APP_SINK_CAST(sink));
      const cv::Mat image = gst_to_cv_mat(sample);

      *self->camera_info_ = self->camera_info_manager_->getCameraInfo();
      self->camera_info_->header.stamp = self->now();

      sensor_msgs::msg::Image image_msg = cv_mat_to_ros_image(image);
      image_msg.header = self->camera_info_->header;

      self->camera_pub_->publish(image_msg, *self->camera_info_);

      gst_sample_unref(sample);

      return GST_FLOW_OK;
    },
    nullptr,  // new_event
    nullptr,  // propose_allocation
    nullptr};
  gst_app_sink_set_callbacks(GST_APP_SINK_CAST(sink), &callbacks, this, nullptr);

  // Set the callback for the bus; this lets us handle errors
  GstBus * bus;
  bus = gst_pipeline_get_bus(GST_PIPELINE(pipeline_));  // NOLINT(bugprone-casting-through-void)
  gst_bus_add_watch(
    bus,
    [](GstBus * /*bus*/, GstMessage * message, gpointer user_data) -> gboolean {
      if (GST_MESSAGE_TYPE(message) == GST_MESSAGE_ERROR) {
        auto * self = reinterpret_cast<GStreamerProxy *>(user_data);
        GError * error;
        gchar * debug;
        gst_message_parse_error(message, &error, &debug);
        RCLCPP_ERROR(self->get_logger(), "Error: %s", error->message);  // NOLINT
        g_error_free(error);
        g_free(debug);
      }
      return static_cast<gboolean>(true);
    },
    this);
  gst_object_unref(bus);

  const GstStateChangeReturn ret = gst_element_set_state(pipeline_, GST_STATE_PLAYING);
  if (ret == GST_STATE_CHANGE_FAILURE) {
    RCLCPP_ERROR(get_logger(), "Failed to start GStreamer pipeline");
    return false;
  }

  return true;
}

auto GStreamerProxy::shutdown_stream() -> void
{
  gst_element_set_state(pipeline_, GST_STATE_NULL);
  gst_object_unref(pipeline_);
}

}  // namespace reach

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(reach::GStreamerProxy)
