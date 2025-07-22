// Copyright (c) 2025 Evan Palmer
//
// Permission is hereby granted, free of charge, to any person obtaining a copy of
// this software and associated documentation files (the "Software"), limited
// exclusively to use with products produced by Reach Robotics Pty Ltd, subject to
// the following conditions:
//
// The Software may only be used in conjunction with products manufactured or
// developed by Reach Robotics Pty Ltd.
//
// Redistributions or use of the Software in any other context, including but
// not limited to, integration, combination, or use with other products or
// software, are strictly prohibited without prior written authorization from Reach
// Robotics Pty Ltd.
//
// All copies of the Software, in whole or in part, must retain this notice and
// the above copyright notice.
//
// THIS SOFTWARE IS PROVIDED "AS IS," WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS
// FOR A PARTICULAR PURPOSE, AND NONINFRINGEMENT. IN NO EVENT SHALL REACH ROBOTICS
// PTY LTD BE LIABLE FOR ANY CLAIM, DAMAGES, OR OTHER LIABILITY, WHETHER IN AN
// ACTION OF CONTRACT, TORT, OR OTHERWISE, ARISING FROM, OUT OF, OR IN CONNECTION
// WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.

#pragma once

#include <chrono>
#include <cstdint>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "libreach/device_id.hpp"
#include "libreach/mode.hpp"
#include "libreach/packet.hpp"
#include "libreach/packet_id.hpp"
#include "libreach/serial_driver.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/state.hpp"

namespace reach::hardware
{

class Alpha5Hardware : public hardware_interface::SystemInterface
{
public:
  auto on_init(const hardware_interface::HardwareComponentInterfaceParams & info)
    -> hardware_interface::CallbackReturn override;

  auto on_configure(const rclcpp_lifecycle::State & previous_state) -> hardware_interface::CallbackReturn override;

  auto prepare_command_mode_switch(
    const std::vector<std::string> & start_interfaces,
    const std::vector<std::string> & stop_interfaces) -> hardware_interface::return_type override;

  auto perform_command_mode_switch(
    const std::vector<std::string> & start_interfaces,
    const std::vector<std::string> & stop_interfaces) -> hardware_interface::return_type override;

  auto on_activate(const rclcpp_lifecycle::State & previous_state) -> hardware_interface::CallbackReturn override;

  auto on_deactivate(const rclcpp_lifecycle::State & previous_state) -> hardware_interface::CallbackReturn override;

  auto read(const rclcpp::Time & time, const rclcpp::Duration & period) -> hardware_interface::return_type override;

  auto write(const rclcpp::Time & time, const rclcpp::Duration & period) -> hardware_interface::return_type override;

private:
  std::unique_ptr<libreach::SerialDriver> alpha_;

  // ROS parameters
  std::string serial_port_;
  std::chrono::milliseconds state_request_rate_;

  // RT state interfaces
  // We maintain separate maps for each state interface to take advantage of the libreach multi-threaded callbacks
  std::unordered_map<std::string, float> async_states_positions_, async_states_velocities_, async_states_efforts_;
  std::mutex position_state_lock_, velocity_state_lock_, effort_state_lock_;

  // Keep track of the mapping between joint names and device IDs to speed up state/control updates
  std::unordered_map<std::uint8_t, std::string> device_ids_to_names_;
  std::unordered_map<std::string, std::uint8_t> names_to_device_ids_;

  // Device operating modes
  std::unordered_map<std::string, libreach::Mode> control_modes_;

  rclcpp::Logger logger_{rclcpp::get_logger("alpha_5_hardware")};
};

}  // namespace reach::hardware
