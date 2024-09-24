// Copyright (c) 2024 Evan Palmer
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
#include "libreach/udp_driver.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/state.hpp"

namespace reach::hardware
{

class Bravo7Hardware : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(Bravo7Hardware)  // NOLINT

  auto on_init(const hardware_interface::HardwareInfo & info) -> hardware_interface::CallbackReturn override;

  auto on_configure(const rclcpp_lifecycle::State & previous_state) -> hardware_interface::CallbackReturn override;

  auto prepare_command_mode_switch(
    const std::vector<std::string> & start_interfaces,
    const std::vector<std::string> & stop_interfaces) -> hardware_interface::return_type override;

  auto perform_command_mode_switch(
    const std::vector<std::string> & start_interfaces,
    const std::vector<std::string> & stop_interfaces) -> hardware_interface::return_type override;

  auto export_state_interfaces() -> std::vector<hardware_interface::StateInterface> override;

  auto export_command_interfaces() -> std::vector<hardware_interface::CommandInterface> override;

  auto on_activate(const rclcpp_lifecycle::State & previous_state) -> hardware_interface::CallbackReturn override;

  auto on_deactivate(const rclcpp_lifecycle::State & previous_state) -> hardware_interface::CallbackReturn override;

  auto read(const rclcpp::Time & time, const rclcpp::Duration & period) -> hardware_interface::return_type override;

  auto write(const rclcpp::Time & time, const rclcpp::Duration & period) -> hardware_interface::return_type override;

private:
  std::unique_ptr<libreach::UdpDriver> bravo_;

  // ROS parameters
  std::string ip_address_;
  std::uint16_t port_;
  std::chrono::milliseconds state_request_rate_;

  // command interfaces
  std::vector<double> hw_commands_velocities_, hw_commands_positions_, hw_commands_torques_;

  // state interfaces
  std::vector<double> async_states_velocities_, async_states_positions_, async_states_torques_;
  std::vector<double> hw_states_velocities_, hw_states_positions_, hw_states_torques_;
  std::mutex position_state_lock_, velocity_state_lock_, torque_state_lock_;

  // Device operating modes
  std::vector<libreach::Mode> control_modes_;

  rclcpp::Logger logger_{rclcpp::get_logger("bravo_7_hardware")};
};

}  // namespace reach::hardware
