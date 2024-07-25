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

#include "bravo_hardware/hardware.hpp"

#include <limits>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

namespace bravo::hardware
{

namespace
{

// TODO(evan-palmer): Get these values
const std::array<float, 7> TORQUE_CONSTANTS{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

/// Convert a torque (Nm) to current (mA) given the torque constant (Nm/A) and gear ratio
auto convert_torque_to_current(float t, float Kt, float Gr = 120.0) -> float { return t / Kt / Gr * 1000.0; }

/// Convert a current (mA) to torque (Nm) given the torque constant (Nm/A) and gear ratio
auto convert_current_to_torque(float c, float Kt, float Gr = 120.0) -> float { return c * Kt * Gr / 1000.0; }

}  // namespace

auto BravoHardware::on_init(const hardware_interface::HardwareInfo & info) -> hardware_interface::CallbackReturn
{
  RCLCPP_INFO(logger_, "Initializing BravoHardware system interface");  // NOLINT

  if (hardware_interface::SystemInterface::on_init(info) != hardware_interface::CallbackReturn::SUCCESS) {
    RCLCPP_INFO(logger_, "Failed to initialize BravoHardware system interface");  // NOLINT
    return hardware_interface::CallbackReturn::ERROR;
  }

  // Initialize the state vectors with NaNs
  hw_states_positions_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  hw_states_velocities_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  hw_states_torques_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());

  // Initialize the async state vectors with NaNs
  async_states_positions_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  async_states_velocities_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  async_states_torques_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());

  // Initialize the command vectors with NaNs
  hw_commands_positions_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  hw_commands_velocities_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  hw_commands_torques_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());

  // Set the default control mode
  control_modes_.resize(info_.joints.size(), libreach::Mode::STANDBY);

  // Load the ROS params
  ip_address_ = info_.hardware_parameters["ip_address"];
  port_ = std::stoi(info_.hardware_parameters["port"]);
  state_request_rate_ = std::chrono::milliseconds(std::stoi(info_.hardware_parameters["state_request_rate"]));

  // Check the state/command interfaces
  for (const hardware_interface::ComponentInfo & joint : info_.joints) {
    if (joint.command_interfaces.size() != 3) {
      RCLCPP_ERROR(  // NOLINT
        logger_,
        "Joint '%s' has %zu command interfaces, expected 3",
        joint.name.c_str(),
        joint.command_interfaces.size());
      return hardware_interface::CallbackReturn::ERROR;
    }

    const bool command_interface_valid = joint.command_interfaces.front().name == hardware_interface::HW_IF_POSITION ||
                                         joint.command_interfaces.front().name == hardware_interface::HW_IF_VELOCITY ||
                                         joint.command_interfaces.front().name == hardware_interface::HW_IF_EFFORT;

    if (!command_interface_valid) {
      RCLCPP_ERROR(  // NOLINT
        logger_,
        "Joint '%s' has command interface '%s', expected '%s', '%s', or '%s'",
        joint.name.c_str(),
        joint.command_interfaces.front().name.c_str(),
        hardware_interface::HW_IF_POSITION,
        hardware_interface::HW_IF_VELOCITY,
        hardware_interface::HW_IF_EFFORT);
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.state_interfaces.size() != 3) {
      RCLCPP_ERROR(  // NOLINT
        logger_,
        "Joint '%s' has %zu state interfaces, expected 3",
        joint.name.c_str(),
        joint.state_interfaces.size());
      return hardware_interface::CallbackReturn::ERROR;
    }

    const bool state_interface_valid = joint.state_interfaces.front().name == hardware_interface::HW_IF_POSITION ||
                                       joint.state_interfaces.front().name == hardware_interface::HW_IF_VELOCITY ||
                                       joint.state_interfaces.front().name == hardware_interface::HW_IF_EFFORT;

    if (!state_interface_valid) {
      RCLCPP_ERROR(  // NOLINT
        logger_,
        "Joint '%s' has state interface '%s', expected '%s', '%s', or '%s'",
        joint.name.c_str(),
        joint.state_interfaces.front().name.c_str(),
        hardware_interface::HW_IF_POSITION,
        hardware_interface::HW_IF_VELOCITY,
        hardware_interface::HW_IF_EFFORT);
      return hardware_interface::CallbackReturn::ERROR;
    }
  }

  RCLCPP_INFO(logger_, "Successfully initialized BravoHardware system interface");  // NOLINT

  return hardware_interface::CallbackReturn::SUCCESS;
}

auto BravoHardware::on_configure(const rclcpp_lifecycle::State & /* previous_state */)
  -> hardware_interface::CallbackReturn
{
  // Use a multi-worker configuration
  const std::size_t buffer_size = 100;
  const std::size_t num_workers = 5;

  try {
    bravo_ = std::make_unique<libreach::UdpDriver>(ip_address_, port_, buffer_size, num_workers);
  }
  catch (const std::exception & e) {
    RCLCPP_ERROR(logger_, "Failed to configure UDP driver: %s", e.what());  // NOLINT
    return hardware_interface::CallbackReturn::ERROR;
  }

  // Register the state callbacks
  bravo_->register_callback(libreach::PacketId::POSITION, [this](const libreach::Packet & packet) {
    const auto idx = static_cast<std::size_t>(packet.device_id()) - 1;
    const auto position = static_cast<double>(libreach::deserialize<float>(packet));
    const std::lock_guard<std::mutex> lock(position_state_lock_);
    async_states_positions_[idx] = position;
  });

  bravo_->register_callback(libreach::PacketId::VELOCITY, [this](const libreach::Packet & packet) {
    const auto idx = static_cast<std::size_t>(packet.device_id()) - 1;
    const auto velocity = static_cast<double>(libreach::deserialize<float>(packet));
    const std::lock_guard<std::mutex> lock(velocity_state_lock_);
    async_states_velocities_[idx] = velocity;
  });

  bravo_->register_callback(libreach::PacketId::CURRENT, [this](const libreach::Packet & packet) {
    const auto idx = static_cast<std::size_t>(packet.device_id()) - 1;
    const float torque = convert_current_to_torque(libreach::deserialize<float>(packet), TORQUE_CONSTANTS[idx]);
    const std::lock_guard<std::mutex> lock(torque_state_lock_);
    async_states_torques_[idx] = static_cast<double>(torque);
  });

  // Configure the state requests
  for (std::size_t i = 1; i < hw_states_positions_.size() + 1; ++i) {
    bravo_->request_at_rate(
      {libreach::PacketId::POSITION, libreach::PacketId::VELOCITY, libreach::PacketId::CURRENT},
      i,
      state_request_rate_);
  }

  RCLCPP_INFO(logger_, "Successfully configured the BravoHardware system interface");  // NOLINT

  return hardware_interface::CallbackReturn::SUCCESS;
}

auto BravoHardware::on_cleanup(const rclcpp_lifecycle::State & /* previous_state */)
  -> hardware_interface::CallbackReturn
{
  return hardware_interface::CallbackReturn::SUCCESS;
}

auto BravoHardware::prepare_command_mode_switch(
  const std::vector<std::string> & start_interfaces,
  const std::vector<std::string> & /* stop_interfaces */) -> hardware_interface::return_type
{
  std::vector<libreach::Mode> new_modes(control_modes_.size());

  for (const auto & key : start_interfaces) {
    for (auto & joint : info_.joints) {
      if (key == joint.name + "/" + hardware_interface::HW_IF_POSITION) {
        new_modes.emplace_back(libreach::Mode::POSITION);
      } else if (key == joint.name + "/" + hardware_interface::HW_IF_VELOCITY) {
        new_modes.emplace_back(libreach::Mode::VELOCITY);
      } else if (key == joint.name + "/" + hardware_interface::HW_IF_EFFORT) {
        new_modes.emplace_back(libreach::Mode::CURRENT);
      } else {
        RCLCPP_WARN(  // NOLINT
          logger_,
          "Unknown command mode for joint '%s': '%s'. Setting mode to 'STANDBY'",
          joint.name.c_str(),
          key.c_str());
        new_modes.emplace_back(libreach::Mode::STANDBY);
      }
    }
  }

  if (!new_modes.empty() && new_modes.size() != info_.joints.size()) {
    return hardware_interface::return_type::ERROR;
  }

  control_modes_ = new_modes;

  return hardware_interface::return_type::OK;
}

auto BravoHardware::perform_command_mode_switch(
  const std::vector<std::string> & /* start_interfaces */,
  const std::vector<std::string> & /* stop_interfaces */) -> hardware_interface::return_type
{
  // Stop all joints
  bravo_->set_velocity(static_cast<std::uint8_t>(libreach::Bravo7DeviceId::ALL_JOINTS), 0.0);

  // Perform the mode switch
  for (std::size_t i = 0; i < info_.joints.size(); ++i) {
    bravo_->set_mode(static_cast<std::uint8_t>(i + 1), control_modes_[i]);
  }

  return hardware_interface::return_type::OK;
}

auto BravoHardware::export_state_interfaces() -> std::vector<hardware_interface::StateInterface>
{
  std::vector<hardware_interface::StateInterface> state_interfaces;

  for (std::size_t i = 0; i < info_.joints.size(); ++i) {
    state_interfaces.emplace_back(info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_states_positions_[i]);
    state_interfaces.emplace_back(info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_states_velocities_[i]);
    state_interfaces.emplace_back(info_.joints[i].name, hardware_interface::HW_IF_EFFORT, &hw_states_torques_[i]);
  }

  return state_interfaces;
}

auto BravoHardware::export_command_interfaces() -> std::vector<hardware_interface::CommandInterface>
{
  std::vector<hardware_interface::CommandInterface> cmd_interfaces;

  for (std::size_t i = 0; i < info_.joints.size(); ++i) {
    cmd_interfaces.emplace_back(info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_commands_positions_[i]);
    cmd_interfaces.emplace_back(info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_commands_velocities_[i]);
    cmd_interfaces.emplace_back(info_.joints[i].name, hardware_interface::HW_IF_EFFORT, &hw_commands_torques_[i]);
  }

  return cmd_interfaces;
}

auto BravoHardware::on_activate(const rclcpp_lifecycle::State & /* previous_state */)
  -> hardware_interface::CallbackReturn
{
  bravo_->set_mode(static_cast<std::uint8_t>(libreach::Bravo7DeviceId::ALL_JOINTS), libreach::Mode::STANDBY);
  return hardware_interface::CallbackReturn::SUCCESS;
}

auto BravoHardware::on_deactivate(const rclcpp_lifecycle::State & /* previous_state */)
  -> hardware_interface::CallbackReturn
{
  bravo_->set_mode(static_cast<std::uint8_t>(libreach::Bravo7DeviceId::ALL_JOINTS), libreach::Mode::DISABLE);
  return hardware_interface::CallbackReturn::SUCCESS;
}

auto BravoHardware::read(const rclcpp::Time & /* time */, const rclcpp::Duration & /* period */)
  -> hardware_interface::return_type
{
  const std::scoped_lock lock{position_state_lock_, velocity_state_lock_, torque_state_lock_};

  hw_states_positions_ = async_states_positions_;
  hw_states_velocities_ = async_states_velocities_;
  hw_states_torques_ = async_states_torques_;

  return hardware_interface::return_type::OK;
}

auto BravoHardware::write(const rclcpp::Time & /* time */, const rclcpp::Duration & /* period */)
  -> hardware_interface::return_type
{
  for (std::size_t i = 0; i < info_.joints.size(); ++i) {
    switch (control_modes_[i]) {
      case libreach::Mode::POSITION:
        if (!std::isnan(hw_commands_positions_[i])) {
          // Get the desired position; if the command is for the jaws, then convert from m to mm
          const double position_d = static_cast<libreach::Bravo7DeviceId>(i + 1) == libreach::Bravo7DeviceId::JOINT_A
                                      ? hw_commands_positions_[i] * 1000.0
                                      : hw_commands_positions_[i];

          bravo_->set_position(static_cast<std::uint8_t>(i + 1), static_cast<float>(position_d));
        }
        break;
      case libreach::Mode::VELOCITY:
        if (!std::isnan(hw_commands_velocities_[i])) {
          // Get the target velocity; if the command is for the jaws, then convert from m/s to mm/s
          const double velocity_d = static_cast<libreach::Bravo7DeviceId>(i + 1) == libreach::Bravo7DeviceId::JOINT_A
                                      ? hw_commands_velocities_[i] * 1000.0
                                      : hw_commands_velocities_[i];

          bravo_->set_velocity(static_cast<std::uint8_t>(i + 1), static_cast<float>(velocity_d));
        }
        break;
      case libreach::Mode::CURRENT:
        if (!std::isnan(hw_commands_torques_[i])) {
          const float torque_d = convert_torque_to_current(hw_commands_torques_[i], TORQUE_CONSTANTS[i]);
          bravo_->set_current(static_cast<std::uint8_t>(i + 1), static_cast<float>(torque_d));
        }
        break;
      default:
        break;
    }
  }

  return hardware_interface::return_type::OK;
}

}  // namespace bravo::hardware

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(bravo::hardware::BravoHardware, hardware_interface::SystemInterface)
