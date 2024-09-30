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

#include "reach_hardware/alpha_5_hardware.hpp"

#include <limits>

#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

namespace reach::hardware
{

namespace
{

// NOTE: The Reach System Communication Protocol uses the opposite order for the Device IDs than what is used in the
// dynamics/kinematics datasheet.

// Kt, provided in the order of the Device IDs (V_pk / (rad/s))
const std::array<float, 7> TORQUE_CONSTANTS{0.209, 0.209, 0.215, 0.215, 0.215, 0.222, 0.222};

// Gr, provided in the order of the Device IDs
const std::array<float, 7> GEAR_RATIO{39.27, 120.0, 120.0, 120.0, 120.0, 120.0, 120.0};

/// Convert a torque (Nm) to current (mA) given the torque constant (Nm/A) and gear ratio
auto convert_torque_to_current(float t, float Kt, float Gr) -> float { return t / Kt / Gr * 1000.0; }

/// Convert a current (mA) to torque (Nm) given the torque constant (Nm/A) and gear ratio
auto convert_current_to_torque(float c, float Kt, float Gr) -> float { return c * Kt * Gr / 1000.0; }

const std::vector<std::string> VALID_COMMAND_INTERFACES{
  hardware_interface::HW_IF_POSITION,
  hardware_interface::HW_IF_VELOCITY,
  hardware_interface::HW_IF_EFFORT};

const std::vector<std::string> VALID_STATE_INTERFACES{
  hardware_interface::HW_IF_POSITION,
  hardware_interface::HW_IF_VELOCITY,
  hardware_interface::HW_IF_EFFORT};

/// Verify that the provided interfaces are the correct size and use a supported interface type
auto validate_interfaces(
  const std::vector<hardware_interface::InterfaceInfo> & interfaces,
  const std::vector<std::string> & valid_interfaces) -> bool
{
  if (interfaces.size() != valid_interfaces.size()) {
    return false;
  }

  return std::ranges::all_of(interfaces, [valid_interfaces](const hardware_interface::InterfaceInfo & interface) {
    return std::ranges::any_of(
      valid_interfaces, [interface](const std::string & valid_interface) { return interface.name == valid_interface; });
  });
}

}  // namespace

auto Alpha5Hardware::on_init(const hardware_interface::HardwareInfo & info) -> hardware_interface::CallbackReturn
{
  RCLCPP_INFO(logger_, "Initializing Alpha5Hardware system interface");

  if (hardware_interface::SystemInterface::on_init(info) != hardware_interface::CallbackReturn::SUCCESS) {
    RCLCPP_INFO(logger_, "Failed to initialize Alpha5Hardware system interface");
    return hardware_interface::CallbackReturn::ERROR;
  }

  for (const auto & [name, desc] : joint_state_interfaces_) {
    async_states_[name] = std::numeric_limits<double>::quiet_NaN();
  }

  for (const auto & joint : info_.joints) {
    if (joint.parameters.find("device_id") == joint.parameters.end()) {
      RCLCPP_ERROR(logger_, "Joint '%s' is missing the 'device_id' parameter", joint.name.c_str());
      return hardware_interface::CallbackReturn::ERROR;
    }

    // Save the mapping between joint names and device IDs
    const auto device_id = static_cast<std::uint8_t>(std::stoi(joint.parameters.at("device_id")));
    device_ids_to_names_[device_id] = joint.name;
    names_to_device_ids_[joint.name] = device_id;

    if (!validate_interfaces(joint.command_interfaces, VALID_COMMAND_INTERFACES)) {
      RCLCPP_ERROR(
        logger_,
        "Joint '%s' has invalid command interfaces. Verify that the joint has %zu command interfaces and is of a "
        "supported type:",
        joint.name.c_str(),
        VALID_COMMAND_INTERFACES.size());
      for (const auto & interface : VALID_COMMAND_INTERFACES) {
        RCLCPP_ERROR(logger_, "  - %s", interface.c_str());
      }
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (!validate_interfaces(joint.state_interfaces, VALID_STATE_INTERFACES)) {
      RCLCPP_ERROR(
        logger_,
        "Joint '%s' has invalid state interfaces. Verify that the joint has %zu state interfaces and is of a "
        "supported type:",
        joint.name.c_str(),
        VALID_STATE_INTERFACES.size());
      for (const auto & interface : VALID_STATE_INTERFACES) {
        RCLCPP_ERROR(logger_, "  - %s", interface.c_str());
      }
      return hardware_interface::CallbackReturn::ERROR;
    }
  }

  // Set the default control mode
  control_modes_.resize(info_.joints.size(), libreach::Mode::STANDBY);

  // Load the ROS params
  serial_port_ = info_.hardware_parameters["serial_port"];
  state_request_rate_ = std::chrono::milliseconds(std::stoi(info_.hardware_parameters["state_request_rate"]));

  RCLCPP_INFO(logger_, "Successfully initialized Alpha5Hardware system interface");  // NOLINT

  return hardware_interface::CallbackReturn::SUCCESS;
}

auto Alpha5Hardware::on_configure(const rclcpp_lifecycle::State & /* previous_state */)
  -> hardware_interface::CallbackReturn
{
  // Reset the state and command values
  for (const auto & [name, desc] : joint_state_interfaces_) {
    set_state(name, std::numeric_limits<double>::quiet_NaN());
  }

  for (const auto & [name, desc] : joint_command_interfaces_) {
    set_command(name, std::numeric_limits<double>::quiet_NaN());
  }

  try {
    // Use a multi-worker configuration
    const std::size_t buffer_size = 100;
    const std::size_t num_workers = 5;
    alpha_ = std::make_unique<libreach::SerialDriver>(serial_port_, buffer_size, num_workers);
  }
  catch (const std::exception & e) {
    RCLCPP_ERROR(logger_, "Failed to configure serial driver: %s", e.what());  // NOLINT
    return hardware_interface::CallbackReturn::ERROR;
  }

  // Register the state callbacks
  alpha_->register_callback(libreach::PacketId::POSITION, [this](const libreach::Packet & packet) {
    const auto position = static_cast<double>(libreach::deserialize<float>(packet));
    const std::string interface = device_ids_to_names_[packet.device_id()] + "/" + hardware_interface::HW_IF_POSITION;
    const std::lock_guard<std::mutex> lock(async_state_lock_);
    async_states_[interface] = position;
  });

  alpha_->register_callback(libreach::PacketId::VELOCITY, [this](const libreach::Packet & packet) {
    const auto velocity = static_cast<double>(libreach::deserialize<float>(packet));
    const std::string interface = device_ids_to_names_[packet.device_id()] + "/" + hardware_interface::HW_IF_VELOCITY;
    const std::lock_guard<std::mutex> lock(async_state_lock_);
    async_states_[interface] = velocity;
  });

  alpha_->register_callback(libreach::PacketId::CURRENT, [this](const libreach::Packet & packet) {
    const auto idx = static_cast<std::size_t>(packet.device_id()) - 1;
    const auto current = libreach::deserialize<float>(packet);
    const float torque = convert_current_to_torque(current, TORQUE_CONSTANTS[idx], GEAR_RATIO[idx]);
    const std::string interface = device_ids_to_names_[packet.device_id()] + "/" + hardware_interface::HW_IF_EFFORT;
    const std::lock_guard<std::mutex> lock(async_state_lock_);
    async_states_[interface] = torque;
  });

  // Configure the state requests
  for (std::size_t i = 1; i < joint_state_interfaces_.size() + 1; ++i) {
    alpha_->request_at_rate(
      {libreach::PacketId::POSITION, libreach::PacketId::VELOCITY, libreach::PacketId::CURRENT},
      i,
      state_request_rate_);
  }

  RCLCPP_INFO(logger_, "Successfully configured the Alpha5Hardware system interface");  // NOLINT

  return hardware_interface::CallbackReturn::SUCCESS;
}

auto Alpha5Hardware::prepare_command_mode_switch(
  const std::vector<std::string> & start_interfaces,
  const std::vector<std::string> & /* stop_interfaces */) -> hardware_interface::return_type
{
  std::vector<libreach::Mode> new_modes(control_modes_.size());

  for (const auto & key : start_interfaces) {
    for (auto & joint : info_.joints) {
      if (key == joint.name + "/" + hardware_interface::HW_IF_POSITION) {
        new_modes.push_back(libreach::Mode::POSITION);
      } else if (key == joint.name + "/" + hardware_interface::HW_IF_VELOCITY) {
        new_modes.push_back(libreach::Mode::VELOCITY);
      } else if (key == joint.name + "/" + hardware_interface::HW_IF_EFFORT) {
        new_modes.push_back(libreach::Mode::CURRENT);
      } else {
        RCLCPP_WARN(  // NOLINT
          logger_,
          "Unknown command mode for joint '%s': '%s'. Setting mode to 'STANDBY'",
          joint.name.c_str(),
          key.c_str());
        new_modes.push_back(libreach::Mode::STANDBY);
      }
    }
  }

  if (!new_modes.empty() && new_modes.size() != info_.joints.size()) {
    return hardware_interface::return_type::ERROR;
  }

  control_modes_ = new_modes;

  return hardware_interface::return_type::OK;
}

auto Alpha5Hardware::perform_command_mode_switch(
  const std::vector<std::string> & /* start_interfaces */,
  const std::vector<std::string> & /* stop_interfaces */) -> hardware_interface::return_type
{
  // Stop all joints
  alpha_->set_velocity(static_cast<std::uint8_t>(libreach::Alpha5DeviceId::ALL_JOINTS), 0.0);

  // Perform the mode switch
  for (std::size_t i = 0; i < info_.joints.size(); ++i) {
    alpha_->set_mode(static_cast<std::uint8_t>(i + 1), control_modes_[i]);
  }

  return hardware_interface::return_type::OK;
}

auto Alpha5Hardware::on_activate(const rclcpp_lifecycle::State & /* previous_state */)
  -> hardware_interface::CallbackReturn
{
  alpha_->set_mode(static_cast<std::uint8_t>(libreach::Alpha5DeviceId::ALL_JOINTS), libreach::Mode::STANDBY);
  return hardware_interface::CallbackReturn::SUCCESS;
}

auto Alpha5Hardware::on_deactivate(const rclcpp_lifecycle::State & /* previous_state */)
  -> hardware_interface::CallbackReturn
{
  alpha_->set_mode(static_cast<std::uint8_t>(libreach::Alpha5DeviceId::ALL_JOINTS), libreach::Mode::DISABLE);
  return hardware_interface::CallbackReturn::SUCCESS;
}

auto Alpha5Hardware::read(const rclcpp::Time & /* time */, const rclcpp::Duration & /* period */)
  -> hardware_interface::return_type
{
  const std::lock_guard<std::mutex> lock(async_state_lock_);

  for (const auto & [name, desc] : joint_state_interfaces_) {
    set_state(name, async_states_[name]);
  }

  return hardware_interface::return_type::OK;
}

auto Alpha5Hardware::write(const rclcpp::Time & /* time */, const rclcpp::Duration & /* period */)
  -> hardware_interface::return_type
{
  // TODO(evan-palmer): Saturate the commands as they approach the limits
  for (std::size_t i = 0; i < info_.joints.size(); ++i) {
    switch (control_modes_[i]) {
      case libreach::Mode::POSITION:
        if (!std::isnan(hw_commands_positions_[i])) {
          // Get the desired position; if the command is for the jaws, then convert from m to mm
          const double position_d = static_cast<libreach::Bravo7DeviceId>(i + 1) == libreach::Bravo7DeviceId::JOINT_A
                                      ? hw_commands_positions_[i] * 1000.0
                                      : hw_commands_positions_[i];

          alpha_->set_position(static_cast<std::uint8_t>(i + 1), static_cast<float>(position_d));
        }
        break;
      case libreach::Mode::VELOCITY:
        if (!std::isnan(hw_commands_velocities_[i])) {
          // Get the target velocity; if the command is for the jaws, then convert from m/s to mm/s
          const double velocity_d = static_cast<libreach::Bravo7DeviceId>(i + 1) == libreach::Bravo7DeviceId::JOINT_A
                                      ? hw_commands_velocities_[i] * 1000.0
                                      : hw_commands_velocities_[i];

          alpha_->set_velocity(static_cast<std::uint8_t>(i + 1), static_cast<float>(velocity_d));
        }
        break;
      case libreach::Mode::CURRENT:
        if (!std::isnan(hw_commands_torques_[i])) {
          const float torque_d = convert_torque_to_current(hw_commands_torques_[i], TORQUE_CONSTANTS[i], GEAR_RATIO[i]);
          alpha_->set_current(static_cast<std::uint8_t>(i + 1), static_cast<float>(torque_d));
        }
        break;
      default:
        break;
    }
  }

  return hardware_interface::return_type::OK;
}

}  // namespace reach::hardware

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(reach::hardware::Alpha5Hardware, hardware_interface::SystemInterface)
