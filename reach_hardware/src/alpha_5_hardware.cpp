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

auto map_device_ids_to_names(const std::vector<hardware_interface::ComponentInfo> & joints)
  -> std::tuple<std::unordered_map<std::uint8_t, std::string>, std::unordered_map<std::string, std::uint8_t>>
{
  // Define a mapping between substrings and corresponding device IDs
  const std::array<std::pair<std::string_view, std::uint8_t>, 5> name_to_id_mapping = {
    {"axis_e", static_cast<std::uint8_t>(libreach::Alpha5DeviceId::JOINT_E)},
    {"axis_d", static_cast<std::uint8_t>(libreach::Alpha5DeviceId::JOINT_D)},
    {"axis_c", static_cast<std::uint8_t>(libreach::Alpha5DeviceId::JOINT_C)},
    {"axis_b", static_cast<std::uint8_t>(libreach::Alpha5DeviceId::JOINT_B)},
    {"axis_a", static_cast<std::uint8_t>(libreach::Alpha5DeviceId::JOINT_A)}};

  std::unordered_map<std::uint8_t, std::string> device_ids_to_names;
  std::unordered_map<std::string, std::uint8_t> names_to_device_ids;

  for (const auto & joint : joints) {
    const auto it = std::ranges::find_if(
      name_to_id_mapping, [joint](const auto & pair) { return joint.name.find(pair.first) != std::string::npos; });

    if (it != std::end(name_to_id_mapping)) {
      device_ids_to_names[it->second] = joint.name;
      names_to_device_ids[joint.name] = it->second;
    }
  }

  return {device_ids_to_names, names_to_device_ids};
}

}  // namespace

auto Alpha5Hardware::on_init(const hardware_interface::HardwareInfo & info) -> hardware_interface::CallbackReturn
{
  RCLCPP_INFO(logger_, "Initializing Alpha5Hardware system interface");  // NOLINT

  if (hardware_interface::SystemInterface::on_init(info) != hardware_interface::CallbackReturn::SUCCESS) {
    RCLCPP_INFO(logger_, "Failed to initialize Alpha5Hardware system interface");  // NOLINT
    return hardware_interface::CallbackReturn::ERROR;
  }

  for (const auto & [name, desc] : joint_state_interfaces_) {
    async_states_positions_[name] = std::numeric_limits<double>::quiet_NaN();
    async_states_velocities_[name] = std::numeric_limits<double>::quiet_NaN();
    async_states_torques_[name] = std::numeric_limits<double>::quiet_NaN();
  }

  std::tie(device_ids_to_names_, names_to_device_ids_) = map_device_ids_to_names(info_.joints);

  if (device_ids_to_names_.size() != info_.joints.size() || names_to_device_ids_.size() != info_.joints.size()) {
    RCLCPP_ERROR(logger_, "Failed to map joint names to Alpha device IDs.");  // NOLINT
    return hardware_interface::CallbackReturn::ERROR;
  }

  // Set the default control mode
  control_modes_.resize(info_.joints.size(), libreach::Mode::STANDBY);

  // Load the ROS params
  serial_port_ = info_.hardware_parameters["serial_port"];
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

  RCLCPP_INFO(logger_, "Successfully initialized Alpha5Hardware system interface");  // NOLINT

  return hardware_interface::CallbackReturn::SUCCESS;
}

auto Alpha5Hardware::on_configure(const rclcpp_lifecycle::State & /* previous_state */)
  -> hardware_interface::CallbackReturn
{
  // Use a multi-worker configuration
  const std::size_t buffer_size = 100;
  const std::size_t num_workers = 5;

  // Reset the state and command values
  for (const auto & [name, desc] : joint_state_interfaces_) {
    set_state(name, std::numeric_limits<double>::quiet_NaN());
  }

  for (const auto & [name, desc] : joint_command_interfaces_) {
    set_command(name, std::numeric_limits<double>::quiet_NaN());
  }

  try {
    alpha_ = std::make_unique<libreach::SerialDriver>(serial_port_, buffer_size, num_workers);
  }
  catch (const std::exception & e) {
    RCLCPP_ERROR(logger_, "Failed to configure serial driver: %s", e.what());  // NOLINT
    return hardware_interface::CallbackReturn::ERROR;
  }

  // Register the state callbacks
  alpha_->register_callback(libreach::PacketId::POSITION, [this](const libreach::Packet & packet) {
    const auto position = static_cast<double>(libreach::deserialize<float>(packet));
    const std::lock_guard<std::mutex> lock(position_state_lock_);
    async_states_positions_[device_ids_to_names_[packet.device_id()]] = position;
  });

  alpha_->register_callback(libreach::PacketId::VELOCITY, [this](const libreach::Packet & packet) {
    const auto velocity = static_cast<double>(libreach::deserialize<float>(packet));
    const std::lock_guard<std::mutex> lock(velocity_state_lock_);
    async_states_velocities_[device_ids_to_names_[packet.device_id()]] = velocity;
  });

  alpha_->register_callback(libreach::PacketId::CURRENT, [this](const libreach::Packet & packet) {
    const float torque =
      convert_current_to_torque(libreach::deserialize<float>(packet), TORQUE_CONSTANTS[i], GEAR_RATIO[i]);
    const std::lock_guard<std::mutex> lock(torque_state_lock_);
    async_states_torques_[i] = static_cast<double>(torque);
    async_states_torques_[device_ids_to_names_[packet.device_id()]] = torque;
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
  alpha_->set_mode(static_cast<std::uint8_t>(libreach::Bravo7DeviceId::ALL_JOINTS), libreach::Mode::STANDBY);
  return hardware_interface::CallbackReturn::SUCCESS;
}

auto Alpha5Hardware::on_deactivate(const rclcpp_lifecycle::State & /* previous_state */)
  -> hardware_interface::CallbackReturn
{
  alpha_->set_mode(static_cast<std::uint8_t>(libreach::Bravo7DeviceId::ALL_JOINTS), libreach::Mode::DISABLE);
  return hardware_interface::CallbackReturn::SUCCESS;
}

auto Alpha5Hardware::read(const rclcpp::Time & /* time */, const rclcpp::Duration & /* period */)
  -> hardware_interface::return_type
{
  const std::scoped_lock lock{position_state_lock_, velocity_state_lock_, torque_state_lock_};

  hw_states_positions_ = async_states_positions_;
  hw_states_velocities_ = async_states_velocities_;
  hw_states_torques_ = async_states_torques_;

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
