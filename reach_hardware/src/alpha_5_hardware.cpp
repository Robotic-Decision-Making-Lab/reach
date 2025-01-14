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
const std::array<float, 5> TORQUE_CONSTANTS{0.0271, 0.0271, 0.0134, 0.0134, 0.0134};

// Gr, provided in the order of the Device IDs
const std::array<float, 5> GEAR_RATIO{348.7, 340.4, 1754.4, 1754.4, 1754.4};

// Command interfaces supported by the Alpha5Hardware system
const std::vector<std::string> COMMAND_INTERFACES{
  hardware_interface::HW_IF_POSITION,
  hardware_interface::HW_IF_VELOCITY,
  hardware_interface::HW_IF_EFFORT};

// State interfaces supported by the Alpha5Hardware system
const std::vector<std::string> STATE_INTERFACES{
  hardware_interface::HW_IF_POSITION,
  hardware_interface::HW_IF_VELOCITY,
  hardware_interface::HW_IF_EFFORT};

/// Convert a torque (Nm) to current (mA) given the torque constant (Nm/A) and gear ratio
auto convert_torque_to_current(float t, float Kt, float Gr) -> float { return t / Kt / Gr * 1000.0; }

/// Convert a current (mA) to torque (Nm) given the torque constant (Nm/A) and gear ratio
auto convert_current_to_torque(float c, float Kt, float Gr) -> float { return c * Kt * Gr / 1000.0; }

/// Verify that the provided interfaces are the correct size and use a supported interface type
auto validate_interfaces(
  const std::vector<hardware_interface::InterfaceInfo> & interfaces,
  const std::vector<std::string> & valid_interfaces) -> bool
{
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
    if (desc.interface_info.name == hardware_interface::HW_IF_POSITION) {
      async_states_positions_[desc.prefix_name] = std::numeric_limits<float>::quiet_NaN();
    } else if (desc.interface_info.name == hardware_interface::HW_IF_VELOCITY) {
      async_states_velocities_[desc.prefix_name] = std::numeric_limits<float>::quiet_NaN();
    } else if (desc.interface_info.name == hardware_interface::HW_IF_EFFORT) {
      async_states_efforts_[desc.prefix_name] = std::numeric_limits<float>::quiet_NaN();
    }
  }

  for (const auto & joint : info_.joints) {
    // Create a lookup table for the device IDs. This prevents us from needing to search for the device ID each time we
    // want to send a command or request state information.
    const auto it = joint.parameters.find("device_id");
    if (it == joint.parameters.end()) {
      RCLCPP_ERROR(logger_, "Joint '%s' is missing the 'device_id' parameter", joint.name.c_str());
      return hardware_interface::CallbackReturn::ERROR;
    }

    const auto device_id = static_cast<std::uint8_t>(std::stoi(it->second));
    device_ids_to_names_[device_id] = joint.name;
    names_to_device_ids_[joint.name] = device_id;

    if (!validate_interfaces(joint.command_interfaces, COMMAND_INTERFACES)) {
      RCLCPP_ERROR(
        logger_,
        "Joint '%s' has invalid command interfaces. Verify that the joint has %zu command interfaces and is of a "
        "supported type:",
        joint.name.c_str(),
        COMMAND_INTERFACES.size());
      for (const auto & interface : COMMAND_INTERFACES) {
        RCLCPP_ERROR(logger_, "  - %s", interface.c_str());
      }
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (!validate_interfaces(joint.state_interfaces, STATE_INTERFACES)) {
      RCLCPP_ERROR(
        logger_,
        "Joint '%s' has invalid state interfaces. Verify that the joint has %zu state interfaces and is of a "
        "supported type:",
        joint.name.c_str(),
        STATE_INTERFACES.size());
      for (const auto & interface : STATE_INTERFACES) {
        RCLCPP_ERROR(logger_, "  - %s", interface.c_str());
      }
      return hardware_interface::CallbackReturn::ERROR;
    }
  }

  serial_port_ = info_.hardware_parameters.at("serial_port");

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
  // The device_ids_to_names_ map isn't dynamically updated, so we pass it by value to the lambda. This helps ensure
  // thread safety at the small expense of a couple copies.
  alpha_->register_callback(
    libreach::PacketId::POSITION, [this, ids_to_names = device_ids_to_names_](const libreach::Packet & packet) {
      const auto position = static_cast<double>(libreach::deserialize<float>(packet));
      const std::lock_guard<std::mutex> lock(position_state_lock_);
      async_states_positions_[ids_to_names.at(packet.device_id())] = position;
    });

  alpha_->register_callback(
    libreach::PacketId::VELOCITY, [this, ids_to_names = device_ids_to_names_](const libreach::Packet & packet) {
      const auto velocity = static_cast<double>(libreach::deserialize<float>(packet));
      const std::lock_guard<std::mutex> lock(velocity_state_lock_);
      async_states_velocities_[ids_to_names.at(packet.device_id())] = velocity;
    });

  alpha_->register_callback(
    libreach::PacketId::CURRENT, [this, ids_to_names = device_ids_to_names_](const libreach::Packet & packet) {
      const auto idx = static_cast<std::size_t>(packet.device_id()) - 1;
      const auto current = libreach::deserialize<float>(packet);
      const float torque = convert_current_to_torque(current, TORQUE_CONSTANTS[idx], GEAR_RATIO[idx]);
      const std::lock_guard<std::mutex> lock(effort_state_lock_);
      async_states_efforts_[ids_to_names.at(packet.device_id())] = torque;
    });

  // Configure the state requests
  for (const auto & joint : info_.joints) {
    alpha_->request_at_rate(
      {libreach::PacketId::POSITION, libreach::PacketId::VELOCITY, libreach::PacketId::CURRENT},
      names_to_device_ids_.at(joint.name),
      std::chrono::milliseconds(10));  // 100 Hz
  }

  RCLCPP_INFO(logger_, "Successfully configured the Alpha5Hardware system interface");  // NOLINT

  return hardware_interface::CallbackReturn::SUCCESS;
}

auto Alpha5Hardware::prepare_command_mode_switch(
  const std::vector<std::string> & start_interfaces,
  const std::vector<std::string> & /* stop_interfaces */) -> hardware_interface::return_type
{
  for (const auto & key : start_interfaces) {
    for (const auto & joint : info_.joints) {
      if (key == joint.name + "/" + hardware_interface::HW_IF_POSITION) {
        control_modes_[joint.name] = libreach::Mode::POSITION;
      } else if (key == joint.name + "/" + hardware_interface::HW_IF_VELOCITY) {
        control_modes_[joint.name] = libreach::Mode::VELOCITY;
      } else if (key == joint.name + "/" + hardware_interface::HW_IF_EFFORT) {
        control_modes_[joint.name] = libreach::Mode::CURRENT;
      }
    }
  }

  return hardware_interface::return_type::OK;
}

auto Alpha5Hardware::perform_command_mode_switch(
  const std::vector<std::string> & /* start_interfaces */,
  const std::vector<std::string> & /* stop_interfaces */) -> hardware_interface::return_type
{
  alpha_->set_joint_velocity(std::to_underlying(libreach::Alpha5DeviceId::ALL_JOINTS), 0.0);

  for (const auto & joint : info_.joints) {
    alpha_->set_mode(names_to_device_ids_[joint.name], control_modes_[joint.name]);
  }

  return hardware_interface::return_type::OK;
}

auto Alpha5Hardware::on_activate(const rclcpp_lifecycle::State & /* previous_state */)
  -> hardware_interface::CallbackReturn
{
  alpha_->set_mode(std::to_underlying(libreach::Alpha5DeviceId::ALL_JOINTS), libreach::Mode::STANDBY);
  return hardware_interface::CallbackReturn::SUCCESS;
}

auto Alpha5Hardware::on_deactivate(const rclcpp_lifecycle::State & /* previous_state */)
  -> hardware_interface::CallbackReturn
{
  alpha_->set_mode(std::to_underlying(libreach::Alpha5DeviceId::ALL_JOINTS), libreach::Mode::DISABLE);
  return hardware_interface::CallbackReturn::SUCCESS;
}

auto Alpha5Hardware::read(const rclcpp::Time & /* time */, const rclcpp::Duration & /* period */)
  -> hardware_interface::return_type
{
  const std::scoped_lock lock{position_state_lock_, velocity_state_lock_, effort_state_lock_};

  for (const auto & [name, desc] : joint_state_interfaces_) {
    if (desc.interface_info.name == hardware_interface::HW_IF_POSITION) {
      set_state(name, async_states_positions_[desc.prefix_name]);
    } else if (desc.interface_info.name == hardware_interface::HW_IF_VELOCITY) {
      set_state(name, async_states_velocities_[desc.prefix_name]);
    } else if (desc.interface_info.name == hardware_interface::HW_IF_EFFORT) {
      set_state(name, async_states_efforts_[desc.prefix_name]);
    }
  }

  return hardware_interface::return_type::OK;
}

auto Alpha5Hardware::write(const rclcpp::Time & /* time */, const rclcpp::Duration & /* period */)
  -> hardware_interface::return_type
{
  for (const auto & [name, desc] : joint_command_interfaces_) {
    const auto command = get_command(name);
    if (std::isnan(command)) {
      continue;
    }

    const auto id = names_to_device_ids_.at(desc.prefix_name);
    const auto id_cast = static_cast<libreach::Alpha5DeviceId>(id);

    double position, velocity, current;  // NOLINT

    switch (control_modes_.at(desc.prefix_name)) {
      case libreach::Mode::POSITION:
        position = id_cast == libreach::Alpha5DeviceId::JOINT_A ? command * 1000.0 : command;
        alpha_->set_joint_position(id, static_cast<float>(position));
        break;
      case libreach::Mode::VELOCITY:
        velocity = id_cast == libreach::Alpha5DeviceId::JOINT_A ? command * 1000.0 : command;
        alpha_->set_joint_velocity(id, static_cast<float>(velocity));
        break;
      case libreach::Mode::CURRENT:
        current = convert_torque_to_current(command, TORQUE_CONSTANTS[id - 1], GEAR_RATIO[id - 1]);
        alpha_->set_current(id, static_cast<float>(current));
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
