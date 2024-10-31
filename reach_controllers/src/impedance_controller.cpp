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

#include "reach_controllers/impedance_controller.hpp"

#include <algorithm>
#include <ranges>

namespace reach::controllers
{

namespace
{

auto reset_reference_msg(reach_msgs::msg::MultiDOFImpedanceCommand * msg, const std::vector<std::string> & joint_names)
  -> void
{
  msg->dof_names = joint_names;
  msg->torques.assign(joint_names.size(), std::numeric_limits<double>::quiet_NaN());
  msg->positions.assign(joint_names.size(), std::numeric_limits<double>::quiet_NaN());
  msg->velocities.assign(joint_names.size(), std::numeric_limits<double>::quiet_NaN());
}

const std::array<std::string, 2> REQUIRED_STATE_INTERFACES{
  hardware_interface::HW_IF_POSITION,
  hardware_interface::HW_IF_VELOCITY};

const std::array<std::string, 3> REQUIRED_REFERENCE_INTERFACES{
  hardware_interface::HW_IF_POSITION,
  hardware_interface::HW_IF_VELOCITY,
  hardware_interface::HW_IF_EFFORT};

}  // namespace

auto ImpedanceController::on_init() -> controller_interface::CallbackReturn
{
  try {
    param_listener_ = std::make_shared<impedance_controller::ParamListener>(get_node());
    params_ = param_listener_->get_params();
  }
  catch (const std::exception & e) {
    std::cerr << "An exception occurred while initializing the controller: " << e.what() << "\n";
    return controller_interface::CallbackReturn::ERROR;
  }

  return controller_interface::CallbackReturn::SUCCESS;
}

auto ImpedanceController::update_parameters() -> void
{
  if (!param_listener_->is_old(params_)) {
    return;
  }
  param_listener_->refresh_dynamic_parameters();
  params_ = param_listener_->get_params();
}

auto ImpedanceController::configure_parameters() -> controller_interface::CallbackReturn
{
  update_parameters();

  joint_names_ = params_.joints;
  num_joints_ = joint_names_.size();

  joint_friction_ = params_.friction;

  controller_gains_.reserve(num_joints_);
  for (const auto & name : joint_names_) {
    controller_gains_.emplace_back(params_.gains.joints_map[name].damping, params_.gains.joints_map[name].stiffness);
  }

  return controller_interface::CallbackReturn::SUCCESS;
}

auto ImpedanceController::on_configure(const rclcpp_lifecycle::State & /*previous_state */)
  -> controller_interface::CallbackReturn
{
  const auto ret = configure_parameters();
  if (ret != controller_interface::CallbackReturn::SUCCESS) {
    return ret;
  }

  reference_.writeFromNonRT(reach_msgs::msg::MultiDOFImpedanceCommand());

  command_interfaces_.reserve(num_joints_);

  system_state_values_.resize(REQUIRED_STATE_INTERFACES.size() * num_joints_, std::numeric_limits<double>::quiet_NaN());
  position_error_.resize(num_joints_, std::numeric_limits<double>::quiet_NaN());
  velocity_error_.resize(num_joints_, std::numeric_limits<double>::quiet_NaN());

  reference_sub_ = get_node()->create_subscription<reach_msgs::msg::MultiDOFImpedanceCommand>(
    "multi_dof_impedance_command",
    rclcpp::SystemDefaultsQoS(),
    [this](const std::shared_ptr<reach_msgs::msg::MultiDOFImpedanceCommand>
             msg) {  // NOLINT(performance-unnecessary-value-param)
      reference_.writeFromNonRT(*msg);
    });

  controller_state_pub_ = get_node()->create_publisher<reach_msgs::msg::MultiDOFImpedanceStateStamped>(
    "~/status", rclcpp::SystemDefaultsQoS());

  rt_controller_state_pub_ =
    std::make_unique<realtime_tools::RealtimePublisher<reach_msgs::msg::MultiDOFImpedanceStateStamped>>(
      controller_state_pub_);

  auto p_terms = controller_gains_ | std::views::transform([](const auto & gains) { return gains.damping; });
  auto d_terms = controller_gains_ | std::views::transform([](const auto & gains) { return gains.stiffness; });

  rt_controller_state_pub_->lock();
  rt_controller_state_pub_->msg_.dof_names = joint_names_;
  rt_controller_state_pub_->msg_.torques.resize(num_joints_, std::numeric_limits<double>::quiet_NaN());
  rt_controller_state_pub_->msg_.position_errors.resize(num_joints_, std::numeric_limits<double>::quiet_NaN());
  rt_controller_state_pub_->msg_.velocity_errors.resize(num_joints_, std::numeric_limits<double>::quiet_NaN());
  rt_controller_state_pub_->msg_.outputs.resize(num_joints_, std::numeric_limits<double>::quiet_NaN());
  rt_controller_state_pub_->msg_.friction_terms = joint_friction_;

  rt_controller_state_pub_->msg_.p_terms.clear();
  std::ranges::copy(p_terms, std::back_inserter(rt_controller_state_pub_->msg_.p_terms));

  rt_controller_state_pub_->msg_.d_terms.clear();
  std::ranges::copy(d_terms, std::back_inserter(rt_controller_state_pub_->msg_.d_terms));

  rt_controller_state_pub_->unlock();

  return controller_interface::CallbackReturn::SUCCESS;
}

auto ImpedanceController::on_activate(const rclcpp_lifecycle::State & /*previous_state */)
  -> controller_interface::CallbackReturn
{
  reset_reference_msg(reference_.readFromNonRT(), joint_names_);

  system_state_values_.assign(system_state_values_.size(), std::numeric_limits<double>::quiet_NaN());
  reference_interfaces_.assign(reference_interfaces_.size(), std::numeric_limits<double>::quiet_NaN());

  return controller_interface::CallbackReturn::SUCCESS;
}

auto ImpedanceController::on_set_chained_mode(bool /*chained_mode*/) -> bool { return true; }

auto ImpedanceController::command_interface_configuration() const -> controller_interface::InterfaceConfiguration
{
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

  config.names.reserve(num_joints_);

  for (std::size_t i = 0; i < num_joints_; ++i) {
    config.names.push_back(
      params_.reference_names.empty()
        ? joint_names_[i] + "/" + hardware_interface::HW_IF_EFFORT
        : params_.reference_names[i] + "/" + joint_names_[i] + "/" + hardware_interface::HW_IF_EFFORT);
  }

  return config;
}

auto ImpedanceController::state_interface_configuration() const -> controller_interface::InterfaceConfiguration
{
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

  config.names.reserve(REQUIRED_STATE_INTERFACES.size() * num_joints_);

  for (const auto & state : REQUIRED_STATE_INTERFACES) {
    for (const auto & joint_name : joint_names_) {
      config.names.push_back(joint_name + "/" + state);  // NOLINT(performance-inefficient-string-concatenation)
    }
  }

  return config;
}

auto ImpedanceController::on_export_reference_interfaces() -> std::vector<hardware_interface::CommandInterface>
{
  reference_interfaces_.resize(
    REQUIRED_REFERENCE_INTERFACES.size() * num_joints_, std::numeric_limits<double>::quiet_NaN());

  std::vector<hardware_interface::CommandInterface> exported_interfaces;
  exported_interfaces.reserve(reference_interfaces_.size());

  std::size_t i = 0;
  for (const auto & reference : REQUIRED_REFERENCE_INTERFACES) {
    for (const auto & joint_name : joint_names_) {
      exported_interfaces.emplace_back(
        get_node()->get_name(),
        joint_name + "/" + reference,  // NOLINT(performance-inefficient-string-concatenation)
        &reference_interfaces_[i]);
      ++i;
    }
  }

  return exported_interfaces;
}

auto ImpedanceController::update_reference_from_subscribers(
  const rclcpp::Time & /*time*/,
  const rclcpp::Duration & /*period*/) -> controller_interface::return_type
{
  auto * current_reference = reference_.readFromNonRT();

  std::size_t i = 0;
  for (const auto & reference : REQUIRED_REFERENCE_INTERFACES) {
    for (std::size_t j = 0; j < num_joints_; ++j) {
      if (reference == hardware_interface::HW_IF_POSITION) {
        reference_interfaces_[i] = current_reference->positions[j];
      } else if (reference == hardware_interface::HW_IF_VELOCITY) {
        reference_interfaces_[i] = current_reference->velocities[j];
      } else if (reference == hardware_interface::HW_IF_EFFORT) {
        reference_interfaces_[i] = current_reference->torques[j];
      } else {
        return controller_interface::return_type::ERROR;
      }
      ++i;
    }
  }

  reset_reference_msg(current_reference, joint_names_);

  return controller_interface::return_type::OK;
}

auto ImpedanceController::update_system_state_values() -> controller_interface::return_type
{
  for (std::size_t i = 0; i < REQUIRED_STATE_INTERFACES.size(); ++i) {
    for (std::size_t j = 0; j < num_joints_; ++j) {
      system_state_values_[i * num_joints_ + j] = state_interfaces_[i * num_joints_ + j].get_value();
    }
  }

  return controller_interface::return_type::OK;
}

auto ImpedanceController::update_and_write_commands(const rclcpp::Time & time, const rclcpp::Duration & /* period */)
  -> controller_interface::return_type
{
  if (params_.enable_parameter_update_without_reactivation) {
    configure_parameters();
  }

  update_system_state_values();

  for (std::size_t i = 0; i < num_joints_; ++i) {
    // Calculate the error terms
    position_error_[i] = reference_interfaces_[i] - system_state_values_[i];
    velocity_error_[i] = reference_interfaces_[num_joints_ + i] - system_state_values_[num_joints_ + i];

    const double feedforward = reference_interfaces_[2 * num_joints_ + i];

    double command = std::numeric_limits<double>::quiet_NaN();

    if (!std::isnan(position_error_[i]) && !std::isnan(velocity_error_[i]) && !std::isnan(feedforward)) {
      // Calculate the impedance control
      command = feedforward + controller_gains_[i].damping * velocity_error_[i] +
                controller_gains_[i].stiffness * position_error_[i];

      // Add friction compensation
      command += joint_friction_[i];
    }

    command_interfaces_[i].set_value(command);
  }

  if (rt_controller_state_pub_ && rt_controller_state_pub_->trylock()) {
    rt_controller_state_pub_->msg_.header.stamp = time;

    for (std::size_t i = 0; i < num_joints_; ++i) {
      rt_controller_state_pub_->msg_.torques[i] = reference_interfaces_[2 * num_joints_ + i];
      rt_controller_state_pub_->msg_.position_errors[i] = position_error_[i];
      rt_controller_state_pub_->msg_.velocity_errors[i] = velocity_error_[i];
      rt_controller_state_pub_->msg_.outputs[i] = command_interfaces_[i].get_value();
      rt_controller_state_pub_->msg_.p_terms[i] = controller_gains_[i].damping;
      rt_controller_state_pub_->msg_.d_terms[i] = controller_gains_[i].stiffness;
      rt_controller_state_pub_->msg_.friction_terms[i] = joint_friction_[i];
    }

    rt_controller_state_pub_->unlockAndPublish();
  }

  return controller_interface::return_type::OK;
}

}  // namespace reach::controllers

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(reach::controllers::ImpedanceController, controller_interface::ChainableControllerInterface)
