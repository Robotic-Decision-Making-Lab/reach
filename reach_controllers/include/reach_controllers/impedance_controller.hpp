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

#include "controller_interface/chainable_controller_interface.hpp"
#include "controller_interface/controller_interface.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "realtime_tools/realtime_buffer.h"
#include "realtime_tools/realtime_publisher.h"

namespace reach::controllers
{

class ImpedanceController : public controller_interface::ChainableControllerInterface
{
public:
  ImpedanceController() = default;

  auto on_init() -> controller_interface::CallbackReturn override;

  // NOLINTNEXTLINE(modernize-use-nodiscard)
  auto command_interface_configuration() const -> controller_interface::InterfaceConfiguration override;

  // NOLINTNEXTLINE(modernize-use-nodiscard)
  auto state_interface_configuration() const -> controller_interface::InterfaceConfiguration override;

  auto on_cleanup(const rclcpp_lifecycle::State & previous_state) -> controller_interface::CallbackReturn override;

  auto on_configure(const rclcpp_lifecycle::State & previous_state) -> controller_interface::CallbackReturn override;

  auto on_activate(const rclcpp_lifecycle::State & previous_state) -> controller_interface::CallbackReturn override;

  auto on_deactivate(const rclcpp_lifecycle::State & previous_state) -> controller_interface::CallbackReturn override;

  auto update_and_write_commands(const rclcpp::Time & time, const rclcpp::Duration & period)
    -> controller_interface::return_type override;

  auto on_set_chained_mode(bool chained_mode) -> bool override;

protected:
  auto on_export_reference_interfaces() -> std::vector<hardware_interface::CommandInterface> override;

  auto update_reference_from_subscribers(const rclcpp::Time & time, const rclcpp::Duration & period)
    -> controller_interface::return_type override;

  auto update_parameters() -> void;

  auto configure_parameters() -> controller_interface::CallbackReturn;

  // realtime_tools::RealtimeBuffer<std::shared_ptr<std_msgs::msg::Float64>> reference_;
  // std::shared_ptr<rclcpp::Subscription<std_msgs::msg::Float64>> reference_sub_;

  // std::shared_ptr<rclcpp::Publisher<control_msgs::msg::SingleDOFStateStamped>> controller_state_pub_;
  // std::unique_ptr<realtime_tools::RealtimePublisher<control_msgs::msg::SingleDOFStateStamped>>
  // rt_controller_state_pub_;

  // std::shared_ptr<polynomial_thrust_curve_controller::ParamListener> param_listener_;
  // polynomial_thrust_curve_controller::Params params_;
};

}  // namespace reach::controllers
