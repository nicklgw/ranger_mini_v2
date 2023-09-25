// Copyright 2021 Factor Robotics
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef FOUR_WHEEL_STEERING_HARDWARE_INTERFACE_HPP_
#define FOUR_WHEEL_STEERING_HARDWARE_INTERFACE_HPP_

#include <cmath>

#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "four_wheel_steering_hardware_interface/visibility_control.hpp"
#include "rclcpp/rclcpp.hpp"

namespace four_wheel_steering_hardware_interface
{
class FourWheelSteeringHardwareInterface : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(FourWheelSteeringHardwareInterface)

  FOUR_WHEEL_STEERING_HARDWARE_INTERFACE_PUBLIC
  hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo & info) override;

  FOUR_WHEEL_STEERING_HARDWARE_INTERFACE_PUBLIC
  hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State &) override;

  FOUR_WHEEL_STEERING_HARDWARE_INTERFACE_PUBLIC
  hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State &) override;

  FOUR_WHEEL_STEERING_HARDWARE_INTERFACE_PUBLIC
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  FOUR_WHEEL_STEERING_HARDWARE_INTERFACE_PUBLIC
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  FOUR_WHEEL_STEERING_HARDWARE_INTERFACE_PUBLIC
  hardware_interface::return_type prepare_command_mode_switch(const std::vector<std::string> & start_interfaces, const std::vector<std::string> & stop_interfaces) override;

  FOUR_WHEEL_STEERING_HARDWARE_INTERFACE_PUBLIC
  hardware_interface::return_type perform_command_mode_switch(const std::vector<std::string> & start_interfaces, const std::vector<std::string> & stop_interfaces) override;

  FOUR_WHEEL_STEERING_HARDWARE_INTERFACE_PUBLIC
  hardware_interface::return_type read(const rclcpp::Time &, const rclcpp::Duration &) override;

  FOUR_WHEEL_STEERING_HARDWARE_INTERFACE_PUBLIC
  hardware_interface::return_type write(const rclcpp::Time &, const rclcpp::Duration &) override;
  
private:
  
};
}  // namespace four_wheel_steering_hardware_interface

#endif  // FOUR_WHEEL_STEERING_HARDWARE_INTERFACE_HPP_

