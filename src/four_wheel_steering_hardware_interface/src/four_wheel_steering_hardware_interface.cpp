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

#include "four_wheel_steering_hardware_interface/four_wheel_steering_hardware_interface.hpp"


namespace four_wheel_steering_hardware_interface
{
hardware_interface::CallbackReturn FourWheelSteeringHardwareInterface::on_init(const hardware_interface::HardwareInfo & /*info*/)
{
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn FourWheelSteeringHardwareInterface::on_activate(const rclcpp_lifecycle::State &)
{
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn FourWheelSteeringHardwareInterface::on_deactivate(const rclcpp_lifecycle::State &)
{
  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> FourWheelSteeringHardwareInterface::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> FourWheelSteeringHardwareInterface::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;

  return command_interfaces;
}

hardware_interface::return_type FourWheelSteeringHardwareInterface::prepare_command_mode_switch(  const std::vector<std::string> & /*start_interfaces*/, const std::vector<std::string> & /*stop_interfaces*/)
{
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type FourWheelSteeringHardwareInterface::perform_command_mode_switch(const std::vector<std::string> & /*start_interfaces*/, const std::vector<std::string> & /*stop_interfaces*/) 
{
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type FourWheelSteeringHardwareInterface::read(const rclcpp::Time &, const rclcpp::Duration &)
{
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type FourWheelSteeringHardwareInterface::write(const rclcpp::Time &, const rclcpp::Duration &)
{
  return hardware_interface::return_type::OK;
}
}  // namespace four_wheel_steering_hardware_interface

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
  four_wheel_steering_hardware_interface::FourWheelSteeringHardwareInterface, hardware_interface::SystemInterface)
