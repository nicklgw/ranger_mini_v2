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
#include "four_wheel_steering_hardware_interface/four_wheel_steering_drive_helper.h"

// 0,虚拟机; 1,实体机
#define PHYSICAL_MACHINE 0

namespace four_wheel_steering_hardware_interface
{
hardware_interface::CallbackReturn FourWheelSteeringHardwareInterface::on_init(const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::SystemInterface::on_init(info) != hardware_interface::CallbackReturn::SUCCESS) 
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  hw_positions_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  hw_velocities_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  hw_efforts_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  hw_commands_positions_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  hw_commands_velocities_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  hw_commands_efforts_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());

  for (const hardware_interface::ComponentInfo & joint : info_.joints) 
  {
    node_id_.emplace_back(std::stoi(joint.parameters.at("node_id")));
  }

  control_level_.resize(info_.joints.size(), integration_level_t::UNDEFINED);

#if PHYSICAL_MACHINE
  int ret = FourWheelSteeringDriveHelper::Init();
  RCLCPP_INFO(rclcpp::get_logger("FourWheelSteeringHardware"), "FourWheelSteeringDriveHelper::Init(): %d", ret);
#else  
  RCLCPP_INFO(rclcpp::get_logger("FourWheelSteeringHardware"), "FourWheelSteeringDriveHelper::Init()");
#endif

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn FourWheelSteeringHardwareInterface::on_activate(const rclcpp_lifecycle::State &)
{
  for (size_t i = 0; i < info_.joints.size(); i++)
  {
    hw_positions_[i] = 0.0;
    hw_velocities_[i] = 0.0;
    hw_efforts_[i] = 0.0;
    hw_commands_positions_[i] = 0.0;
    hw_commands_velocities_[i] = 0.0;
    hw_commands_efforts_[i] = 0.0;
  }

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn FourWheelSteeringHardwareInterface::on_deactivate(const rclcpp_lifecycle::State &)
{
#if PHYSICAL_MACHINE
  FourWheelSteeringDriveHelper::Exit();
  RCLCPP_INFO(rclcpp::get_logger("FourWheelSteeringHardware"), "FourWheelSteeringDriveHelper::Exit()");
#endif

  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> FourWheelSteeringHardwareInterface::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;

  for (size_t i = 0; i < info_.joints.size(); i++) 
  {
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      info_.joints[i].name, hardware_interface::HW_IF_EFFORT, &hw_efforts_[i]));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_velocities_[i]));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_positions_[i]));
  }

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> FourWheelSteeringHardwareInterface::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  for (size_t i = 0; i < info_.joints.size(); i++) 
  {
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
      info_.joints[i].name, hardware_interface::HW_IF_EFFORT, &hw_commands_efforts_[i]));
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
      info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_commands_velocities_[i]));
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
      info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_commands_positions_[i]));
  }

  return command_interfaces;
}

hardware_interface::return_type FourWheelSteeringHardwareInterface::prepare_command_mode_switch(const std::vector<std::string> & start_interfaces, const std::vector<std::string> & stop_interfaces)
{
  for (std::string key : stop_interfaces) 
  {
    for (size_t i = 0; i < info_.joints.size(); i++) 
    {
      if (key.find(info_.joints[i].name) != std::string::npos) 
      {
        control_level_[i] = integration_level_t::UNDEFINED;
      }
    }
  }

  for (std::string key : start_interfaces) 
  {
    for (size_t i = 0; i < info_.joints.size(); i++) 
    {
      switch (control_level_[i]) 
      {
        case integration_level_t::UNDEFINED:
          if (key == info_.joints[i].name + "/" + hardware_interface::HW_IF_EFFORT) 
          {
            control_level_[i] = integration_level_t::EFFORT;
          }

        case integration_level_t::EFFORT:
          if (key == info_.joints[i].name + "/" + hardware_interface::HW_IF_VELOCITY) 
          {
            control_level_[i] = integration_level_t::VELOCITY;
          }

        case integration_level_t::VELOCITY:
          if (key == info_.joints[i].name + "/" + hardware_interface::HW_IF_POSITION) 
          {
            control_level_[i] = integration_level_t::POSITION;
          }

        case integration_level_t::POSITION:
          break;
      }
    }
  }

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type FourWheelSteeringHardwareInterface::perform_command_mode_switch(const std::vector<std::string> & /*start_interfaces*/, const std::vector<std::string> & /*stop_interfaces*/) 
{
  for (size_t i = 0; i < info_.joints.size(); i++) 
  {
    switch (control_level_[i]) 
    {
      case integration_level_t::UNDEFINED:
        break;

      case integration_level_t::EFFORT:
        break;

      case integration_level_t::VELOCITY:
        break;

      case integration_level_t::POSITION:
        break;
    }
  }

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type FourWheelSteeringHardwareInterface::read(const rclcpp::Time & time, const rclcpp::Duration & period)
{
  for (size_t i = 0; i < info_.joints.size(); i++) 
  {
#if PHYSICAL_MACHINE
    hw_velocities_[i] = FourWheelSteeringDriveHelper::GetVelocity(node_id_[i]);
    hw_positions_[i] = FourWheelSteeringDriveHelper::GetPosition(node_id_[i]);
#else    
    switch (control_level_[i]) 
    {
      case integration_level_t::POSITION:
        hw_positions_[i];
        break;
      case integration_level_t::VELOCITY:
        hw_positions_[i] = hw_positions_[i] + period.seconds() * hw_velocities_[i];
        break;
      case integration_level_t::EFFORT:

        break;
      case integration_level_t::UNDEFINED:

        break;
    }
#endif
  }

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type FourWheelSteeringHardwareInterface::write(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  for (size_t i = 0; i < info_.joints.size(); i++) 
  {
    float input_torque, input_vel, input_pos;

    switch (control_level_[i]) 
    {
      case integration_level_t::POSITION:
        {
          input_pos = hw_commands_positions_[i];
        #if PHYSICAL_MACHINE
          FourWheelSteeringDriveHelper::SetPosition(node_id_[i], input_pos);
        #else
          hw_positions_[i] = hw_commands_positions_[i];
        #endif
        }
        break;
      case integration_level_t::VELOCITY:
        {
          input_vel = hw_commands_velocities_[i];
          #if PHYSICAL_MACHINE
            hw_velocities_[i] = hw_commands_velocities_[i];
          #else
            FourWheelSteeringDriveHelper::SetVelocity(node_id_[i], input_vel);  // 设置指定轮子角速度，单位rad/s
          #endif
        }
        break;
      case integration_level_t::EFFORT:
        input_torque = hw_commands_efforts_[i];
        break;
      case integration_level_t::UNDEFINED:
        break;
    }
  }

  return hardware_interface::return_type::OK;
}
}  // namespace four_wheel_steering_hardware_interface

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
  four_wheel_steering_hardware_interface::FourWheelSteeringHardwareInterface, hardware_interface::SystemInterface)
