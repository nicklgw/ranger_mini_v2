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

#include "bdrive_hardware_interface/bdrive_hardware_interface.hpp"
#include "bdrive_hardware_interface/bdrive_helper.h"

// 0,虚拟机; 1,实体机
#define PHYSICAL_MACHINE 0

namespace bdrive_hardware_interface
{
hardware_interface::CallbackReturn BDriveHardwareInterface::on_init(const hardware_interface::HardwareInfo & info)
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
  int ret = BDriveHelper::Init();
  RCLCPP_INFO(rclcpp::get_logger("FourWheelSteeringHardware"), "BDriveHelper::Init(): %d", ret);
#else  
  RCLCPP_INFO(rclcpp::get_logger("FourWheelSteeringHardware"), "BDriveHelper::Init()");
#endif

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn BDriveHardwareInterface::on_activate(const rclcpp_lifecycle::State &)
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

hardware_interface::CallbackReturn BDriveHardwareInterface::on_deactivate(const rclcpp_lifecycle::State &)
{
#if PHYSICAL_MACHINE
  BDriveHelper::Exit();
  RCLCPP_INFO(rclcpp::get_logger("BDriveHelper"), "BDriveHelper::Exit()");
#endif

  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> BDriveHardwareInterface::export_state_interfaces()
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

std::vector<hardware_interface::CommandInterface> BDriveHardwareInterface::export_command_interfaces()
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

hardware_interface::return_type BDriveHardwareInterface::prepare_command_mode_switch(const std::vector<std::string> & start_interfaces, const std::vector<std::string> & stop_interfaces)
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

hardware_interface::return_type BDriveHardwareInterface::perform_command_mode_switch(const std::vector<std::string> & /*start_interfaces*/, const std::vector<std::string> & /*stop_interfaces*/) 
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

hardware_interface::return_type BDriveHardwareInterface::read(const rclcpp::Time & /*time*/, const rclcpp::Duration & period)
{
  for (size_t i = 0; i < info_.joints.size(); i++) 
  {
#if PHYSICAL_MACHINE
    hw_velocities_[i] = BDriveHelper::GetVelocity(node_id_[i]);
    hw_positions_[i] = BDriveHelper::GetPosition(node_id_[i]);
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

hardware_interface::return_type BDriveHardwareInterface::write(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  for (size_t i = 0; i < info_.joints.size(); i++) 
  {
    float input_torque = 0.0, input_vel = 0.0, input_pos = 0.0;

    switch (control_level_[i]) 
    {
      case integration_level_t::POSITION:
        {
          input_pos = hw_commands_positions_[i];
        #if PHYSICAL_MACHINE
          BDriveHelper::SetPosition(node_id_[i], input_pos);
        #else
          hw_positions_[i] = hw_commands_positions_[i];
        #endif
        }
        break;
      case integration_level_t::VELOCITY:
        {
          input_vel = hw_commands_velocities_[i];
          #if PHYSICAL_MACHINE
            BDriveHelper::SetVelocity(node_id_[i], input_vel);  // 设置指定轮子角速度，单位rad/s
          #else
            hw_velocities_[i] = hw_commands_velocities_[i];
          #endif
        }
        break;
      case integration_level_t::EFFORT:
        input_torque = hw_commands_efforts_[i];
        break;
      case integration_level_t::UNDEFINED:
        break;
    }
    
    RCLCPP_INFO(rclcpp::get_logger("BDriveHardwareInterface"), 
    "i: %ld, node_id: %d, control_level: %d, input_pos: %.3f, input_vel: %.3f", 
    i, node_id_[i], static_cast<int32_t>(control_level_[i]), input_pos, input_vel);
  }

  return hardware_interface::return_type::OK;
}
}  // namespace bdrive_hardware_interface

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
  bdrive_hardware_interface::BDriveHardwareInterface, hardware_interface::SystemInterface)
