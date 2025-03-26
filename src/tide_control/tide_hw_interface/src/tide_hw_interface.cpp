/*
 * @Author: qinghuan 1484237245@qq.com
 * @Date: 2025-02-08 11:29:40
 * @FilePath: /TideControls/src/tide_control/tide_hw_interface/src/tide_hw_interface.cpp
 * @Description:
 *
 * Copyright (c) 2025 by qinghuan, All Rights Reserved.
 */
#include "tide_hw_interface.hpp"
#include <chrono>
#include <cmath>
#include <limits>
#include <vector>
#include <string>
#include <set>
#include <iomanip>

namespace tide_hw_interface
{

CanDevice::CanDevice(const std::string& interface_name, ReceiveCallback callback)
  : interface(interface_name)
  , thread_running(std::make_shared<std::atomic<bool>>(true))
  , receive_callback_(std::move(callback))
{
  sender = std::make_shared<SocketCanSender>(interface_name, false);
  receiver = std::make_shared<SocketCanReceiver>(interface_name, false);
  receiver_thread = std::make_shared<std::thread>(&CanDevice::receiveLoop, this);
}

CanDevice::~CanDevice()
{
  if (thread_running)
  {
    thread_running->store(false);
    if (receiver_thread && receiver_thread->joinable())
    {
      receiver_thread->join();
    }
  }
}

void CanDevice::receiveLoop()
{
  std::array<uint8_t, 8> rx_data;
  while (thread_running->load() && rclcpp::ok())
  {
    try
    {
      auto rx_frame = receiver->receive(rx_data.data(), std::chrono::milliseconds(1));
      receive_callback_(rx_data, rx_frame.get());
    }
    catch (const SocketCanTimeout&)
    {
      continue;
    }
    catch (const std::exception& e)
    {
      if (thread_running->load())
      {
        RCLCPP_ERROR(rclcpp::get_logger("CanDevice"), "Error in receive loop: %s", e.what());
      }
      break;
    }
  }
}

TideHardwareInterface::TideHardwareInterface() : hardware_interface::SystemInterface() {}

hardware_interface::CallbackReturn
TideHardwareInterface::on_init(const hardware_interface::HardwareInfo& info)
{
  if (hardware_interface::SystemInterface::on_init(info) !=
      hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  joint_count = info_.joints.size();
  state_positions_.resize(joint_count, std::numeric_limits<double>::quiet_NaN());
  state_velocities_.resize(joint_count, std::numeric_limits<double>::quiet_NaN());
  state_currents_.resize(joint_count, std::numeric_limits<double>::quiet_NaN());
  state_temperatures_.resize(joint_count, std::numeric_limits<double>::quiet_NaN());

  cmd_positions_.resize(joint_count, std::numeric_limits<double>::quiet_NaN());
  cmd_velocities_.resize(joint_count, std::numeric_limits<double>::quiet_NaN());

  enable_virtual_control_ = info_.hardware_parameters.at("enable_virtual_control") == "true";
  need_calibration_ = info_.hardware_parameters.at("need_calibration") == "true";
  can_device_count_ = std::stoi(info_.hardware_parameters.at("can_device_count"));

  for (const auto& joint : info_.joints)
  {
    Motor_Config_t config;
    config.motor_name = joint.name;

    for (const auto& [key, value] : joint.parameters)
    {
      if (key == "can_bus")
        config.can_bus = value;
      else if (key == "tx_id")
      {
        config.tx_id = std::stoi(value);
      }
      else if (key == "rx_id")
      {
        config.rx_id = std::stoi(value);
      }
      else if (key == "motor_type")
      {
        if (value == "M2006")
        {
          config.motor_type = M2006;
        }
        else if (value == "M3508")
        {
          config.motor_type = M3508;
        }
        else if (value == "GM6020")
        {
          config.motor_type = GM6020;
        }
        else if (value == "VIRTUAL_JOINT")
        {
          config.motor_type = VIRTUAL_JOINT;
        }
        else
        {
          RCLCPP_ERROR(rclcpp::get_logger("TideHardwareInterface"), "Unknown motor type: %s",
                       value.c_str());
        }
      }
      else if (key == "offset")
        config.offset = std::stoi(value);
    }

    auto motor = std::make_shared<DJI_Motor>(config);
    configureMotorCan(motor);
    motors_.push_back(motor);
  }

  if (enable_virtual_control_)
  {
    return hardware_interface::CallbackReturn::SUCCESS;
  }
  for (size_t i = 0; i < can_device_count_; i++)
  {
    std::string can_device_name = "can" + std::to_string(i);
    const std::string bus_name = can_device_name;
    auto receive_callback = [this, bus_name](const std::array<uint8_t, 8>& data, uint32_t can_id) {
      rclcpp::Time current_time = rclcpp::Clock().now();
      for (auto& motor : motors_)
      {
        if (motor->config_.can_bus == bus_name && motor->config_.rx_id == can_id)
        {
          motor->status = MOTOR_OK;
          motor->rx_buff = data;
          motor->decode_feedback();
          motor->update_timestamp(current_time);
          break;
        }
      }
    };
    can_devices_.push_back(std::make_shared<CanDevice>(can_device_name, receive_callback));
  }

  RCLCPP_INFO(rclcpp::get_logger("TideHardwareInterface"),
              "Successful loaded %ld motors and %ld can devices", joint_count, can_device_count_);

  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> TideHardwareInterface::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> interfaces;

  for (size_t i = 0; i < joint_count; i++)
  {
    for (const auto& state_interface : info_.joints[i].state_interfaces)
    {
      if (state_interface.name == "position")
      {
        interfaces.emplace_back(info_.joints[i].name, state_interface.name, &state_positions_[i]);
      }
      else if (state_interface.name == "velocity")
      {
        interfaces.emplace_back(info_.joints[i].name, state_interface.name, &state_velocities_[i]);
      }
      else if (state_interface.name == "effort")
      {
        interfaces.emplace_back(info_.joints[i].name, state_interface.name, &state_currents_[i]);
      }
    }
  }
  return interfaces;
}

std::vector<hardware_interface::CommandInterface> TideHardwareInterface::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> interfaces;

  for (size_t i = 0; i < joint_count; i++)
  {
    for (const auto& command_interface : info_.joints[i].command_interfaces)
    {
      if (command_interface.name == "position")
      {
        interfaces.emplace_back(info_.joints[i].name, command_interface.name, &cmd_positions_[i]);
      }
      else if (command_interface.name == "velocity")
      {
        interfaces.emplace_back(info_.joints[i].name, command_interface.name, &cmd_velocities_[i]);
      }
    }
  }
  return interfaces;
}

hardware_interface::CallbackReturn
TideHardwareInterface::on_configure(const rclcpp_lifecycle::State& /*previous_state*/)
{
  std::fill(state_positions_.begin(), state_positions_.end(), 0.0);
  std::fill(state_velocities_.begin(), state_velocities_.end(), 0.0);
  std::fill(state_currents_.begin(), state_currents_.end(), 0.0);
  std::fill(state_temperatures_.begin(), state_temperatures_.end(), 0.0);

  std::fill(cmd_positions_.begin(), cmd_positions_.end(), 0.0);
  std::fill(cmd_velocities_.begin(), cmd_velocities_.end(), 0.0);

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn
TideHardwareInterface::on_activate(const rclcpp_lifecycle::State& /*previous_state*/)
{
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn
TideHardwareInterface::on_deactivate(const rclcpp_lifecycle::State& /*previous_state*/)
{
  try
  {
    stopMotors();
    return hardware_interface::CallbackReturn::SUCCESS;
  }
  catch (const std::exception& e)
  {
    RCLCPP_ERROR(rclcpp::get_logger("TideHardwareInterface"), "Error in on_deactivate: %s",
                 e.what());
    return hardware_interface::CallbackReturn::ERROR;
  }
}

hardware_interface::CallbackReturn
TideHardwareInterface::on_cleanup(const rclcpp_lifecycle::State& /*previous_state*/)
{
  try
  {
    stop_thread_ = true;
    can_devices_.clear();
    motors_.clear();
    return hardware_interface::CallbackReturn::SUCCESS;
  }
  catch (const std::exception& e)
  {
    RCLCPP_ERROR(rclcpp::get_logger("TideHardwareInterface"), "Error in on_cleanup: %s", e.what());
    return hardware_interface::CallbackReturn::ERROR;
  }
}

void TideHardwareInterface::configureMotorCan(std::shared_ptr<DJI_Motor> motor)
{
  switch (motor->config_.motor_type)
  {
    case M2006:
    case M3508:
      motor->config_.rx_id = 0x200 + motor->config_.tx_id;
      if (motor->config_.tx_id <= 4)
      {
        motor->config_.identifier = 0x200;
      }
      else
      {
        motor->config_.tx_id -= 4;
        motor->config_.identifier = 0x1ff;
      }
      break;
    case GM6020:
      motor->config_.rx_id = 0x204 + motor->config_.tx_id;
      if (motor->config_.tx_id <= 4)
      {
        motor->config_.identifier = 0x1ff;
      }
      else
      {
        motor->config_.tx_id -= 4;
        motor->config_.identifier = 0x2ff;
      }
      break;
    default:
      return;
  }
}

bool TideHardwareInterface::sendCanFrame(std::shared_ptr<CanDevice> device, const uint8_t* data,
                                         size_t len, uint32_t id)
{
  try
  {
    CanId send_id(id, 0, FrameType::DATA, StandardFrame);
    device->sender->send(data, len, send_id, std::chrono::milliseconds(1));
  }
  catch (const std::exception& e)
  {
    RCLCPP_ERROR(rclcpp::get_logger("TideHardwareInterface"), "Failed to send CAN frame: %s",
                 e.what());
    return false;
  }
  return true;
}

void TideHardwareInterface::stopMotors()
{
  for (auto& motor : motors_)
  {
    motor->stop();
  }
}

hardware_interface::return_type TideHardwareInterface::read(const rclcpp::Time& time,
                                                            const rclcpp::Duration& period)
{
  auto current_time = time;

  for (size_t i = 0; i < joint_count; i++)
  {
    auto& motor = motors_[i];
    if (enable_virtual_control_ || motor->config_.motor_type == VIRTUAL_JOINT)
    {
      if (!std::isnan(cmd_positions_[i]) &&
          info_.joints[i].command_interfaces[0].name == "position")
      {
        state_positions_[i] = cmd_positions_[i];
      }
      else if (!std::isnan(cmd_velocities_[i]) &&
               info_.joints[i].command_interfaces[0].name == "velocity")
      {
        state_positions_[i] += cmd_velocities_[i] * period.seconds();
        state_velocities_[i] = cmd_velocities_[i];
      }
    }
    else
    {
      motor->check_connection(current_time);

      state_positions_[i] = motor->angle_current;
      state_velocities_[i] = motor->measure.speed_aps;
      state_currents_[i] = motor->measure.real_current;
      state_temperatures_[i] = motor->measure.temperature;
    }
  }
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type TideHardwareInterface::write(const rclcpp::Time& time,
                                                             const rclcpp::Duration& /*period*/)
{
  if (need_calibration_)
  {
    for (const auto& motor : motors_)
    {
      std::stringstream ss;
      ss << std::left << std::setw(15) << motor->config_.motor_name << "Encoder: " << std::setw(6)
         << motor->measure.ecd;
      RCLCPP_INFO(rclcpp::get_logger("TideHardwareInterface"), "%s", ss.str().c_str());
    }
    return hardware_interface::return_type::OK;
  }

  for (auto can_device : can_devices_)
  {
    for (size_t i = 0; i < 3; i++)
    {
      can_device->tx_buff[i].fill(0);
    }
  }

  for (size_t i = 0; i < joint_count; i++)
  {
    double command = 0.0;
    if (!std::isnan(cmd_positions_[i]) && info_.joints[i].command_interfaces[0].name == "position")
    {
      command = (cmd_positions_[i]);
    }
    else if (!std::isnan(cmd_velocities_[i]) &&
             info_.joints[i].command_interfaces[0].name == "velocity")
    {
      command = cmd_velocities_[i];
    }

    auto motor = motors_[i];
    motor->output = static_cast<int16_t>(
        std::clamp(command, -30000.0, 30000.0));  // TODO:这里应给不同的电机类型使用不同的限幅

    auto current_time = time;
    if (motor->check_connection(current_time))
    {
      for (auto can_device : can_devices_)
      {
        if ((motor->config_.can_bus == can_device->interface) &&
            motor->config_.motor_type != VIRTUAL_JOINT)
        {
          size_t buff_index = (motor->config_.identifier == 0x200) ? 0 :
                              (motor->config_.identifier == 0x1ff) ? 1 :
                                                                     2;
          size_t data_index = (motor->config_.tx_id - 1) * 2;

          can_device->tx_buff[buff_index][data_index] = motor->output >> 8;
          can_device->tx_buff[buff_index][data_index + 1] = motor->output & 0xff;

          break;
        }
      }
    }
  }

  for (auto can_device : can_devices_)
  {
    for (size_t i = 0; i < 3; i++)
    {
      auto id = (i == 0) ? 0x200 : (i == 1) ? 0x1ff : 0x2ff;
      bool result = sendCanFrame(can_device, can_device->tx_buff[i].data(), 8, id);
      if (!result)
      {
        can_device->tx_buff[i].fill(0);
      }
    }
  }
  return hardware_interface::return_type::OK;
}

}  // namespace tide_hw_interface

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(tide_hw_interface::TideHardwareInterface, hardware_interface::SystemInterface)
