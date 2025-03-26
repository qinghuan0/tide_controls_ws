/*
 * @Author: qinghuan 1484237245@qq.com
 * @Date: 2025-02-05 16:07:31
 * @FilePath: /TideControls/src/tide_control/tide_hw_interface/include/tide_hw_interface.hpp
 * @Description:
 *
 * Copyright (c) 2025 by qinghuan, All Rights Reserved.
 */
#ifndef TIDE_HARDWARE_INTERFACE_HPP
#define TIDE_HARDWARE_INTERFACE_HPP

#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tide_motor.hpp"
#include "socket_can/socket_can_sender.hpp"
#include "socket_can/socket_can_receiver.hpp"

#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

namespace tide_hw_interface
{

class CanDevice
{
public:
  using ReceiveCallback = std::function<void(const std::array<uint8_t, 8>&, uint32_t)>;

  CanDevice(const std::string& interface_name, ReceiveCallback callback);
  ~CanDevice();

  std::string interface;
  std::shared_ptr<SocketCanSender> sender;
  std::shared_ptr<SocketCanReceiver> receiver;
  std::shared_ptr<std::thread> receiver_thread;
  std::shared_ptr<std::atomic<bool>> thread_running;
  std::array<std::array<uint8_t, 8>, 3> tx_buff{};

private:
  void receiveLoop();
  ReceiveCallback receive_callback_;
};

class TideHardwareInterface : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(TideHardwareInterface)
  TideHardwareInterface();
  virtual ~TideHardwareInterface() = default;

  hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo& info) override;

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  hardware_interface::return_type read(const rclcpp::Time& time,
                                       const rclcpp::Duration& period) override;

  hardware_interface::return_type write(const rclcpp::Time& time,
                                        const rclcpp::Duration& period) override;

  hardware_interface::CallbackReturn
  on_configure(const rclcpp_lifecycle::State& previous_state) override;

  hardware_interface::CallbackReturn
  on_cleanup(const rclcpp_lifecycle::State& previous_state) override;

  hardware_interface::CallbackReturn
  on_activate(const rclcpp_lifecycle::State& previous_state) override;

  hardware_interface::CallbackReturn
  on_deactivate(const rclcpp_lifecycle::State& previous_state) override;

private:
  bool sendCanFrame(std::shared_ptr<CanDevice> device, const uint8_t* data, size_t len, uint32_t id);
  void configureMotorCan(std::shared_ptr<DJI_Motor> motor);
  void stopMotors();

  std::vector<std::shared_ptr<DJI_Motor>> motors_;
  std::vector<std::shared_ptr<CanDevice>> can_devices_;

  size_t joint_count{ 0 };
  size_t can_device_count_{ 0 };

  std::vector<double> cmd_positions_;
  std::vector<double> cmd_velocities_;
  std::vector<double> state_positions_;
  std::vector<double> state_velocities_;
  std::vector<double> state_currents_;
  std::vector<double> state_temperatures_;

  bool need_calibration_{ false };
  bool enable_virtual_control_{ false };
  std::atomic<bool> stop_thread_{ false };
  mutable std::mutex device_mutex_;
};

}  // namespace tide_hw_interface

#endif  // TIDE_HARDWARE_INTERFACE_HPP
