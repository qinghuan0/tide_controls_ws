/*
 * @Author: qinghuan 1484237245@qq.com
 * @Date: 2025-01-24 10:47:11
 * @FilePath: /TideControls/src/tide_controllers/tide_shooter_controller/include/tide_shooter_controller.hpp
 * @Description:
 *
 * Copyright (c) 2025 by qinghuan, All Rights Reserved.
 */

#ifndef TIDE_SHOOTER_CONTROLLER_HPP_
#define TIDE_SHOOTER_CONTROLLER_HPP_

#include <chrono>
#include <cmath>
#include <memory>
#include <string>
#include <vector>
#include <chrono>

#include "hardware_interface/handle.hpp"
#include "controller_interface/controller_interface.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "realtime_tools/realtime_box.hpp"
#include "realtime_tools/realtime_publisher.hpp"
#include "tf2_msgs/msg/tf_message.hpp"
#include <control_toolbox/pid_ros.hpp>
#include "realtime_tools/realtime_buffer.hpp"

#include "control_msgs/msg/multi_dof_state_stamped.hpp"
#include "tide_msgs/msg/shooter_cmd.hpp"
#include "tide_msgs/msg/shooter_state.hpp"
#include "tide_msgs/msg/power_heat_data.hpp"

#include "shooter_controller_parameters.hpp"

namespace tide_shooter_controller
{
// TODO: calculate the real value
constexpr double bullet_vel2vel = 28.7;
constexpr double shooting_freq2vel = 10.0;
constexpr double vel2bullet_vel = 0.1;     // 1 / bullet_vel2vel
constexpr double vel2shooting_freq = 0.1;  // 1 / shooting_freq2vel

class TideShooterController : public controller_interface::ControllerInterface
{
public:
  TideShooterController();

  controller_interface::CallbackReturn on_init() override;

  controller_interface::InterfaceConfiguration command_interface_configuration() const override;

  controller_interface::InterfaceConfiguration state_interface_configuration() const override;

  controller_interface::return_type update(const rclcpp::Time& time,
                                           const rclcpp::Duration& period) override;

  controller_interface::CallbackReturn
  on_configure(const rclcpp_lifecycle::State& previous_state) override;

  controller_interface::CallbackReturn
  on_activate(const rclcpp_lifecycle::State& previous_state) override;

  controller_interface::CallbackReturn
  on_deactivate(const rclcpp_lifecycle::State& previous_state) override;

  controller_interface::CallbackReturn
  on_cleanup(const rclcpp_lifecycle::State& previous_state) override;

  controller_interface::CallbackReturn
  on_error(const rclcpp_lifecycle::State& previous_state) override;

private:
  using CMD = tide_msgs::msg::ShooterCmd;
  using STATE = tide_msgs::msg::ShooterState;
  using RefereeShooterHeat = tide_msgs::msg::PowerHeatData;

  std::shared_ptr<ParamListener> param_listener_;
  Params params_;

  size_t friction_wheel_size_;
  bool open_loop_;
  uint8_t mode_;
  double bullet_vel_cmd_, bullet_vel_real_;
  double shooting_freq_cmd_, shooting_freq_real_;
  uint16_t cooling_heat_{ 0 };

  std::vector<std::shared_ptr<control_toolbox::PidROS>> friction_wheel_pid_;
  std::vector<double> friction_wheel_fb_;
  std::vector<bool> friction_wheel_reverse_;
  double loader_fb_;
  bool loader_reverse_;
  std::shared_ptr<control_toolbox::PidROS> loader_pid_;

  void update_parameters();

  rclcpp::Subscription<RefereeShooterHeat>::SharedPtr referee_shooter_heat_sub_ = nullptr;
  rclcpp::Subscription<CMD>::SharedPtr cmd_sub_ = nullptr;
  realtime_tools::RealtimeBox<std::shared_ptr<CMD>> recv_cmd_ptr_{ nullptr };

  std::vector<std::unique_ptr<const hardware_interface::LoanedStateInterface>>
      friction_state_interface_;
  std::unique_ptr<const hardware_interface::LoanedStateInterface> loader_state_interface_;

  std::vector<std::unique_ptr<hardware_interface::LoanedCommandInterface>> friction_cmd_interface_;
  std::unique_ptr<hardware_interface::LoanedCommandInterface> loader_cmd_interface_;

  std::shared_ptr<rclcpp::Publisher<STATE>> shooter_state_pub_{ nullptr };
  std::shared_ptr<realtime_tools::RealtimePublisher<STATE>> rt_shooter_state_pub_{ nullptr };
};

}  // namespace tide_shooter_controller

#endif  // TIDE_SHOOTER_CONTROLLER_HPP_
