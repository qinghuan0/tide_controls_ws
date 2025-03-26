/*
 * @Author: qinghuan 1484237245@qq.com
 * @Date: 2025-01-24 10:47:11
 * @FilePath: /TideControls/src/tide_controllers/tide_gimbal_controller/include/tide_gimbal_controller.hpp
 * @Description:
 *
 * Copyright (c) 2025 by qinghuan, All Rights Reserved.
 */

#ifndef TIDE_GIMBAL_CONTROLLER_HPP_
#define TIDE_GIMBAL_CONTROLLER_HPP_

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

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include "tf2_msgs/msg/tf_message.hpp"

#include "control_msgs/msg/multi_dof_state_stamped.hpp"
#include "tide_msgs/msg/gimbal_cmd.hpp"
#include "tide_msgs/msg/gimbal_state.hpp"
#include "tide_msgs/msg/shooter_cmd.hpp"
#include "tide_msgs/msg/shooter_state.hpp"
#include "bullet_solver.hpp"

#include "gimbal_controller_parameters.hpp"

namespace tide_gimbal_controller
{
class TideGimbalController : public controller_interface::ControllerInterface
{
public:
  TideGimbalController();

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
  using CMD = tide_msgs::msg::GimbalCmd;
  using STATE = tide_msgs::msg::GimbalState;
  using Tracker = tide_msgs::msg::Target;

  using ShooterCmd = tide_msgs::msg::ShooterCmd;

  std::shared_ptr<control_toolbox::PidROS> pid_pitch_pos_, pid_yaw_pos_;

  std::unique_ptr<const hardware_interface::LoanedStateInterface> pitch_state_interface_{ nullptr };
  std::unique_ptr<const hardware_interface::LoanedStateInterface> yaw_state_interface_{ nullptr };
  std::unique_ptr<hardware_interface::LoanedCommandInterface> pitch_command_interface_{ nullptr };
  std::unique_ptr<hardware_interface::LoanedCommandInterface> yaw_command_interface_{ nullptr };

  uint8_t mode_{ 0 };
  uint8_t last_mode_{ 0 };
  bool open_loop_{ false };
  double pitch_pos_cmd_{ 0.0 }, yaw_pos_cmd_{ 0.0 };
  double pitch_pos_fb_{ 0.0 }, yaw_pos_fb_{ 0.0 };
  int16_t pitch_reverse_{ 1 }, yaw_reverse_{ 1 };
  double pitch_cmd_{ 0.0 }, yaw_cmd_{ 0.0 };

  std::shared_ptr<BulletSolver> bullet_solver_;
  rclcpp::Time last_tracking_time_;

  std::shared_ptr<ParamListener> param_listener_;
  Params params_;

  void update_parameters();
  std::pair<double, double> sentry_mode();
  std::pair<double, double> auto_aim_mode();

  std::vector<rclcpp::Subscription<Tracker>::SharedPtr> auto_aim_subs_;

  std::vector<std::shared_ptr<realtime_tools::RealtimeBuffer<std::shared_ptr<Tracker>>>>
      tracker_targets_;

  std::shared_ptr<Tracker> select_best_target();

  rclcpp::Subscription<STATE>::SharedPtr ex_state_sub_ = nullptr;
  realtime_tools::RealtimeBuffer<std::shared_ptr<STATE>> ex_state_rt_{ nullptr };

  rclcpp::Subscription<CMD>::SharedPtr cmd_sub_ = nullptr;
  realtime_tools::RealtimeBuffer<std::shared_ptr<CMD>> recv_cmd_ptr_{ nullptr };

  std::shared_ptr<realtime_tools::RealtimePublisher<STATE>> rt_gimbal_state_pub_{ nullptr };
  std::shared_ptr<realtime_tools::RealtimePublisher<ShooterCmd>> rt_shooter_cmd_pub_{ nullptr };
};

}  // namespace tide_gimbal_controller

#endif  // TIDE_GIMBAL_CONTROLLER_HPP_
