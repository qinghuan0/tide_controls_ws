/*
 * @Author: qinghuan 1484237245@qq.com
 * @Date: 2025-01-26 16:04:35
 * @FilePath: /TideControls/src/tide_controllers/tide_gimbal_controller/src/tide_gimbal_controller.cpp
 * @Description:
 *
 * Copyright (c) 2025 by qinghuan, All Rights Reserved.
 */
#include "tide_gimbal_controller.hpp"
#include "angles/angles.h"

namespace tide_gimbal_controller
{
using controller_interface::interface_configuration_type;
using controller_interface::InterfaceConfiguration;

TideGimbalController::TideGimbalController() : controller_interface::ControllerInterface() {}

controller_interface::CallbackReturn TideGimbalController::on_init()
{
  try
  {
    param_listener_ = std::make_shared<ParamListener>(get_node());
    params_ = param_listener_->get_params();

    auto cmd = std::make_shared<CMD>();
    cmd->pitch_ref = 0.0;
    cmd->yaw_ref = 0.0;
    cmd->mode = 0;

    recv_cmd_ptr_.initRT(cmd);

    auto ex_state = std::make_shared<STATE>();
    ex_state->pitch_cur = 0.0;
    ex_state->yaw_cur = 0.0;

    ex_state_rt_.initRT(ex_state);

    last_tracking_time_ = this->get_node()->now();
  }
  catch (const std::exception& e)
  {
    fprintf(stderr, "Exception thrown during init stage with message: %s \n", e.what());
    return controller_interface::CallbackReturn::ERROR;
  }

  return controller_interface::CallbackReturn::SUCCESS;
}

InterfaceConfiguration TideGimbalController::command_interface_configuration() const
{
  std::vector<std::string> joint_names;
  joint_names.push_back(params_.pitch.joint + "/position");
  joint_names.push_back(params_.yaw.joint + "/position");

  return { interface_configuration_type::INDIVIDUAL, joint_names };
}

InterfaceConfiguration TideGimbalController::state_interface_configuration() const
{
  std::vector<std::string> joint_names;
  joint_names.push_back(params_.pitch.joint + "/position");
  joint_names.push_back(params_.yaw.joint + "/position");

  return { interface_configuration_type::INDIVIDUAL, joint_names };
}

controller_interface::CallbackReturn
TideGimbalController::on_configure(const rclcpp_lifecycle::State& /*previous_state*/)
{
  params_ = param_listener_->get_params();
  open_loop_ = params_.open_loop;
  pid_pitch_pos_ = std::make_shared<control_toolbox::PidROS>(get_node(), "pitch.pid", true);
  pid_yaw_pos_ = std::make_shared<control_toolbox::PidROS>(get_node(), "yaw.pid", true);

  std::string joint_name = params_.pitch.joint;
  std::string link_name;

  size_t pos = joint_name.find("_joint");
  if (pos != std::string::npos)
  {
    link_name = joint_name.substr(0, pos) + "_link";
  }
  else
  {
    link_name = joint_name + "_link";
  }

  bullet_solver_ =
      std::make_shared<BulletSolver>(params_.bullet_solver.resistance_coff, params_.bullet_solver.g,
                                     link_name, params_.bullet_solver.time_delay,
                                     params_.bullet_solver.cam_offset);

  if (!pid_pitch_pos_->initPid() || !pid_yaw_pos_->initPid())
  {
    RCLCPP_ERROR(get_node()->get_logger(), "Failed to initialize PIDs.");
    return controller_interface::CallbackReturn::ERROR;
  }

  cmd_sub_ = get_node()->create_subscription<CMD>("~/gimbal_cmd", rclcpp::SystemDefaultsQoS(),
                                                  [this](const std::shared_ptr<CMD> msg) -> void {
                                                    recv_cmd_ptr_.writeFromNonRT(msg);
                                                  });

  ex_state_sub_ = get_node()->create_subscription<STATE>(
      "~/ex_state_interface", rclcpp::SensorDataQoS(),
      [this](const std::shared_ptr<STATE> msg) -> void { ex_state_rt_.writeFromNonRT(msg); });

  if (params_.tracker_topic.size() > 0)
  {
    for (const auto& topic : params_.tracker_topic)
    {
      auto tracker_buffer =
          std::make_shared<realtime_tools::RealtimeBuffer<std::shared_ptr<Tracker>>>();
      tracker_targets_.push_back(tracker_buffer);

      int idx = tracker_targets_.size() - 1;
      auto sub = get_node()->create_subscription<Tracker>(
          topic, rclcpp::SensorDataQoS(), [this, idx](const Tracker::SharedPtr msg) -> void {
            tracker_targets_[idx]->writeFromNonRT(msg);
          });

      auto_aim_subs_.push_back(sub);
      RCLCPP_INFO(get_node()->get_logger(), "Subscribed to tracker topic: %s", topic.c_str());
    }
  }

  auto marker_pub = get_node()->create_publisher<visualization_msgs::msg::Marker>(
      "~/bullet_traj", rclcpp::SystemDefaultsQoS());

  bullet_solver_->rt_bullet_traj_pub_ =
      std::make_shared<realtime_tools::RealtimePublisher<visualization_msgs::msg::Marker>>(
          marker_pub);

  auto aiming_point_pub = get_node()->create_publisher<visualization_msgs::msg::Marker>(
      "~/aiming_point", rclcpp::SystemDefaultsQoS());

  bullet_solver_->rt_aiming_point_pub_ =
      std::make_shared<realtime_tools::RealtimePublisher<visualization_msgs::msg::Marker>>(
          aiming_point_pub);

  auto gimbal_state_pub =
      get_node()->create_publisher<STATE>("~/gimbal_state", rclcpp::SystemDefaultsQoS());
  rt_gimbal_state_pub_ =
      std::make_shared<realtime_tools::RealtimePublisher<STATE>>(gimbal_state_pub);

  auto shooter_cmd_pub = get_node()->create_publisher<ShooterCmd>(params_.shooter_cmd_topic,
                                                                  rclcpp::SystemDefaultsQoS());
  rt_shooter_cmd_pub_ =
      std::make_shared<realtime_tools::RealtimePublisher<ShooterCmd>>(shooter_cmd_pub);
  return controller_interface::CallbackReturn::SUCCESS;
}

void TideGimbalController::update_parameters()
{
  if (!param_listener_->is_old(params_))
  {
    return;
  }
  params_ = param_listener_->get_params();
}

controller_interface::return_type TideGimbalController::update(const rclcpp::Time& time,
                                                               const rclcpp::Duration& period)
{
  update_parameters();
  auto logger = get_node()->get_logger();

  auto cmd = *recv_cmd_ptr_.readFromRT();
  last_mode_ = mode_;
  mode_ = cmd->mode;
  double pitch_fb = 0.0, yaw_fb = 0.0;
  double pitch_cmd = 0.0, yaw_cmd = 0.0;

  if (params_.use_external_state_interface)
  {
    open_loop_ = true;
    mode_ = 2;
    auto ex_state = *ex_state_rt_.readFromRT();
    if (ex_state != nullptr)
    {
      pitch_fb = ex_state->pitch_cur;
      yaw_fb = ex_state->yaw_cur;
    }
  }
  else
  {
    pitch_fb = pitch_state_interface_->get_value();
    yaw_fb = yaw_state_interface_->get_value();
  }

  pitch_pos_fb_ = pitch_fb;
  yaw_pos_fb_ = yaw_fb;

  if (!tracker_targets_.empty())
  {
    auto best_target = select_best_target();
    if (best_target)
    {
      bullet_solver_->tracker_target_.writeFromNonRT(best_target);
      bullet_solver_->update();
    }
  }

  switch (mode_)
  {
    case 0:
    {
      pitch_cmd = cmd->pitch_ref;
      yaw_cmd = cmd->yaw_ref;
      break;
    }
    case 1:
    {
      auto result = sentry_mode();
      pitch_cmd = result.first;
      yaw_cmd = result.second;
      break;
    }
    case 2:
    {
      auto result = auto_aim_mode();
      pitch_cmd = result.first;
      yaw_cmd = result.second;
      break;
    }
    default:
      break;
  }

  pitch_pos_cmd_ = std::clamp(pitch_cmd, params_.pitch.min, params_.pitch.max);
  yaw_pos_cmd_ = std::clamp(yaw_cmd, params_.yaw.min, params_.yaw.max);

  pitch_pos_cmd_ = params_.pitch.reverse ? -pitch_pos_cmd_ : pitch_pos_cmd_;
  yaw_pos_cmd_ = params_.yaw.reverse ? -yaw_pos_cmd_ : yaw_pos_cmd_;

  if (open_loop_)
  {
    if (params_.use_external_state_interface)
    {
      pitch_command_interface_->set_value(pitch_pos_fb_);
      yaw_command_interface_->set_value(yaw_pos_fb_);
    }
    else
    {
      pitch_command_interface_->set_value(pitch_pos_cmd_);
      yaw_command_interface_->set_value(yaw_pos_cmd_);
    }
  }
  else
  {
    double pitch_error = pitch_pos_cmd_ - pitch_pos_fb_;
    double yaw_error = yaw_pos_cmd_ - yaw_pos_fb_;

    pitch_error = angles::shortest_angular_distance(pitch_pos_fb_, pitch_pos_cmd_);
    yaw_error = angles::shortest_angular_distance(yaw_pos_fb_, yaw_pos_cmd_);

    double pitch_cmd_tmp = pid_pitch_pos_->computeCommand(pitch_error, period);
    double yaw_cmd_tmp = pid_yaw_pos_->computeCommand(yaw_error, period);

    pitch_command_interface_->set_value(pitch_cmd_tmp);
    yaw_command_interface_->set_value(yaw_cmd_tmp);
  }

  if (rt_gimbal_state_pub_->trylock())
  {
    auto& gimbal_state = rt_gimbal_state_pub_->msg_;
    gimbal_state.header.stamp = time;
    gimbal_state.mode = mode_;
    gimbal_state.is_tracking = bullet_solver_->tracking_;
    gimbal_state.pitch_ref = pitch_pos_cmd_;
    gimbal_state.yaw_ref = yaw_pos_cmd_;
    gimbal_state.pitch_cur = pitch_pos_fb_;
    gimbal_state.yaw_cur = yaw_pos_fb_;
    rt_gimbal_state_pub_->unlockAndPublish();
  }

  return controller_interface::return_type::OK;
}

controller_interface::CallbackReturn
TideGimbalController::on_activate(const rclcpp_lifecycle::State& /*previous_state*/)
{
  pitch_state_interface_ = std::make_unique<const hardware_interface::LoanedStateInterface>(
      std::move(state_interfaces_[0]));
  yaw_state_interface_ = std::make_unique<const hardware_interface::LoanedStateInterface>(
      std::move(state_interfaces_[1]));

  pitch_command_interface_ = std::make_unique<hardware_interface::LoanedCommandInterface>(
      std::move(command_interfaces_[0]));
  yaw_command_interface_ = std::make_unique<hardware_interface::LoanedCommandInterface>(
      std::move(command_interfaces_[1]));

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn
TideGimbalController::on_deactivate(const rclcpp_lifecycle::State& /*previous_state*/)
{
  pitch_state_interface_.reset();
  yaw_state_interface_.reset();
  pitch_command_interface_.reset();
  yaw_command_interface_.reset();
  bullet_solver_.reset();

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn
TideGimbalController::on_cleanup(const rclcpp_lifecycle::State& previous_state)
{
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn
TideGimbalController::on_error(const rclcpp_lifecycle::State& previous_state)
{
  return controller_interface::CallbackReturn::SUCCESS;
}

std::pair<double, double> TideGimbalController::sentry_mode()
{
  auto lost_time = get_node()->now() - last_tracking_time_;
  rt_shooter_cmd_pub_->msg_.bullet_velocity = 23.0;
  rt_shooter_cmd_pub_->msg_.shooting_freq = 30.0;

  rt_shooter_cmd_pub_->msg_.mode = 0;
  if (bullet_solver_->tracking_ || lost_time.seconds() < 2.0)
  {
    return auto_aim_mode();
  }

  if (mode_ != last_mode_)
  {
    pitch_cmd_ = pitch_pos_fb_;
    yaw_cmd_ = yaw_pos_fb_;
  }

  if (pitch_cmd_ > params_.pitch.scan_range[1] || pitch_cmd_ < params_.pitch.scan_range[0])
  {
    pitch_reverse_ *= -1;
  }

  if (yaw_cmd_ > params_.yaw.scan_range[1] || yaw_cmd_ < params_.yaw.scan_range[0])
  {
    yaw_reverse_ *= -1;
  }

  pitch_cmd_ += pitch_reverse_ * params_.pitch.scan_add;
  yaw_cmd_ += yaw_reverse_ * params_.yaw.scan_add;

  if (rt_shooter_cmd_pub_->trylock())
  {
    rt_shooter_cmd_pub_->unlockAndPublish();
  }

  return std::make_pair(pitch_cmd_, yaw_cmd_);
}

std::pair<double, double> TideGimbalController::auto_aim_mode()
{
  if (!bullet_solver_->tracking_)
  {
    if (rt_shooter_cmd_pub_->trylock())
    {
      rt_shooter_cmd_pub_->unlockAndPublish();
    }
    return std::make_pair(pitch_pos_fb_, yaw_pos_fb_);
  }

  bool solve_success = bullet_solver_->solve(23.0, pitch_pos_fb_, yaw_pos_fb_);
  bullet_solver_->ballistic_visualization();

  if (solve_success)
  {
    auto result = bullet_solver_->get_result();

    if (abs(yaw_pos_fb_ - result.second) < 0.06)
    {
      rt_shooter_cmd_pub_->msg_.mode = 1;
    }
    return result;
  }
  else
  {
    if (rt_shooter_cmd_pub_->trylock())
    {
      rt_shooter_cmd_pub_->unlockAndPublish();
    }
    return std::make_pair(pitch_pos_fb_, yaw_pos_fb_);
  }
}

// 这段代码的设计思想是给拥有多路感知如哨兵使用的，一个云台订阅多个感知的topic，然后选择最佳的目标进行追踪，后来才发现没考虑到弹道解算，所以暂时废弃，在配置文件的tracker_topic参数只填写与该云台对应的那一路相机自瞄话题即可
std::shared_ptr<TideGimbalController::Tracker> TideGimbalController::select_best_target()
{
  std::shared_ptr<Tracker> best_target = nullptr;
  double min_angle_diff = 9999.99;
  double current_yaw = yaw_pos_fb_;
  bool has_target = false;

  for (const auto& tracker_buffer : tracker_targets_)
  {
    auto target = *tracker_buffer->readFromRT();
    if (target && target->tracking)
    {
      has_target = true;
      last_tracking_time_ = get_node()->now();
      double target_yaw = std::atan2(target->position.y, target->position.x);
      double angle_diff = std::abs(angles::shortest_angular_distance(current_yaw, target_yaw));

      if (angle_diff < min_angle_diff)
      {
        min_angle_diff = angle_diff;
        best_target = target;
      }
    }
  }
  if (!has_target)
  {
    bullet_solver_->tracking_ = false;
  }

  return best_target;
}

}  // namespace tide_gimbal_controller

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(tide_gimbal_controller::TideGimbalController,
                       controller_interface::ControllerInterface)
