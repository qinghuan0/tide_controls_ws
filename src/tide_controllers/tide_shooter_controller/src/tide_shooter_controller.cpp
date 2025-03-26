/*
 * @Author: qinghuan 1484237245@qq.com
 * @Date: 2025-01-26 16:04:35
 * @FilePath: /TideControls/src/tide_controllers/tide_shooter_controller/src/tide_shooter_controller.cpp
 * @Description:
 *
 * Copyright (c) 2025 by qinghuan, All Rights Reserved.
 */
#include "tide_shooter_controller.hpp"
#include "angles/angles.h"

namespace tide_shooter_controller
{
using controller_interface::interface_configuration_type;
using controller_interface::InterfaceConfiguration;

TideShooterController::TideShooterController() : controller_interface::ControllerInterface() {}

controller_interface::CallbackReturn TideShooterController::on_init()
{
  try
  {
    param_listener_ = std::make_shared<ParamListener>(get_node());
    params_ = param_listener_->get_params();
    friction_wheel_size_ = params_.friction_wheels.joint.size();

    RCLCPP_INFO(get_node()->get_logger(), "Loaded %ld friction wheels.", friction_wheel_size_);
  }
  catch (const std::exception& e)
  {
    fprintf(stderr, "Exception thrown during init stage with message: %s \n", e.what());
    return controller_interface::CallbackReturn::ERROR;
  }

  return controller_interface::CallbackReturn::SUCCESS;
}

InterfaceConfiguration TideShooterController::command_interface_configuration() const
{
  std::vector<std::string> joint_names;

  for (size_t i = 0; i < friction_wheel_size_; i++)
  {
    joint_names.push_back(params_.friction_wheels.joint[i] + "/velocity");
  }
  joint_names.push_back(params_.loader.joint + "/velocity");

  return { interface_configuration_type::INDIVIDUAL, joint_names };
}

InterfaceConfiguration TideShooterController::state_interface_configuration() const
{
  std::vector<std::string> joint_names;

  for (size_t i = 0; i < friction_wheel_size_; i++)
  {
    joint_names.push_back(params_.friction_wheels.joint[i] + "/velocity");
  }
  joint_names.push_back(params_.loader.joint + "/velocity");

  return { interface_configuration_type::INDIVIDUAL, joint_names };
}

controller_interface::CallbackReturn
TideShooterController::on_configure(const rclcpp_lifecycle::State& /*previous_state*/)
{
  params_ = param_listener_->get_params();
  open_loop_ = params_.open_loop;
  friction_wheel_pid_.resize(friction_wheel_size_);
  friction_state_interface_.resize(friction_wheel_size_);
  friction_cmd_interface_.resize(friction_wheel_size_);
  friction_wheel_fb_.resize(friction_wheel_size_);
  friction_wheel_reverse_.resize(friction_wheel_size_);

  for (size_t i = 0; i < friction_wheel_size_; i++)
  {
    friction_wheel_pid_[i] =
        std::make_shared<control_toolbox::PidROS>(get_node(), "friction_wheels.pid", true);
    if (!friction_wheel_pid_[i]->initPid())
    {
      RCLCPP_ERROR(get_node()->get_logger(), "Failed to initialize PIDs.");
      return CallbackReturn::FAILURE;
    }

    for (size_t j = 0; j < params_.friction_wheels.reverse.size(); j++)
    {
      if (params_.friction_wheels.joint[i] == params_.friction_wheels.reverse[j])
      {
        friction_wheel_reverse_[i] = true;
        break;
      }
      else
      {
        friction_wheel_reverse_[i] = false;
      }
    }
  }

  loader_pid_ = std::make_shared<control_toolbox::PidROS>(get_node(), "loader.pid", true);
  loader_reverse_ = params_.loader.reverse;
  if (!loader_pid_->initPid())
  {
    RCLCPP_ERROR(get_node()->get_logger(), "Failed to initialize PIDs.");
    return CallbackReturn::FAILURE;
  }

  cmd_sub_ = get_node()->create_subscription<CMD>("~/shooter_cmd", rclcpp::SystemDefaultsQoS(),
                                                  [this](const std::shared_ptr<CMD> msg) -> void {
                                                    auto cmd = std::make_shared<CMD>();
                                                    cmd->mode = msg->mode;
                                                    cmd->bullet_velocity = msg->bullet_velocity;
                                                    cmd->shooting_freq = msg->shooting_freq;
                                                    recv_cmd_ptr_.set(cmd);
                                                  });

  referee_shooter_heat_sub_ = get_node()->create_subscription<RefereeShooterHeat>(
      "/power_heat_data", rclcpp::SystemDefaultsQoS(),
      [this](const std::shared_ptr<RefereeShooterHeat> msg) -> void {
        if (params_.referee_shooter_type == "id1_17mm")
          cooling_heat_ = msg->shooter_id1_17mm_cooling_heat;
        else if (params_.referee_shooter_type == "id2_17mm")
          cooling_heat_ = msg->shooter_id2_17mm_cooling_heat;
        else if (params_.referee_shooter_type == "id1_42mm")
          cooling_heat_ = msg->shooter_id1_42mm_cooling_heat;
        else
          RCLCPP_WARN(get_node()->get_logger(), "Unknown referee shooter type.");
      });
  shooter_state_pub_ =
      get_node()->create_publisher<STATE>("~/shooter_state", rclcpp::SystemDefaultsQoS());

  rt_shooter_state_pub_ =
      std::make_shared<realtime_tools::RealtimePublisher<STATE>>(shooter_state_pub_);

  auto cmd = std::make_shared<CMD>();
  cmd->mode = 0;
  cmd->bullet_velocity = 0;
  cmd->shooting_freq = 0;
  recv_cmd_ptr_.set(cmd);

  return controller_interface::CallbackReturn::SUCCESS;
}

void TideShooterController::update_parameters()
{
  if (!param_listener_->is_old(params_))
  {
    return;
  }
  params_ = param_listener_->get_params();
}

controller_interface::return_type TideShooterController::update(const rclcpp::Time& time,
                                                                const rclcpp::Duration& period)
{
  update_parameters();
  auto logger = get_node()->get_logger();

  std::shared_ptr<CMD> cmd;
  recv_cmd_ptr_.get(cmd);

  if (cmd == nullptr)
  {
    RCLCPP_WARN(logger, "Shooter command received was a nullptr.");
    return controller_interface::return_type::ERROR;
  }

  for (size_t i = 0; i < friction_wheel_size_; i++)
  {
    friction_wheel_fb_[i] = friction_state_interface_[i]->get_value();
    if (std::isnan(friction_wheel_fb_[i]))
    {
      RCLCPP_WARN(logger, "Friction wheel feedback is NaN.");
      return controller_interface::return_type::ERROR;
    }
  }
  loader_fb_ = loader_state_interface_->get_value();
  if (std::isnan(loader_fb_))
  {
    RCLCPP_WARN(logger, "Loader feedback is NaN.");
    return controller_interface::return_type::ERROR;
  }

  mode_ = cmd->mode;

  // TODO: 应随等级变化，由于今年只有哨兵是无下位机，暂时先这么写，没时间了
  if (cooling_heat_ >= 390)
  {
    mode_ = 0;
  }

  switch (mode_)
  {
    case 0:
    {
      bullet_vel_cmd_ = cmd->bullet_velocity;
      shooting_freq_cmd_ = 0;
      break;
    }
    case 1:
    {
      bullet_vel_cmd_ = cmd->bullet_velocity;
      shooting_freq_cmd_ = cmd->shooting_freq;
      break;
    }
    default:
      break;
  }

  double loader_vel = shooting_freq_cmd_ * shooting_freq2vel;
  double friction_wheel_vel = bullet_vel_cmd_ * bullet_vel2vel;
  if (loader_reverse_)
  {
    loader_vel = -loader_vel;
  }
  if (open_loop_)
  {
    for (size_t i = 0; i < friction_wheel_size_; i++)
    {
      auto velue = friction_wheel_vel;
      if (friction_wheel_reverse_[i])
      {
        velue = -velue;
      }
      friction_cmd_interface_[i]->set_value(velue);
    }
    loader_cmd_interface_->set_value(loader_vel);
  }
  else
  {
    double loader_error = loader_vel - loader_fb_;
    double loader_cmd = loader_pid_->computeCommand(loader_error, period);
    loader_cmd_interface_->set_value(loader_cmd);
    for (size_t i = 0; i < friction_wheel_size_; i++)
    {
      double friction_wheel_error = 0.0;
      if (friction_wheel_reverse_[i])
      {
        friction_wheel_error = -friction_wheel_vel - friction_wheel_fb_[i];
      }
      else
      {
        friction_wheel_error = friction_wheel_vel - friction_wheel_fb_[i];
      }
      double friction_wheel_cmd =
          friction_wheel_pid_[i]->computeCommand(friction_wheel_error, period);
      friction_cmd_interface_[i]->set_value(friction_wheel_cmd);
    }
  }
  double temp_bullet_vel = 0.0;
  for (size_t i = 0; i < friction_wheel_size_; i++)
  {
    temp_bullet_vel += abs(friction_wheel_fb_[i]);
  }
  bullet_vel_real_ = (temp_bullet_vel / friction_wheel_size_) * vel2bullet_vel;

  shooting_freq_real_ = loader_fb_ * vel2shooting_freq;

  if (rt_shooter_state_pub_->trylock())
  {
    auto& shooter_state = rt_shooter_state_pub_->msg_;
    shooter_state.header.stamp = time;
    shooter_state.mode = mode_;
    shooter_state.bullet_velocity = bullet_vel_real_;
    shooter_state.shooting_freq = shooting_freq_real_;

    rt_shooter_state_pub_->unlockAndPublish();
  }

  return controller_interface::return_type::OK;
}

controller_interface::CallbackReturn
TideShooterController::on_activate(const rclcpp_lifecycle::State& /*previous_state*/)
{
  for (size_t i = 0; i < friction_wheel_size_; i++)
  {
    friction_state_interface_[i] = std::make_unique<const hardware_interface::LoanedStateInterface>(
        std::move(state_interfaces_[i]));

    friction_cmd_interface_[i] = std::make_unique<hardware_interface::LoanedCommandInterface>(
        std::move(command_interfaces_[i]));
  }

  loader_state_interface_ = std::make_unique<hardware_interface::LoanedStateInterface>(
      std::move(state_interfaces_[friction_wheel_size_]));
  loader_cmd_interface_ = std::make_unique<hardware_interface::LoanedCommandInterface>(
      std::move(command_interfaces_[friction_wheel_size_]));

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn
TideShooterController::on_deactivate(const rclcpp_lifecycle::State& /*previous_state*/)
{
  for (size_t i = 0; i < friction_wheel_size_; i++)
  {
    friction_state_interface_[i].reset();
    friction_cmd_interface_[i].reset();
  }
  loader_state_interface_.reset();
  loader_cmd_interface_.reset();

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn
TideShooterController::on_cleanup(const rclcpp_lifecycle::State& previous_state)
{
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn
TideShooterController::on_error(const rclcpp_lifecycle::State& previous_state)
{
  return controller_interface::CallbackReturn::SUCCESS;
}

}  // namespace tide_shooter_controller

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(tide_shooter_controller::TideShooterController,
                       controller_interface::ControllerInterface)
