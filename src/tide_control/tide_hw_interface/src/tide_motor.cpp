/*
 * @Author: qinghuan 1484237245@qq.com
 * @Date: 2024-12-24 08:39:07
 * @FilePath: /tide_controls_full/src/tide_control/tide_hw_interface/src/tide_motor.cpp
 * @Description:
 *
 * Copyright (c) 2024 by qinghuan, All Rights Reserved.
 */
#include "tide_motor.hpp"
#include <cmath>

namespace tide_hw_interface
{
DJI_Motor::DJI_Motor(const Motor_Config_t& config)
{
  config_ = config;
  last_time_ = rclcpp::Clock().now();
  last_comm_time_ = last_time_;
}

void DJI_Motor::decode_feedback()
{
  measure.last_ecd = measure.ecd;
  measure.ecd = rx_buff[0] << 8 | rx_buff[1];
  measure.speed_aps =
      (1.0f - SPEED_SMOOTH_COEF) * measure.speed_aps +
      act2vel * SPEED_SMOOTH_COEF * (double)((int16_t)(rx_buff[2] << 8 | rx_buff[3]));
  measure.real_current = (1.0f - CURRENT_SMOOTH_COEF) * measure.real_current +
                         CURRENT_SMOOTH_COEF * (double)((int16_t)(rx_buff[4] << 8 | rx_buff[5]));
  measure.temperature = rx_buff[6];

  if (measure.ecd - measure.last_ecd > 4096)
    measure.total_round--;
  else if (measure.ecd - measure.last_ecd < -4096)
    measure.total_round++;

  measure.total_angle = (measure.total_round * 8191 + measure.ecd - config_.offset) * act2pos;

  double normalized_angle = measure.total_angle;

  while (normalized_angle > M_PI)
  {
    normalized_angle -= M_PI * 2;
  }
  while (normalized_angle <= -M_PI)
  {
    normalized_angle += M_PI * 2;
  }

  angle_current = normalized_angle;
}

bool DJI_Motor::check_connection(const rclcpp::Time& current_time)
{
  if (config_.motor_type == VIRTUAL_JOINT)
  {
    status = MOTOR_OK;
    return true;
  }

  double current_seconds = current_time.seconds();
  double last_comm_seconds = last_comm_time_.seconds();
  double time_diff = current_seconds - last_comm_seconds;

  if (time_diff > MOTOR_WATCHDOG_TIMEOUT)
  {
    status = MOTOR_LOST;
    return false;
  }

  return (status == MOTOR_OK);
}

}  // namespace tide_hw_interface
