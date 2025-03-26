/*
 * @Author: qinghuan 1484237245@qq.com
 * @Date: 2024-12-24 08:36:23
 * @FilePath: /tide_controls_full/src/tide_control/tide_hw_interface/include/tide_motor.hpp
 * @Description:
 *
 * Copyright (c) 2024 by qinghuan, All Rights Reserved.
 */
#ifndef TIDE_MOTOR_HPP_
#define TIDE_MOTOR_HPP_

#include <rclcpp/rclcpp.hpp>
#include <array>
#include <string>
#include <cmath>

namespace tide_hw_interface
{
constexpr double SPEED_SMOOTH_COEF = 0.85f;
constexpr double CURRENT_SMOOTH_COEF = 0.9f;
constexpr double act2pos = 0.0007670840;        // 2PI/8192
constexpr double act2vel = 0.1047197551;        // 2PI/60
constexpr double MOTOR_WATCHDOG_TIMEOUT = 1.0;  // 电机通信超时时间，单位秒

typedef enum
{
  MOTOR_TYPE_NONE = 0,
  GM6020,
  M3508,
  M2006,
  VIRTUAL_JOINT,
} Motor_Type_e;

typedef enum
{
  MOTOR_LOST = 0,
  MOTOR_OK,
} Motor_Status_e;

typedef enum
{
  OPEN_LOOP = 0,
  SPEED_LOOP,
  POSITION_LOOP,
} Motor_CloseMode_e;

typedef struct
{
  std::string motor_name;
  std::string can_bus;
  uint32_t tx_id;
  uint32_t rx_id;
  uint32_t identifier;
  uint16_t offset;
  Motor_Type_e motor_type;
} Motor_Config_t;

class DJI_Motor
{
public:
  struct Measure
  {
    uint16_t last_ecd{ 0 };
    uint16_t ecd{ 0 };
    double speed_aps{ 0.0 };
    int16_t real_current{ 0 };
    uint8_t temperature{ 0 };
    double total_angle{ 0.0 };
    int32_t total_round{ 0 };

    Measure() = default;
  };

  Motor_Status_e status = MOTOR_LOST;

  Motor_Config_t config_;
  Measure measure;
  std::array<uint8_t, 8> rx_buff = { 0 };
  int16_t output = 0;
  double angle_current = 0.0;

  explicit DJI_Motor(const Motor_Config_t& config);
  void decode_feedback();
  void stop() { output = 0; }

  bool check_connection(const rclcpp::Time& current_time);
  void update_timestamp(const rclcpp::Time& time) { last_comm_time_ = time; }

private:
  rclcpp::Time last_time_{ 0, 0, RCL_ROS_TIME };
  rclcpp::Time last_comm_time_{ 0, 0, RCL_ROS_TIME };
};

}  // namespace tide_hw_interface

#endif
