/*
 * @Author: naigia 2423723345@qq.com
 * @Date: 2025-02-20 08:21:42
 * @FilePath: /TideControls/src/tide_control/tide_referee/include/tide_referee.hpp
 * @Description:
 *
 * Copyright (c) 2025 by naigia, All Rights Reserved.
 */
#ifndef TIDE_REFEREE_HPP_
#define TIDE_REFEREE_HPP_

#define READER_BUFFER_SIZE 255  // do not change this
#define MAX_BUFFER_SIZE 2048
#define DECODE_BUFFER_SIZE 128
#define TRANSMIT_BUFFER 128

// ROS
#include <rclcpp/logging.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp/qos.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/subscription.hpp>
#include <rclcpp/utilities.hpp>

#include "tide_msgs/msg/referee_data.hpp"

// C++ system
#include <chrono>
#include <cstring>
#include <deque>
#include <functional>
#include <future>
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "tide_serial_driver/crc.hpp"
#include "tide_serial_driver/protocol.hpp"
#include "tide_serial_driver/serial_driver.hpp"

namespace tide_serial_driver
{
namespace tide_referee
{
class TideReferee : public rclcpp::Node
{
public:
  explicit TideReferee(const rclcpp::NodeOptions& options);

private:
  void rx();
  int receive();
  void Determine_ID();
  PkgState decode();
  void classify(uint8_t* rxbuff);

  std::shared_ptr<SerialConfig> config;
  std::shared_ptr<Port> port;

  std::deque<uint8_t> receive_buffer;

  rclcpp::Publisher<tide_msgs::msg::RefereeData>::SharedPtr referee_pub;
  rclcpp::Publisher<tide_msgs::msg::BulletRemaining>::SharedPtr bullet_remaining_pub;
  rclcpp::Publisher<tide_msgs::msg::GameRobotHP>::SharedPtr game_robothp_pub;
  rclcpp::Publisher<tide_msgs::msg::GameRobotStatus>::SharedPtr game_robotstatus_pub;
  rclcpp::Publisher<tide_msgs::msg::GameStatus>::SharedPtr game_status_pub;
  rclcpp::Publisher<tide_msgs::msg::PowerHeatData>::SharedPtr power_heat_data_pub;
  rclcpp::Publisher<tide_msgs::msg::RefereeWarning>::SharedPtr referee_warning_pub;
  rclcpp::Publisher<tide_msgs::msg::RobotHurt>::SharedPtr robot_hurt_pub;

  tide_msgs::msg::RefereeData refereedata;

  uint8_t decodeBuffer[DECODE_BUFFER_SIZE];
  uint8_t receiveBuffer[READER_BUFFER_SIZE];
  uint8_t uart_flag = 0;
  int read_sum = 0;
  int error_sum_payload = 0;
  int error_sum_header = 0;
  int pkg_sum = 0;

  bool crc_ok_header = false;
  bool crc_ok = false;

  uint16_t frame_length;
  std::thread rx_thread;
  judge_t judge_Info;
  PkgState pkgState;
  std::chrono::high_resolution_clock::time_point start = std::chrono::high_resolution_clock::now();
};
}  // namespace tide_referee
}  // namespace tide_serial_driver

#endif  // TIDE_REFEREE_HPP_
