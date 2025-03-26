/*
 * @Author: naigia 2423723345@qq.com
 * @Date: 2025-02-20 08:21:42
 * @FilePath: /TideControls/src/tide_control/tide_referee/src/tide_referee.cpp
 * @Description:
 *
 * Copyright (c) 2025 by naigia, All Rights Reserved.
 */
#include "tide_referee.hpp"
#include "tide_msgs/msg/referee_data.hpp"

namespace tide_serial_driver
{
namespace tide_referee
{
TideReferee::TideReferee(const rclcpp::NodeOptions& options)
  : rclcpp ::Node("tide_referee", options)
  , config(std::make_shared<SerialConfig>(115200, 8, false, StopBit::ONE, Parity::NONE))
{
  // referee_pub = this->create_publisher<tide_msgs::msg::RefereeData>("/referee", 10);
  config->devname = "/dev/referee";
  port = std::make_shared<Port>(config);
  bullet_remaining_pub =
      this->create_publisher<tide_msgs::msg::BulletRemaining>("/bullet_remaining", 10);
  game_robothp_pub = this->create_publisher<tide_msgs::msg::GameRobotHP>("/robothp", 10);
  game_robotstatus_pub =
      this->create_publisher<tide_msgs::msg::GameRobotStatus>("/robot_status", 10);
  game_status_pub = this->create_publisher<tide_msgs::msg::GameStatus>("/game_status", 10);
  power_heat_data_pub =
      this->create_publisher<tide_msgs::msg::PowerHeatData>("/power_heat_data", 10);
  referee_warning_pub =
      this->create_publisher<tide_msgs::msg::RefereeWarning>("/referee_warning", 10);
  robot_hurt_pub = this->create_publisher<tide_msgs::msg::RobotHurt>("/robot_hurt", 10);

  while (true)
  {
    if (port->isPortOpen())
      break;
    else
      port->openPort();
    break;
  }

  rx_thread = std::thread(&TideReferee::rx, this);
}

void TideReferee::rx()
{
  while (true)
  {
    receive();
    pkgState = PkgState::COMPLETE;
    while (receive_buffer.size() > 0 && pkgState != PkgState::HEADER_INCOMPLETE &&
           pkgState != PkgState::PAYLOAD_INCOMPLETE)
    {
      pkgState = decode();
    }
  }
}

int TideReferee::receive()
{
  int read_num = 0;

  read_num = read(port->fd, receiveBuffer, 64);
  read_sum += read_num;

  if (read_num > 0)
  {
    receive_buffer.insert(receive_buffer.end(), receiveBuffer, receiveBuffer + read_num);
  }
  else
  {
    port->closePort();
    port->openPort();
  }
  return read_num;
}

void TideReferee::Determine_ID(void) {}

PkgState TideReferee::decode()
{
  int size = receive_buffer.size();

  for (int i = 0; i < size; i++)
  {
    if (receive_buffer[i] == 0xA5)
    {
      if (i + int(sizeof(Header_t)) > size)
      {
        return PkgState::HEADER_INCOMPLETE;
      }

      std::copy(receive_buffer.begin() + i, receive_buffer.begin() + i + sizeof(Header_t),
                decodeBuffer);
      crc_ok_header = crc::verifyCRC8CheckSum(decodeBuffer, LEN_FRAME_HEAD);

      if (!crc_ok_header)
      {
        RCLCPP_ERROR(get_logger(), "CRC error in header !");
        error_sum_header++;
        try
        {
          receive_buffer.erase(receive_buffer.begin() + i,
                               receive_buffer.begin() + i + sizeof(Header_t));
        }
        catch (const std::exception& ex)
        {
          RCLCPP_ERROR(get_logger(), "Error creating serial port:  %s", ex.what());
        }

        return PkgState::CRC_HEADER_ERRROR;
      }

      judge_Info.fream_header = arrayToStruct<Header_t>(decodeBuffer);

      if (i + int(judge_Info.fream_header.dataLen) + int(sizeof(Header_t)) + 2 > size)
      {
        return PkgState::PAYLOAD_INCOMPLETE;
      }
      std::copy(receive_buffer.begin() + i,
                receive_buffer.begin() + i + LEN_CMD_ID + judge_Info.fream_header.dataLen +
                    sizeof(Header_t) + LEN_FRAME_TAIL,
                decodeBuffer);
      crc_ok =
          crc::Verify_CRC16_Check_Sum(decodeBuffer, LEN_CMD_ID + judge_Info.fream_header.dataLen +
                                                        sizeof(Header_t) + LEN_FRAME_TAIL);

      if (!crc_ok)
      {
        error_sum_payload++;
        try
        {
          // check if there is a coming pkg
          for (int j = i + 1; j < int(judge_Info.fream_header.dataLen) + int(sizeof(Header_t)) +
                                      LEN_FRAME_TAIL + LEN_CMD_ID + i;
               j++)
          {
            if (receive_buffer[j] == 0xA5)
            {
              if (j + sizeof(Header_t) > judge_Info.fream_header.dataLen + sizeof(Header_t) +
                                             LEN_FRAME_TAIL + LEN_CMD_ID + i)
              {
                receive_buffer.erase(receive_buffer.begin(), receive_buffer.begin() + j);
                return PkgState::HEADER_INCOMPLETE;
              }

              std::copy(receive_buffer.begin() + i, receive_buffer.begin() + i + sizeof(Header_t),
                        decodeBuffer);
              crc_ok_header = crc::verifyCRC8CheckSum(decodeBuffer, sizeof(Header_t));

              if (!crc_ok_header)
              {
                receive_buffer.erase(receive_buffer.begin(),
                                     receive_buffer.begin() + j + sizeof(Header_t));
                j += sizeof(Header_t) - 1;
                continue;
              }

              judge_Info.fream_header = arrayToStruct<Header_t>(decodeBuffer);

              if (j + sizeof(Header_t) + judge_Info.fream_header.dataLen + LEN_FRAME_TAIL +
                  LEN_CMD_ID)
              {
                receive_buffer.erase(receive_buffer.begin(), receive_buffer.begin() + j);
                return PkgState::PAYLOAD_INCOMPLETE;
              }

              std::copy(receive_buffer.begin() + i,
                        receive_buffer.begin() + i + judge_Info.fream_header.dataLen +
                            sizeof(Header_t) + LEN_FRAME_TAIL + LEN_CMD_ID,
                        decodeBuffer);
              crc_ok = crc::Verify_CRC16_Check_Sum(decodeBuffer, judge_Info.fream_header.dataLen +
                                                                     sizeof(Header_t) +
                                                                     LEN_FRAME_TAIL + LEN_CMD_ID);

              if (!crc_ok)
              {
                RCLCPP_ERROR(get_logger(), "CRC error in payload !");
                receive_buffer.erase(receive_buffer.begin(), receive_buffer.begin() + j +
                                                                 sizeof(Header_t) +
                                                                 judge_Info.fream_header.dataLen +
                                                                 LEN_FRAME_TAIL + LEN_CMD_ID);
                j += sizeof(Header_t) + judge_Info.fream_header.dataLen + LEN_FRAME_TAIL +
                     LEN_CMD_ID - 1;
                continue;
              }
            }
          }
          receive_buffer.erase(receive_buffer.begin(),
                               receive_buffer.begin() + i + judge_Info.fream_header.dataLen +
                                   sizeof(Header_t) + LEN_FRAME_TAIL + LEN_CMD_ID);
        }
        catch (const std::exception& ex)
        {
          RCLCPP_ERROR(get_logger(), "Error creating serial port:  %s", ex.what());
        }

        return PkgState::CRC_PKG_ERROR;
      }
      // complete
      try
      {
        receive_buffer.erase(receive_buffer.begin(),
                             receive_buffer.begin() + i + judge_Info.fream_header.dataLen +
                                 sizeof(Header_t) + LEN_FRAME_TAIL + LEN_CMD_ID);
      }
      catch (const std::exception& ex)
      {
        RCLCPP_ERROR(get_logger(), "Error creating serial port:  %s", ex.what());
      }

      pkg_sum++;

      std::chrono::high_resolution_clock::time_point end =
          std::chrono::high_resolution_clock::now();
      std::chrono::duration<double> time = end - start;

      classify(decodeBuffer);
      return PkgState::COMPLETE;
      std::cout << "/* COMPLETE,COMPLETE,COMPLETE,COMPLETE */" << std::endl;
    }
  }
  receive_buffer.erase(receive_buffer.begin(), receive_buffer.end());
  return PkgState::OTHER;
}

void TideReferee::classify(uint8_t* rxbuff)
{
  uint32_t cmd_id = rxbuff[5] | (rxbuff[6] << 8);  //读取命令ID

  switch (cmd_id)
  {
    case ID_game_state:
      memcpy(&judge_Info.game_status, rxbuff + 7, judge_Info.fream_header.dataLen);
      refereedata.gamestatus.game_type = judge_Info.game_status.game_type;
      refereedata.gamestatus.game_progress = judge_Info.game_status.game_progress;
      refereedata.gamestatus.stage_remain_time = judge_Info.game_status.stage_remain_time;
      refereedata.gamestatus.sync_timestamp = judge_Info.game_status.SyncTimeStamp;
      break;
    case ID_game_result:
      memcpy(&judge_Info.game_result, rxbuff + 7, judge_Info.fream_header.dataLen);
      refereedata.gameresult.winner = judge_Info.game_result.winner;
      break;
    case ID_game_robot_HP:
      memcpy(&judge_Info.game_robot_HP, rxbuff + 7, judge_Info.fream_header.dataLen);
      memcpy(&refereedata.gamerobothp, &judge_Info.game_robot_HP, judge_Info.fream_header.dataLen);
      break;
    case ID_event_data:
      memcpy(&judge_Info.event_data, rxbuff + 7, judge_Info.fream_header.dataLen);
      refereedata.eventdata.event_data = judge_Info.event_data.event_data;
      break;
    case ID_referee_warning:
      memcpy(&judge_Info.referee_warning, rxbuff + 7, judge_Info.fream_header.dataLen);
      memcpy(&refereedata.refereewarning, &judge_Info.referee_warning,
             judge_Info.fream_header.dataLen);
      break;
    case ID_floor_robot_pos:
      memcpy(&judge_Info.ground_robot_pos, rxbuff + 7, judge_Info.fream_header.dataLen);
      memcpy(&refereedata.groundrobotpos, &judge_Info.ground_robot_pos,
             judge_Info.fream_header.dataLen);
      break;
    case ID_game_robot_status:
      memcpy(&judge_Info.game_robot_status, rxbuff + 7, judge_Info.fream_header.dataLen);
      refereedata.gamerobotstatus.robot_id = judge_Info.game_robot_status.robot_id;
      refereedata.gamerobotstatus.robot_level = judge_Info.game_robot_status.robot_level;
      refereedata.gamerobotstatus.current_hp = judge_Info.game_robot_status.current_HP;
      refereedata.gamerobotstatus.maximum_hp = judge_Info.game_robot_status.maximum_HP;
      refereedata.gamerobotstatus.shooter_barrel_cooling_value =
          judge_Info.game_robot_status.shooter_barrel_cooling_value;
      refereedata.gamerobotstatus.shooter_barrel_heat_limit =
          judge_Info.game_robot_status.shooter_barrel_heat_limit;
      refereedata.gamerobotstatus.chassis_power_limit =
          judge_Info.game_robot_status.chassis_power_limit;
      refereedata.gamerobotstatus.power_management_gimbal_output =
          judge_Info.game_robot_status.power_management_gimbal_output;
      refereedata.gamerobotstatus.power_management_chassis_output =
          judge_Info.game_robot_status.power_management_chassis_output;
      refereedata.gamerobotstatus.power_management_shooter_output =
          judge_Info.game_robot_status.power_management_shooter_output;
      Determine_ID();
      break;
    case ID_power_heat_data:
      memcpy(&judge_Info.power_heat_data, rxbuff + 7, judge_Info.fream_header.dataLen);
      memcpy(&refereedata.powerheatdata, &judge_Info.power_heat_data,
             judge_Info.fream_header.dataLen);
      break;
    case ID_game_robot_pos:
      memcpy(&judge_Info.game_robot_pos, rxbuff + 7, judge_Info.fream_header.dataLen);
      memcpy(&refereedata.gamerobotpos, &judge_Info.game_robot_pos, judge_Info.fream_header.dataLen);
      break;
    case ID_buff_musk:
      memcpy(&judge_Info.buff, rxbuff + 7, judge_Info.fream_header.dataLen);
      memcpy(&refereedata.buff, &judge_Info.buff, judge_Info.fream_header.dataLen);
      break;
    case ID_robot_hurt:
      memcpy(&judge_Info.robot_hurt, rxbuff + 7, judge_Info.fream_header.dataLen);
      refereedata.robothurt.armor_id = judge_Info.robot_hurt.armor_id;
      refereedata.robothurt.hurt_type = judge_Info.robot_hurt.hurt_type;
      break;
    case ID_shoot_data:
      memcpy(&judge_Info.shoot_data, rxbuff + 7, judge_Info.fream_header.dataLen);
      memcpy(&refereedata.shootdata, &judge_Info.shoot_data, judge_Info.fream_header.dataLen);
      break;
    case ID_bullet_remaining:
      memcpy(&judge_Info.bullet_remaining, rxbuff + 7, judge_Info.fream_header.dataLen);
      memcpy(&refereedata.bulletremaining, &judge_Info.bullet_remaining,
             judge_Info.fream_header.dataLen);
      break;
    case ID_rfid_status:
      memcpy(&judge_Info.rfid_status, rxbuff + 7, judge_Info.fream_header.dataLen);
      refereedata.rfidstatus.rfid_status = judge_Info.rfid_status.rfid_status;
      break;
    case ID_sentry_auto_data:
      memcpy(&judge_Info.sentry_info, rxbuff + 7, judge_Info.fream_header.dataLen);
      refereedata.sentryinfo.sentry_info = judge_Info.sentry_info.sentry_info;
      refereedata.sentryinfo.sentry_info_2 = judge_Info.sentry_info.sentry_info_2;
      break;

    default:
      break;
  }
  // if (cmd_id == 0x201)
  // {
  //   std::cout << "robot_id :" << int(refereedata.gamerobotstatus.robot_id) << std::endl;
  //   std::cout << "buffer_energy :" << int(judge_Info.power_heat_data.buffer_energy) << std::endl;
  // }
  // referee_pub->publish(refereedata);
  bullet_remaining_pub->publish(refereedata.bulletremaining);
  game_robothp_pub->publish(refereedata.gamerobothp);
  game_robotstatus_pub->publish(refereedata.gamerobotstatus);
  game_status_pub->publish(refereedata.gamestatus);
  power_heat_data_pub->publish(refereedata.powerheatdata);
  referee_warning_pub->publish(refereedata.refereewarning);
  robot_hurt_pub->publish(refereedata.robothurt);
}

}  // namespace tide_referee
}  // namespace tide_serial_driver
#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(tide_serial_driver::tide_referee::TideReferee)
