/*
 * @Author: qinghuan 1484237245@qq.com
 * @Date: 2025-01-13 22:45:26
 * @FilePath: /TideControls/src/tide_controllers/tide_gimbal_controller/include/bullet_solver.hpp
 * @Description:
 *
 * Copyright (c) 2025 by qinghuan, All Rights Reserved.
 */
#ifndef BULLET_SOLVER_HPP
#define BULLET_SOLVER_HPP

#include "tide_msgs/msg/target.hpp"
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include "realtime_tools/realtime_buffer.hpp"
#include "realtime_tools/realtime_publisher.hpp"
#include <memory>

namespace tide_gimbal_controller
{
class BulletSolver
{
public:
  explicit BulletSolver(double resistance_coff, double g, std::string frame, double time_delay,
                        std::vector<double> cam_offset);
  ~BulletSolver() = default;
  bool tracking_{ false };

  bool solve(double bullet_speed, double cur_pitch, double cur_yaw);
  void update();
  void ballistic_visualization();

  std::pair<double, double> get_result() const
  {
    return std::make_pair(output_pitch_, output_yaw_);
  }

  visualization_msgs::msg::Marker trajectory_marker;
  visualization_msgs::msg::Marker aiming_point_marker;
  realtime_tools::RealtimeBuffer<tide_msgs::msg::Target::SharedPtr> tracker_target_;
  realtime_tools::RealtimeBuffer<geometry_msgs::msg::TransformStamped> tf_gimbal2target_;
  std::shared_ptr<realtime_tools::RealtimePublisher<visualization_msgs::msg::Marker>>
      rt_bullet_traj_pub_ = nullptr;
  std::shared_ptr<realtime_tools::RealtimePublisher<visualization_msgs::msg::Marker>>
      rt_aiming_point_pub_ = nullptr;

private:
  std::array<geometry_msgs::msg::Point, 4> all_armors_pos_;
  geometry_msgs::msg::Point tar_pos_{};
  geometry_msgs::msg::Point pos_{};
  geometry_msgs::msg::Vector3 vel_{};
  double yaw_{};
  double v_yaw_{};
  double radius_1_{};
  double radius_2_{};
  double dz_{};
  double cur_pitch_{ 0.0 };
  double cur_yaw_{ 0.0 };
  double time_delay_{ 0.0 };

  double bullet_speed_{ 25.0 };
  double resistance_coff_{ 0.001 };
  double fly_time_{ 0.0 };
  double g_{ 9.81 };
  std::vector<double> cam_offset_{};

  double output_pitch_{ 0.0 };
  double output_yaw_{ 0.0 };

  std::string gimbal_frame_{ "" };

  double calculateRho(const geometry_msgs::msg::Point& point) const;
  double calculateFlyTime(double target_rho, double pitch_angle) const;
  double calculateRealZ(double pitch_angle, double fly_time) const;
  void selectMinYawArmor(geometry_msgs::msg::Point& target_pos);
  void selectMinDisArmor(geometry_msgs::msg::Point& pos);
};

}  // namespace tide_gimbal_controller

#endif  // BULLET_SOLVER_HPP
