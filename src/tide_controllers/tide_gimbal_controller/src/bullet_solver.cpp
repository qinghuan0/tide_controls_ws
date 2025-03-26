/*
 * @Author: qinghuan 1484237245@qq.com
 * @Date: 2025-01-13 22:45:20
 * @FilePath: /TideControls/src/tide_controllers/tide_gimbal_controller/src/bullet_solver.cpp
 * @Description:
 *
 * Copyright (c) 2025 by qinghuan, All Rights Reserved.
 */
// 参考自华南师大的弹道解算：https://github.com/CodeAlanqian/SolveTrajectory
#include "bullet_solver.hpp"

namespace tide_gimbal_controller
{

BulletSolver::BulletSolver(double resistance_coff, double g, std::string frame, double time_delay,
                           std::vector<double> cam_offset)
{
  resistance_coff_ = resistance_coff;
  g_ = g;
  time_delay_ = time_delay;
  gimbal_frame_ = frame;
  if (cam_offset.size() != 3)
  {
    std::cerr << "cam_offset size must be 3" << std::endl;
    return;
  }
  cam_offset_ = cam_offset;
  trajectory_marker.header.frame_id = gimbal_frame_;
  trajectory_marker.ns = "trajectory";
  trajectory_marker.type = visualization_msgs::msg::Marker::POINTS;
  trajectory_marker.action = visualization_msgs::msg::Marker::ADD;
  trajectory_marker.scale.x = 0.1;
  trajectory_marker.scale.y = 0.1;
  trajectory_marker.color.a = 1.0;
  trajectory_marker.color.r = 1.0;
  trajectory_marker.color.g = 0.0;
  trajectory_marker.color.b = 1.0;
  trajectory_marker.lifetime = rclcpp::Duration::from_seconds(0.1);

  aiming_point_marker.header.frame_id = frame;
  aiming_point_marker.ns = "aiming_point";
  aiming_point_marker.type = visualization_msgs::msg::Marker::SPHERE;
  aiming_point_marker.action = visualization_msgs::msg::Marker::ADD;
  aiming_point_marker.scale.x = aiming_point_marker.scale.y = aiming_point_marker.scale.z = 0.12;
  aiming_point_marker.color.r = 1.0;
  aiming_point_marker.color.g = 1.0;
  aiming_point_marker.color.b = 1.0;
  aiming_point_marker.color.a = 1.0;
  aiming_point_marker.lifetime = rclcpp::Duration::from_seconds(0.1);

  auto target = std::make_shared<tide_msgs::msg::Target>();
  tracker_target_.initRT(target);

  auto transform = *std::make_shared<geometry_msgs::msg::TransformStamped>();
  tf_gimbal2target_.initRT(transform);
}

double BulletSolver::calculateRho(const geometry_msgs::msg::Point& point) const
{
  return std::sqrt(std::pow(point.x, 2) + std::pow(point.y, 2));
}

double BulletSolver::calculateFlyTime(double target_rho, double pitch_angle) const
{
  return (-std::log(1 - target_rho * resistance_coff_ / (bullet_speed_ * std::cos(pitch_angle)))) /
         resistance_coff_;
}

double BulletSolver::calculateRealZ(double pitch_angle, double fly_time) const
{
  double z;
  // z = (bullet_speed_ * std::sin(pitch_angle) + (g_ / resistance_coff_)) *
  //         (1 - std::exp(-resistance_coff_ * fly_time)) / resistance_coff_ -
  //     g_ * fly_time / resistance_coff_;

  z = bullet_speed_ * std::sin(pitch_angle) * fly_time - g_ * std::pow(fly_time, 2) / 2;
  return z;
}

void BulletSolver::selectMinYawArmor(geometry_msgs::msg::Point& pos)
{
  bool use_1 = true;
  yaw_ += v_yaw_ * time_delay_;

  double yaw_diff_min = 999.9;
  // 四块装甲板id逆时针排序
  for (size_t i = 0; i < 4; i++)
  {
    // 不打背后的装甲板
    if (i != 2)
    {
      double tmp_yaw = yaw_ + i * M_PI / 2.0;
      double tmp_yaw_diff = abs(cur_yaw_ - tmp_yaw);
      double r = use_1 ? radius_1_ : radius_2_;
      all_armors_pos_[i].x = pos.x - cam_offset_[0] + r * std::cos(tmp_yaw);
      all_armors_pos_[i].y = pos.y - cam_offset_[1] + r * std::sin(tmp_yaw);
      all_armors_pos_[i].z = (use_1 ? pos.z : pos.z + dz_) - cam_offset_[2];

      if (tmp_yaw_diff < yaw_diff_min)
      {
        tar_pos_ = all_armors_pos_[i];
        yaw_diff_min = tmp_yaw_diff;
      }
    }

    use_1 = !use_1;
  }
}

void BulletSolver::selectMinDisArmor(geometry_msgs::msg::Point& pos)
{
  bool use_1 = true;
  yaw_ += v_yaw_ * time_delay_;
  double min_dis = 999.9;

  for (size_t i = 0; i < 4; i++)
  {
    double tmp_yaw = yaw_ + i * M_PI / 2.0;
    double r = use_1 ? radius_1_ : radius_2_;
    all_armors_pos_[i].x = pos.x - cam_offset_[0] + r * std::cos(tmp_yaw);
    all_armors_pos_[i].y = pos.y - cam_offset_[1] + r * std::sin(tmp_yaw);
    all_armors_pos_[i].z = (use_1 ? pos.z : pos.z + dz_) - cam_offset_[2];

    double dis = std::sqrt(std::pow(all_armors_pos_[i].x, 2) + std::pow(all_armors_pos_[i].y, 2) +
                           std::pow(all_armors_pos_[i].z, 2));
    if (dis < min_dis)
    {
      min_dis = dis;
      tar_pos_ = all_armors_pos_[i];
    }
    use_1 = !use_1;
  }
}

bool BulletSolver::solve(double bullet_speed, double cur_pitch, double cur_yaw)
{
  bullet_speed_ = bullet_speed;
  cur_pitch_ = cur_pitch;
  cur_yaw_ = cur_yaw;
  selectMinDisArmor(pos_);
  double temp_z = tar_pos_.z;
  double target_rho = calculateRho(tar_pos_);
  double output_yaw = std::atan2(tar_pos_.y, tar_pos_.x);
  double output_pitch = std::atan2(temp_z, target_rho);

  double error_z = 999;

  for (size_t i = 0; i < 20; i++)
  {
    target_rho = calculateRho(tar_pos_);
    output_pitch = std::atan2(temp_z, target_rho);

    fly_time_ = calculateFlyTime(target_rho, output_pitch);
    double real_z = calculateRealZ(output_pitch, fly_time_);

    error_z = tar_pos_.z - real_z;
    temp_z += error_z;

    if (std::abs(error_z) < 0.001)
    {
      break;
    }
  }

  output_pitch_ = output_pitch;
  output_yaw_ = output_yaw;

  if (std::isnan(output_pitch_) || std::isnan(output_yaw_) || std::isinf(output_pitch_) ||
      std::isinf(output_yaw_))
  {
    return false;
  }

  return true;
}

void BulletSolver::ballistic_visualization()
{
  trajectory_marker.points.clear();
  geometry_msgs::msg::Point point{};
  geometry_msgs::msg::Point real_tar_pos{};
  real_tar_pos.x = tar_pos_.x - cam_offset_[0];
  real_tar_pos.y = tar_pos_.y - cam_offset_[1];
  real_tar_pos.z = tar_pos_.z - cam_offset_[2];
  double target_rho = std::sqrt(std::pow(real_tar_pos.x, 2) + std::pow(real_tar_pos.y, 2));
  int point_num = int(target_rho * 20);

  for (int i = 0; i <= point_num; i++)
  {
    double rt_bullet_rho = target_rho * i / point_num;
    double fly_time = calculateFlyTime(rt_bullet_rho, output_pitch_);
    double rt_bullet_z = calculateRealZ(output_pitch_, fly_time);
    point.x = rt_bullet_rho * std::cos(output_yaw_);
    point.y = rt_bullet_rho * std::sin(output_yaw_);
    point.z = rt_bullet_z;
    trajectory_marker.points.push_back(point);
  }

  aiming_point_marker.header.stamp = rclcpp::Time();
  aiming_point_marker.pose.position.x = point.x;
  aiming_point_marker.pose.position.y = point.y;
  aiming_point_marker.pose.position.z = point.z;

  if (rt_bullet_traj_pub_->trylock())
  {
    rt_bullet_traj_pub_->msg_ = trajectory_marker;
    rt_bullet_traj_pub_->unlockAndPublish();
  }

  if (rt_aiming_point_pub_->trylock())
  {
    rt_aiming_point_pub_->msg_ = aiming_point_marker;
    rt_aiming_point_pub_->unlockAndPublish();
  }
}

void BulletSolver::update()
{
  auto msg = *tracker_target_.readFromRT();
  tracking_ = msg->tracking;
  pos_ = msg->position;
  vel_ = msg->velocity;
  yaw_ = msg->yaw;
  v_yaw_ = msg->v_yaw;
  radius_1_ = msg->radius_1;
  radius_2_ = msg->radius_2;
  dz_ = msg->dz;
}

}  // namespace tide_gimbal_controller
