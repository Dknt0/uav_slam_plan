/**
 * 飞控类
 *
 * Dknt 2023.12
 */

#ifndef OFFBOARD_AUTOPILOT_H
#define OFFBOARD_AUTOPILOT_H

#include <mavsdk/geometry.h>
#include <mavsdk/log_callback.h>
#include <mavsdk/mavsdk.h>
#include <mavsdk/plugins/action/action.h>
#include <mavsdk/plugins/offboard/offboard.h>
#include <mavsdk/plugins/telemetry/telemetry.h>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <chrono>
#include <future>
#include <iostream>
#include <mutex>
#include <thread>

class Autopilot {
 public:
  // Autopilot() {}
  Autopilot(const std::string port = "udp://:14550");

  bool Arm();
  bool Takeoff(const float altitude);
  bool Land();
  bool SetVelocity(const float vel);
  bool GotoPosition(float t_front, float t_left, float t_up, float t_yaw);

  bool Shutdown();

  // 位置外部控制不可用
  bool StartOffboardPosition();
  bool ExitOffboardPosition();
  bool SetPointOffboardPosition(float t_north, float t_east, float t_down,
                                float t_yaw);

  bool StartOffboardVelocity();
  bool ExitOffboardVelocity();
  bool SetPointOffboardVelocity(float v_down = 0., float w_yaw = 0.,
                                float v_forward = 0., float v_right = 0.);

  std::thread test_thread_;
  void CoordinateTest();

 protected:
  inline bool ShouldQuit() {
    std::unique_lock<std::mutex> lock(quit_flag_lock_);
    return quit_flag_;
  }

  bool quit_flag_ = false;
  std::mutex quit_flag_lock_;

  // MAVSDK 工具
  std::string port_;
  std::shared_ptr<mavsdk::Mavsdk> mavsdk_;
  std::shared_ptr<mavsdk::System> system_;
  std::shared_ptr<mavsdk::Telemetry> telemetry_;
  std::shared_ptr<mavsdk::Action> action_;
  std::shared_ptr<mavsdk::Offboard> offboard_;
  std::shared_ptr<mavsdk::geometry::CoordinateTransformation> transformer_;

  // 初始状态
  // Eigen::Isometry3f T_ned_body_origin;
  mavsdk::Telemetry::Position initial_global_pos_;
  mavsdk::Telemetry::EulerAngle initlal_orientation_;
  Eigen::Quaterniond q_Wned_Wfrd_;
  Eigen::Quaterniond q_Wned_Wflu_;
  // 状态变量
  mavsdk::Telemetry::PositionVelocityNed position_velocity_ned_;
  mavsdk::Telemetry::Odometry odometry_;
  mavsdk::Telemetry::Imu imu_;
  std::mutex data_lock_;
  // mavsdk::Telemetry::VelocityNed velocity_ned_;

  // 速度外部控制
  // TODO 标志位暂时未用
  bool offboard_velocity_flag = false;
  std::thread offboard_velocity_thread_;
  mavsdk::Offboard::VelocityBodyYawspeed velocity_setpoint_{0, 0, 0, 0};
  std::mutex velocity_setpoint_lock_;
  std::promise<void> offboard_velocity_promise_;
  std::future<void> offboard_velocity_future_;

  // 位置外部控制
  // MAVSDK 暂时不支持位置外部控制
  bool offboard_position_flag = false;
  std::thread offboard_position_thread_;
  mavsdk::Offboard::PositionNedYaw position_setpoint_{0, 0, 0, 0};
  std::mutex position_setpoint_lock_;
  std::promise<void> offboard_position_promise_;
  std::future<void> offboard_position_future_;

  double telemetry_update_rate_ = 50.;
};

#endif  // OFFBOARD_AUTOPILOT_H
