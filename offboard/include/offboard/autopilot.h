/**
 * 飞控类
 *
 * Dknt 2023.12
 */

#ifndef OFFBOARD_AUTOPILOT_H
#define OFFBOARD_AUTOPILOT_H

#include <chrono>
#include <iostream>
#include <mutex>
#include <thread>
#include <future>

#include <mavsdk/log_callback.h>
#include <mavsdk/mavsdk.h>
#include <mavsdk/plugins/action/action.h>
#include <mavsdk/plugins/offboard/offboard.h>
#include <mavsdk/plugins/telemetry/telemetry.h>

#include <Eigen/Core>
#include <Eigen/Geometry>



class Autopilot {
 public:
  // Autopilot() {}
  Autopilot(const std::string port = "udp://:14550");

  bool Arm();
  bool Takeoff(const float altitude);
  bool Land();

  bool StartOffboardPosition();
  bool ExitOffboardPosition();
  bool SetPointPosition();

  bool StartOffboardVelocity();
  bool FinishOffboardVelocity();
  bool SetPointOffboardVelocity(float v_down = 0., float w_yaw = 0., float v_forward = 0., float v_right = 0.);

  std::thread test_thread_;
  void Test() {
    auto ThreadFunction = [&]() {
      while (true) {
        {
          std::unique_lock<std::mutex> lock(odometry_lock_);
          std::cout << "Odom position body: " << odometry_.position_body.x_m
                    << " " << odometry_.position_body.y_m << " "
                    << odometry_.position_body.z_m << std::endl;

          Eigen::Quaternionf q(odometry_.q.w, odometry_.q.x, odometry_.q.y,
                               odometry_.q.z);
          auto eular_angle = q.toRotationMatrix().eulerAngles(2, 1, 0);
          std::cout << "Odom orientation: yaw " << eular_angle[0] << " pitch "
                    << eular_angle[1] << " roll " << eular_angle[2] << std::endl;
        }

        std::this_thread::sleep_for(std::chrono::microseconds(500));
      }
    };
    test_thread_ = std::thread(ThreadFunction);

  }

 protected:
  std::string port_;
  std::shared_ptr<mavsdk::Mavsdk> mavsdk_;
  std::shared_ptr<mavsdk::System> system_;
  std::shared_ptr<mavsdk::Telemetry> telemetry_;
  std::shared_ptr<mavsdk::Action> action_;
  std::shared_ptr<mavsdk::Offboard> offboard_;

  mavsdk::Telemetry::Position position_ned_;
  std::mutex position_lock_;
  mavsdk::Telemetry::VelocityNed velocity_ned_;
  std::mutex velocity_ned_lock_;
  mavsdk::Telemetry::Odometry odometry_;
  std::mutex odometry_lock_;
  mavsdk::Telemetry::Imu imu_;
  std::mutex imu_lock_;

  std::thread offboard_velocity_thread_;
  mavsdk::Offboard::VelocityBodyYawspeed velocity_setpoint_{0, 0, 0, 0};
  std::mutex velocity_setpoint_lock_;
  std::promise<void> offboard_velocity_promise_;
  std::future<void> offboard_velocity_future_;

  double telemetry_update_rate_ = 100.;


};

#endif  // OFFBOARD_AUTOPILOT_H
