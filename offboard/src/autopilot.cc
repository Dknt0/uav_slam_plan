/**
 * 飞控类
 *
 * Dknt 2023.12
 */

#include "autopilot.h"

// clang-format off
// UBUNTU/LINUX terminal color codes
#define RESET       "\033[0m"
#define BLACK       "\033[30m"          /* Black */
#define RED         "\033[31m"          /* Red */
#define GREEN       "\033[32m"          /* Green */
#define YELLOW      "\033[33m"          /* Yellow */
#define BLUE        "\033[34m"          /* Blue */
#define MAGENTA     "\033[35m"          /* Magenta */
#define CYAN        "\033[36m"          /* Cyan */
#define WHITE       "\033[37m"          /* White */
#define BOLDBLACK   "\033[1m\033[30m"   /* Bold Black */
#define BOLDRED     "\033[1m\033[31m"   /* Bold Red */
#define BOLDGREEN   "\033[1m\033[32m"   /* Bold Green */
#define BOLDYELLOW  "\033[1m\033[33m"   /* Bold Yellow */
#define BOLDBLUE    "\033[1m\033[34m"   /* Bold Blue */
#define BOLDMAGENTA "\033[1m\033[35m"   /* Bold Magenta */
#define BOLDCYAN    "\033[1m\033[36m"   /* Bold Cyan */
#define BOLDWHITE   "\033[1m\033[37m"   /* Bold White */

#define AUTOPILOT_MESSAGE_HEAD BOLDCYAN  << "[Autopilot] " << RESET
#define AUTOPILOT_PROCESS_HEAD BOLDGREEN << "[Autopilot] " << RESET
#define AUTOPILOT_ERROR_HEAD   BOLDRED   << "[Autopilot] " << RESET
// clang-format on

/// @brief 角度转弧度，不归一化处理
/// @tparam T
/// @param deg
/// @return
template <typename T>
inline T Deg2Rad(const T deg) {
  return (deg * T(M_PI) / T(180));
}

/// @brief 弧度转角度，不归一化处理
/// @tparam T
/// @param rad
/// @return
template <typename T>
inline T Rad2Deg(const T rad) {
  return (rad * T(180) / T(M_PI));
}

/// @brief 角归一化 度
/// @tparam T
/// @param deg
/// @return
template <typename T>
inline T NormolizeAngelDeg(const T deg) {
  if (deg > T(360))
    return (deg - 360);
  else if (deg < T(0))
    return (deg + 360);
  else
    return deg;
}

/// @brief 角归一化 弧度
/// @tparam T
/// @param rad
/// @return
template <typename T>
inline T NormolizeAngelRad(const T rad) {
  T double_pi = T(M_PI * 2);
  if (rad > T(M_PI))
    return (rad - double_pi);
  else if (rad < -T(M_PI))
    return (rad + double_pi);
  else
    return rad;
}

using namespace std::chrono_literals;

/// @brief 构造函数
/// @param port
Autopilot::Autopilot(const std::string port) : port_(port) {
  // 日志等级
  mavsdk::log::subscribe([](mavsdk::log::Level level,
                            const std::string &message, const std::string &file,
                            int line) {
    (void)message;
    (void)file;
    (void)line;
    return (level != mavsdk::log::Level::Err);
  });

  mavsdk_ = std::make_shared<mavsdk::Mavsdk>();

  // 监听端口
  std::cout << AUTOPILOT_MESSAGE_HEAD << "Created object, UAV port set to "
            << port << std::endl;
  std::cout << AUTOPILOT_MESSAGE_HEAD << "Waiting for connection..."
            << std::endl;
  {
    auto error = mavsdk_->add_any_connection(port_);
    if (error != mavsdk::ConnectionResult::Success) {
      std::cout << AUTOPILOT_ERROR_HEAD
                << "Connection failed: " << static_cast<int>(error)
                << std::endl;
      exit(1);
    }
  }

  // 初始化插件
  while (mavsdk_->systems().size() == 0 && !ShouldQuit()) {
    std::this_thread::sleep_for(0.5s);
  }
  system_ = mavsdk_->systems()[0];
  telemetry_ = std::make_shared<mavsdk::Telemetry>(system_);
  action_ = std::make_shared<mavsdk::Action>(system_);
  offboard_ = std::make_shared<mavsdk::Offboard>(system_);
  std::cout << AUTOPILOT_MESSAGE_HEAD << "Connected." << std::endl;

  // position velocity callback NED
  {
    auto error =
        telemetry_->set_rate_position_velocity_ned(telemetry_update_rate_);
    if (error != mavsdk::Telemetry::Result::Success) {
      std::cout << AUTOPILOT_ERROR_HEAD
                << "Failed to setting telemetry position velocity rate: "
                << static_cast<int>(error) << std::endl;
    }
  }
  telemetry_->subscribe_position_velocity_ned(
      [this](mavsdk::Telemetry::PositionVelocityNed position_velocity_ned) {
        data_lock_.lock();
        this->position_velocity_ned_ = position_velocity_ned;
        data_lock_.unlock();
      });

  // // velocity callback NED
  // {
  //   auto error = telemetry_->set_rate_velocity_ned(telemetry_update_rate_);
  //   if (error != mavsdk::Telemetry::Result::Success) {
  //     std::cout << AUTOPILOT_ERROR_HEAD
  //               << "Failed to setting telemetry velocity rate: "
  //               << static_cast<int>(error) << std::endl;
  //   }
  // }
  // telemetry_->subscribe_velocity_ned(
  //     [this](mavsdk::Telemetry::VelocityNed velocity_ned) {
  //       velocity_ned_lock_.lock();
  //       this->velocity_ned_ = velocity_ned;
  //       velocity_ned_lock_.unlock();
  //     });

  // odometry callback
  {
    auto error = telemetry_->set_rate_odometry(telemetry_update_rate_);
    if (error != mavsdk::Telemetry::Result::Success) {
      std::cout << AUTOPILOT_ERROR_HEAD
                << "Failed to set telemetry odometry rate: "
                << static_cast<int>(error) << std::endl;
    }
  }
  telemetry_->subscribe_odometry([this](mavsdk::Telemetry::Odometry odometry) {
    data_lock_.lock();
    this->odometry_ = odometry;
    data_lock_.unlock();
  });

  // IMU callback
  {
    auto error = telemetry_->set_rate_imu(telemetry_update_rate_);
    if (error != mavsdk::Telemetry::Result::Success) {
      std::cout << AUTOPILOT_ERROR_HEAD << "Failed to set telemetry imu rate: "
                << static_cast<int>(error) << std::endl;
    }
  }
  telemetry_->subscribe_imu([this](mavsdk::Telemetry::Imu imu) {
    data_lock_.lock();
    this->imu_ = imu;
    data_lock_.unlock();
  });

  // // Set initial global location
  // mavsdk::Telemetry::PositionVelocityNed initial_position_ned_ =
  //     telemetry_->position_velocity_ned();
  // mavsdk::Telemetry::EulerAngle initial_attitude_ =
  //     telemetry_->attitude_euler();
  // Eigen::Vector3f init_position(initial_position_ned_.position.north_m,
  //                               initial_position_ned_.position.east_m,
  //                               initial_position_ned_.position.down_m);
  // float roll_rad = Deg2Rad(initial_attitude_.roll_deg);
  // float pitch_rad = Deg2Rad(initial_attitude_.pitch_deg);
  // float yaw_rad = Deg2Rad(initial_attitude_.yaw_deg);
  // Eigen::Quaternionf init_orientation =
  //     Eigen::AngleAxisf(roll_rad, Eigen::Vector3f::UnitX()) *
  //     Eigen::AngleAxisf(pitch_rad, Eigen::Vector3f::UnitY()) *
  //     Eigen::AngleAxisf(yaw_rad, Eigen::Vector3f::UnitZ());
  // T_ned_body_origin = Eigen::Isometry3f(init_orientation);
  // T_ned_body_origin.pretranslate(init_position);

  // 获取初始全局位姿
  initial_global_pos_ = telemetry_->position();
  initlal_orientation_ = telemetry_->attitude_euler();
  mavsdk::geometry::CoordinateTransformation::GlobalCoordinate
      global_coordinate;
  global_coordinate.longitude_deg = initial_global_pos_.longitude_deg;
  global_coordinate.latitude_deg = initial_global_pos_.latitude_deg;
  transformer_ = std::make_shared<mavsdk::geometry::CoordinateTransformation>(
      global_coordinate);

  // 姿态变换
  double yaw_rad = Deg2Rad(initlal_orientation_.yaw_deg);
  double pitch_rad = Deg2Rad(initlal_orientation_.pitch_deg);
  double roll_rad = Deg2Rad(initlal_orientation_.roll_deg);
  q_Wned_Wfrd_ = Eigen::Quaterniond(
      Eigen::AngleAxisd(roll_rad, Eigen::Vector3d::UnitX()) *
      Eigen::AngleAxisd(pitch_rad, Eigen::Vector3d::UnitY()) *
      Eigen::AngleAxisd(yaw_rad, Eigen::Vector3d::UnitZ()));
  Eigen::Quaterniond q_Wfrd_Wflu(
      Eigen::AngleAxisd(-M_PI, Eigen::Vector3d::UnitX()));
  q_Wned_Wflu_ = q_Wned_Wfrd_ * q_Wfrd_Wflu;

  std::cout << AUTOPILOT_MESSAGE_HEAD << "Initialization completed."
            << std::endl;
}

/// @brief 启动
/// @return
bool Autopilot::Arm() {
  std::cout << AUTOPILOT_MESSAGE_HEAD << "Waiting for RTF..." << std::endl;
  while (telemetry_->health_all_ok() && !ShouldQuit()) {
    std::this_thread::sleep_for(0.5s);
  }

  {
    auto error = action_->arm();
    if (error != mavsdk::Action::Result::Success) {
      std::cout << AUTOPILOT_ERROR_HEAD
                << "Failed to arm: " << static_cast<int>(error) << std::endl;
      return false;
    }
  }

  std::cout << AUTOPILOT_MESSAGE_HEAD << "Armed" << std::endl;
  return true;
}

/// @brief 起飞
/// @param altitude 起飞高度
/// @return
bool Autopilot::Takeoff(const float altitude) {
  {
    auto error = action_->set_takeoff_altitude(altitude);
    if (error != mavsdk::Action::Result::Success) {
      std::cout << AUTOPILOT_ERROR_HEAD
                << "Failed to set takeoff altitude: " << static_cast<int>(error)
                << std::endl;
      return false;
    }
  }

  {
    auto error = action_->takeoff();
    if (error != mavsdk::Action::Result::Success) {
      std::cout << AUTOPILOT_ERROR_HEAD
                << "Failed to send takeoff instruction: "
                << static_cast<int>(error) << std::endl;
      return false;
    }
  }

  std::cout.precision(2);
  while (!ShouldQuit()) {
    data_lock_.lock();
    auto cur_altitude = -this->position_velocity_ned_.position.down_m;
    data_lock_.unlock();
    if (cur_altitude > altitude - 0.25) {
      break;
    }
    std::cout << '\r' << AUTOPILOT_PROCESS_HEAD
              << "Taking off. Target alt: " << altitude
              << "m. Current alt: " << cur_altitude << "m         "
              << std::flush;
    std::this_thread::sleep_for(0.5s);
  }
  std::cout << std::endl;

  std::cout << AUTOPILOT_MESSAGE_HEAD << "Takenoff." << std::endl;

  return true;
}

/// @brief 降落
/// @return
bool Autopilot::Land() {
  {
    auto error = action_->land();
    if (error != mavsdk::Action::Result::Success) {
      std::cout << AUTOPILOT_ERROR_HEAD
                << "Failed to send land command: " << static_cast<int>(error)
                << std::endl;
      return false;
    }
  }

  // 等待降落完成
  std::cout.precision(2);
  while (telemetry_->in_air() && !ShouldQuit()) {
    data_lock_.lock();
    auto cur_altitude = this->odometry_.position_body.z_m;
    data_lock_.unlock();
    if (cur_altitude < 0.2) {
      break;
    }

    std::cout << '\r' << AUTOPILOT_PROCESS_HEAD
              << "Landing. Current alt: " << cur_altitude << "m           "
              << std::flush;
    std::this_thread::sleep_for(0.5s);
  }
  std::cout << std::endl;

  std::cout << AUTOPILOT_MESSAGE_HEAD << "Landed." << std::endl;
  return true;
}

/// @brief 设置速度
/// @param vel 
/// @return 
bool Autopilot::SetVelocity(const float vel) {
  {
    auto error = action_->set_maximum_speed(vel);
    if (error != mavsdk::Action::Result::Success) {
      std::cout << AUTOPILOT_ERROR_HEAD
                << "Failed to set velocity: " << static_cast<int>(error)
                << std::endl;
      return false;
    }
  }
  std::cout << AUTOPILOT_MESSAGE_HEAD << "Set velocity " << vel << " m/s." << std::endl;
  return true;
}

/// @brief 位置控制 WFLU
/// @param t_front 前
/// @param t_left 左
/// @param t_up 上
/// @param t_yaw 偏航角 rad
/// @return
bool Autopilot::GotoPosition(float t_front, float t_left, float t_up,
                             float t_yaw) {
  Eigen::Vector3d position_Wflu(t_front, t_left, t_up);
  Eigen::Vector3d position_Wned = q_Wned_Wflu_ * position_Wflu;

  auto global_coordinates =
      transformer_->global_from_local({position_Wned[0], position_Wned[1]});

  // TODO 朝向目标点

  {
    auto error = action_->goto_location(
        global_coordinates.latitude_deg, global_coordinates.longitude_deg,
        initial_global_pos_.absolute_altitude_m - position_Wned[2],
        NormolizeAngelDeg(initlal_orientation_.yaw_deg + Rad2Deg(t_yaw)));
    if (error != mavsdk::Action::Result::Success) {
      std::cout << AUTOPILOT_ERROR_HEAD
                << "Failed to call goto_location: " << static_cast<int>(error)
                << std::endl;
      return false;
    }
  }

  // TODO wait ?
  return true;
}

/// @brief 强制关闭飞控
/// @return
bool Autopilot::Shutdown() {
  std::unique_lock<std::mutex> lock(quit_flag_lock_);
  quit_flag_ = true;
  std::cout << AUTOPILOT_ERROR_HEAD << "Forced shutdown." << std::endl;
  return true;
}

/// @brief 开启外部速度控制模式
/// @return
bool Autopilot::StartOffboardVelocity() {
  // 发送第一次 setpoint
  {
    std::unique_lock<std::mutex> lock(velocity_setpoint_lock_);
    auto error = offboard_->set_velocity_body(velocity_setpoint_);
    if (error != mavsdk::Offboard::Result::Success) {
      std::cout << AUTOPILOT_ERROR_HEAD
                << "Failed to send initial velocity setpoint_: "
                << static_cast<int>(error) << std::endl;
    }
  }

  // 切换到 offboard mode
  {
    auto error = offboard_->start();
    if (error != mavsdk::Offboard::Result::Success) {
      std::cout << AUTOPILOT_ERROR_HEAD
                << "Failed to start offboard mode: " << static_cast<int>(error)
                << std::endl;
    }
  }

  offboard_velocity_promise_ = std::promise<void>();
  offboard_velocity_future_ = offboard_velocity_promise_.get_future();

  std::cout << AUTOPILOT_MESSAGE_HEAD << "Started offboard mode." << std::endl;
  auto offboard_function = [this]() {
    while (offboard_velocity_future_.wait_for(0.02s) ==
               std::future_status::timeout &&
           !ShouldQuit()) {
      std::unique_lock<std::mutex> lock(velocity_setpoint_lock_);
      auto error = offboard_->set_velocity_body(velocity_setpoint_);
      if (error != mavsdk::Offboard::Result::Success) {
        std::cout << AUTOPILOT_MESSAGE_HEAD
                  << "Failed to send velocity setpoint: "
                  << static_cast<int>(error) << std::endl;
      }
    }
  };

  offboard_velocity_thread_ = std::thread(offboard_function);
  return true;
}

/// @brief 结束速度外部控制模式
/// @return
bool Autopilot::ExitOffboardVelocity() {
  offboard_velocity_promise_.set_value();
  offboard_velocity_thread_.join();
  std::cout << AUTOPILOT_MESSAGE_HEAD << "Finished offboard mode." << std::endl;
  return true;
}

/// @brief 设置外部控制速度 FRD
/// @param v_down 下降速度
/// @param w_yaw 偏航速度
/// @param v_forward 前向速度
/// @param v_right 横向速度
/// @return
bool Autopilot::SetPointOffboardVelocity(float v_down, float w_yaw,
                                         float v_forward, float v_right) {
  std::unique_lock<std::mutex> lock(velocity_setpoint_lock_);
  velocity_setpoint_.forward_m_s = v_forward;
  velocity_setpoint_.right_m_s;
  velocity_setpoint_.down_m_s;
  velocity_setpoint_.yawspeed_deg_s;

  return true;
}

// /// @brief 开启位置外部控制模式
// /// @return
// bool Autopilot::StartOffboardPosition() {
//   // 设置 setpoint 为无人机当前位置
//   {
//     std::unique_lock<std::mutex> lock_2(data_lock_);
//     Eigen::Vector3f current_position_body(odometry_.position_body.x_m,
//                                           odometry_.position_body.y_m,
//                                           odometry_.position_body.z_m);
//     Eigen::Quaternionf current_orientation_body(odometry_.q.w, odometry_.q.x,
//                                                 odometry_.q.y,
//                                                 odometry_.q.z);
//     Eigen::Isometry3f current_pose_body(current_orientation_body);
//     current_pose_body.pretranslate(current_position_body);

//     Eigen::Isometry3f current_pose_ned = T_ned_body_origin *
//     current_pose_body; Eigen::Vector3f current_position_ned =
//     current_pose_ned.translation(); Eigen::Quaternionf
//     current_orientation_ned(current_pose_ned.rotation());

//     position_setpoint_.north_m = current_position_ned[0];
//     position_setpoint_.east_m = current_position_ned[1];
//     position_setpoint_.down_m = current_position_ned[2];
//     position_setpoint_.yaw_deg =
//         Rad2Deg(current_orientation_ned.matrix().eulerAngles(2, 1, 0)[0]);
//   }

//   // 发送第一次 setpoint
//   {
//     std::unique_lock<std::mutex> lock(position_setpoint_lock_);
//     auto error = offboard_->set_position_ned(position_setpoint_);

//     std::cout << position_setpoint_.north_m << " " <<
//     position_setpoint_.east_m
//               << " " << position_setpoint_.down_m << " "
//               << position_setpoint_.yaw_deg << std::endl;

//     if (error != mavsdk::Offboard::Result::Success) {
//       std::cout << AUTOPILOT_ERROR_HEAD
//                 << "Failed to send initial position setpoint_: "
//                 << static_cast<int>(error) << std::endl;
//       return false;
//     }
//   }

//   // 切换到 offboard mode
//   {
//     auto error = offboard_->start();
//     if (error != mavsdk::Offboard::Result::Success) {
//       std::cout << AUTOPILOT_ERROR_HEAD
//                 << "Failed to start offboard mode: " <<
//                 static_cast<int>(error)
//                 << std::endl;
//     }
//     return false;
//   }

//   offboard_position_promise_ = std::promise<void>();
//   offboard_position_future_ = offboard_position_promise_.get_future();

//   std::cout << AUTOPILOT_MESSAGE_HEAD << "Started offboard mode." <<
//   std::endl; auto offboard_function = [this]() {
//     while (offboard_position_future_.wait_for(0.02s) ==
//            std::future_status::timeout && !ShouldQuit()) {
//       std::unique_lock<std::mutex> lock(position_setpoint_lock_);

//       auto error = offboard_->set_position_ned(position_setpoint_);
//       if (error != mavsdk::Offboard::Result::Success) {
//         std::cout << AUTOPILOT_MESSAGE_HEAD
//                   << "Failed to send position setpoint: "
//                   << static_cast<int>(error) << std::endl;
//       }

//       std::cout << position_setpoint_.north_m << " "
//                 << position_setpoint_.east_m << " " <<
//                 position_setpoint_.down_m
//                 << " " << position_setpoint_.yaw_deg << std::endl;
//     }
//   };

//   offboard_position_thread_ = std::thread(offboard_function);
//   return true;
// }

// /// @brief 退出位置外部控制模式
// /// @return
// bool Autopilot::ExitOffboardPosition() {
//   offboard_position_promise_.set_value();
//   offboard_position_thread_.join();
//   std::cout << AUTOPILOT_MESSAGE_HEAD << "Finished offboard mode." <<
//   std::endl; return true;
// }

// /// @brief 设置外部控制位置 FRD
// /// @param t_x x
// /// @param t_y y
// /// @param t_z z
// /// @param t_yaw 偏航角 rad
// /// @return
// bool Autopilot::SetPointOffboardPosition(float t_x, float t_y, float t_z,
//                                          float t_yaw) {
//   std::unique_lock<std::mutex> lock(position_setpoint_lock_);
//   Eigen::Vector3f target_position_body(t_x, t_y, t_z);
//   Eigen::AngleAxisf target_orientation_body(t_yaw, Eigen::Vector3f::UnitZ());
//   Eigen::Isometry3f target_pose_body(target_orientation_body);
//   target_pose_body.pretranslate(target_position_body);

//   Eigen::Isometry3f target_pose_ned = T_ned_body_origin * target_pose_body;
//   Eigen::Vector3f target_position_ned = target_pose_ned.translation();
//   Eigen::Quaternionf target_orientation_ned(target_pose_ned.rotation());

//   position_setpoint_.north_m = target_position_ned[0];
//   position_setpoint_.east_m = target_position_ned[1];
//   position_setpoint_.down_m = target_position_ned[2];
//   position_setpoint_.yaw_deg =
//       Rad2Deg(target_orientation_ned.matrix().eulerAngles(2, 1, 0)[0]);

//   return true;
// }

/// @brief 位置控制测试
void Autopilot::CoordinateTest() {
  data_lock_.lock();

  mavsdk::geometry::CoordinateTransformation::LocalCoordinate local_coordinate;
  local_coordinate.east_m = position_velocity_ned_.position.east_m;
  local_coordinate.north_m = position_velocity_ned_.position.north_m;

  mavsdk::geometry::CoordinateTransformation::GlobalCoordinate
      global_coordinate = transformer_->global_from_local(local_coordinate);

  auto global_test = transformer_->global_from_local({0, 0});

  auto local_test = transformer_->local_from_global(
      {global_test.latitude_deg, global_test.longitude_deg});

  std::cout << "Body position: x " << odometry_.position_body.x_m << " y "
            << odometry_.position_body.y_m << " z "
            << odometry_.position_body.z_m << std::endl;

  std::cout << "Ned position: north " << position_velocity_ned_.position.north_m
            << " east " << position_velocity_ned_.position.east_m << " down "
            << position_velocity_ned_.position.down_m << std::endl;

  std::cout << "Global position: longitude " << global_coordinate.longitude_deg
            << " latitude " << global_coordinate.latitude_deg << std::endl;

  std::cout << "Global position test: longitude " << global_test.longitude_deg
            << " latitude " << global_test.latitude_deg << std::endl;

  std::cout << "Ned position test: x " << local_test.north_m << " y "
            << local_test.east_m << std::endl;

  std::cout << "Initial yaw angle: " << initlal_orientation_.yaw_deg
            << std::endl;

  data_lock_.unlock();
  auto global_test_2 = transformer_->global_from_local({5, 0});

  action_->goto_location(global_test_2.latitude_deg,
                         global_test_2.longitude_deg,
                         initial_global_pos_.absolute_altitude_m + 3, 90);

  // std::cout << T_ned_body_origin.matrix() << std::endl;
}
