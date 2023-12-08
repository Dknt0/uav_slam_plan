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
  while (mavsdk_->systems().size() == 0) {
    std::this_thread::sleep_for(0.5s);
  }
  system_ = mavsdk_->systems()[0];
  telemetry_ = std::make_shared<mavsdk::Telemetry>(system_);
  action_ = std::make_shared<mavsdk::Action>(system_);
  offboard_ = std::make_shared<mavsdk::Offboard>(system_);
  std::cout << AUTOPILOT_MESSAGE_HEAD << "Connected." << std::endl;

  // position callback NED
  {
    auto error = telemetry_->set_rate_position(telemetry_update_rate_);
    if (error != mavsdk::Telemetry::Result::Success) {
      std::cout << AUTOPILOT_ERROR_HEAD
                << "Failed to setting telemetry position rate: "
                << static_cast<int>(error) << std::endl;
    }
  }
  telemetry_->subscribe_position([this](mavsdk::Telemetry::Position position) {
    position_lock_.lock();
    this->position_ned_ = position;
    position_lock_.unlock();
  });

  // velocity callback NED
  {
    auto error = telemetry_->set_rate_velocity_ned(telemetry_update_rate_);
    if (error != mavsdk::Telemetry::Result::Success) {
      std::cout << AUTOPILOT_ERROR_HEAD
                << "Failed to setting telemetry velocity rate: "
                << static_cast<int>(error) << std::endl;
    }
  }
  telemetry_->subscribe_velocity_ned(
      [this](mavsdk::Telemetry::VelocityNed velocity_ned) {
        velocity_ned_lock_.lock();
        this->velocity_ned_ = velocity_ned;
        velocity_ned_lock_.unlock();
      });

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
    odometry_lock_.lock();
    this->odometry_ = odometry;
    odometry_lock_.unlock();
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
    imu_lock_.lock();
    this->imu_ = imu;
    imu_lock_.unlock();
  });

  std::cout << AUTOPILOT_MESSAGE_HEAD << "Initialization completed."
            << std::endl;
}

/// @brief 启动
/// @return
bool Autopilot::Arm() {
  std::cout << AUTOPILOT_MESSAGE_HEAD << "Waiting for RTF..." << std::endl;
  while (telemetry_->health_all_ok()) {
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
  while (true) {
    position_lock_.lock();
    auto cur_altitude = this->position_ned_.relative_altitude_m;
    position_lock_.unlock();
    if (cur_altitude > altitude - 1) {
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
  while (telemetry_->in_air()) {
    auto cur_altitude = this->position_ned_.relative_altitude_m;
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

/// @brief 开启外部控制模式
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
           std::future_status::timeout) {
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

/// @brief 结束外部控制模式
/// @return
bool Autopilot::FinishOffboardVelocity() {
  offboard_velocity_promise_.set_value();
  offboard_velocity_thread_.join();
  std::cout << AUTOPILOT_MESSAGE_HEAD << "Finished offboard mode." << std::endl;
  return true;
}

/// @brief 设置外部控制模式 FRD
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
