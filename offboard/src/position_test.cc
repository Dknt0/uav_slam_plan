/**
 * 位置控制 Debug
 * 
 * 测试坐标系关系
 * 
 * Dknt 2023.12
 */

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

using namespace std::chrono_literals;

std::string port = "udp://:14550";

int main(int argc, char **argv) {
  // 日志等级
  mavsdk::log::subscribe([](mavsdk::log::Level level,
                            const std::string &message, const std::string &file,
                            int line) {
    (void)message;
    (void)file;
    (void)line;
    return (level != mavsdk::log::Level::Err);
  });

  mavsdk::Mavsdk mavsdk;
  mavsdk.add_any_connection(port);

  while (mavsdk.systems().size() == 0) {
    std::this_thread::sleep_for(0.5s);
  }

  auto system_ = mavsdk.systems()[0];
  auto telemetry_ = mavsdk::Telemetry(system_);
  auto action_ = mavsdk::Action(system_);
  auto offboard_ = mavsdk::Offboard(system_);

  while (telemetry_.health_all_ok()) {
    std::this_thread::sleep_for(0.5s);
  }

  
  // 启动
  std::cout << "Arming." << std::endl;
  action_.arm();
  action_.set_takeoff_altitude(4.0);

  // 初始位置
  // 初始位置的获取可能失败，应该在连接建立一段时间后再进行
  auto initial_global_pos_ = telemetry_.position();
  auto initlal_orientation_ = telemetry_.attitude_euler();
  mavsdk::geometry::CoordinateTransformation::GlobalCoordinate
      global_coordinate;
  global_coordinate.longitude_deg = initial_global_pos_.longitude_deg;
  global_coordinate.latitude_deg = initial_global_pos_.latitude_deg;
  auto transformer_ =
      mavsdk::geometry::CoordinateTransformation(global_coordinate);

  std::cout << initial_global_pos_ << std::endl;
  std::cout << initlal_orientation_ << std::endl;

  // 起飞
  std::cin.get();
  std::cout << "Taking off." << std::endl;
  action_.takeoff();

  // 路径点
  {
    std::cin.get();
    std::cout << "Going to target." << std::endl;
    auto global_test_2 = transformer_.global_from_local({5, 0});
    std::promise<void> goto_promise;
    std::future<void> goto_future = goto_promise.get_future();
    // 这个函数不代表到达位置，只发送命令
    action_.goto_location_async(global_test_2.latitude_deg,
                                global_test_2.longitude_deg,
                                initial_global_pos_.absolute_altitude_m + 4.0,
                                90, [&](mavsdk::Action::Result res) {
                                  std::cout << "Res callback" << std::endl;
                                  goto_promise.set_value();
                                });
    while (goto_future.wait_for(0.1s) == std::future_status::timeout) {
      std::cout << ".";
      std::this_thread::sleep_for(1s);
    }
    // std::cout << std::endl;
  }

  {
    std::cin.get();
    std::cout << "Going to target." << std::endl;
    auto global_test_2 = transformer_.global_from_local({0, 5});
    action_.goto_location(global_test_2.latitude_deg,
                                global_test_2.longitude_deg,
                                initial_global_pos_.absolute_altitude_m + 4.0,
                                90);
    // std::cout << std::endl;
  }

  {
    std::cin.get();
    std::cout << "Going to target." << std::endl;
    auto global_test_2 = transformer_.global_from_local({0, 0});
    std::promise<void> goto_promise;
    std::future<void> goto_future = goto_promise.get_future();
    action_.goto_location_async(global_test_2.latitude_deg,
                                global_test_2.longitude_deg,
                                initial_global_pos_.absolute_altitude_m + 4.0,
                                90, [&](mavsdk::Action::Result res) {
                                  std::cout << "Res callback" << std::endl;
                                  goto_promise.set_value();
                                });
    while (goto_future.wait_for(0.1s) == std::future_status::timeout) {
      std::cout << ".";
      std::this_thread::sleep_for(1s);
    }
    // std::cout << std::endl;
  }

  std::cin.get();
  action_.land();

  return 0;
}
