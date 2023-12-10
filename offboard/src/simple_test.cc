#include <ros/ros.h>

#include "autopilot_ros1.h"

const std::string port = "udp://:14550";

using namespace std::chrono_literals;

int main(int argc, char** argv) {
  ros::init(argc, argv, "autopilot_test");
  ros::NodeHandle nh = ros::NodeHandle();

  AutopilotROS1 autopilot(nh, port);

  // char key{'0'};
  // while (key != 'q') {
  //   autopilot.CoordinateTest();
  //   std::cin >> key;
  // }

  if (!autopilot.Arm()) exit(1);
  if (!autopilot.Takeoff(4)) exit(1);

  std::cin.get();
  autopilot.GotoPosition(5, 0, 4, 0);

  std::cin.get();
  autopilot.GotoPosition(0, 5, 4, 0);

  std::cin.get();
  autopilot.GotoPosition(0, 0, 4, 1.57);

  std::cin.get();
  autopilot.GotoPosition(0, 0, 4, 0);
  // autopilot.StartOffboardPosition();

  // std::cin.get();
  // autopilot.SetPointOffboardPosition(3, 0, 3, 0);
  // std::cin.get();
  // autopilot.SetPointOffboardPosition(0, 0, 3, 0);
  // std::cin.get();

  // autopilot.ExitOffboardPosition();

  std::cin.get();
  autopilot.Land();

  return 0;
}
