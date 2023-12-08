#include <ros/ros.h>

#include "autopilot_ros1.h"

const std::string port = "udp://:14550";

using namespace std::chrono_literals;

int main(int argc, char** argv) {
  ros::init(argc, argv, "autopilot_test");
  ros::NodeHandle nh = ros::NodeHandle();

  AutopilotROS1 autopilot(nh, port);

  // autopilot.Test();

  if (!autopilot.Arm()) exit(1);
  if (!autopilot.Takeoff(5)) exit(1);

  autopilot.StartOffboardVelocity();

  std::cin.get();
  autopilot.SetPointOffboardVelocity(0, 0, 2, 0);
  std::cin.get();
  autopilot.SetPointOffboardVelocity();
  std::cin.get();

  autopilot.FinishOffboardVelocity();

  autopilot.Land();

  return 0;
}
