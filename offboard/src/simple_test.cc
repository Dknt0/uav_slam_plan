#include <ros/ros.h> 

#include "autopilot.h"

const std::string port = "udp://:14550";

using namespace std::chrono_literals;

int main(int argc, char** argv) {
  // ros::init(argc, argv, "autopilot_test");
  // ros::NodeHandle nh = ros::NodeHandle("~");

  Autopilot autopilot(port);

  // autopilot.Test();
  
  autopilot.Arm();
  autopilot.Takeoff(5);

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
