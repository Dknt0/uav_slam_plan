#include <geometry_msgs/Pose.h>
#include <ros/ros.h>

#include "autopilot_ros1.h"

const std::string port = "udp://:14550";

using namespace std::chrono_literals;

AutopilotROS1* autopilot;

void pose_cmd_callback(geometry_msgs::Pose::ConstPtr msg) {
  Eigen::Quaterniond q(msg->orientation.w, msg->orientation.x,
                       msg->orientation.y, msg->orientation.z);
  double yaw = q.matrix().matrix().eulerAngles(0, 1, 2)[2];
  std::cout << "Got pose command: " << msg->position.x << " " << msg->position.y
            << " " << msg->position.z << " " << yaw << " "
            << (yaw + M_PI_2) << std::endl;

  autopilot->GotoPosition(-msg->position.y, msg->position.x, msg->position.z,
                          (yaw + M_PI_2));
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "autopilot_test");
  ros::NodeHandle nh = ros::NodeHandle();
  ros::Subscriber pose_cmd_sub = nh.subscribe<geometry_msgs::Pose>(
      "/planning/pose_cmd", 10, pose_cmd_callback);

  autopilot = new AutopilotROS1(nh, port);

  if (!autopilot->Arm()) exit(1);
  if (!autopilot->Takeoff(2.5)) exit(1);
  if (!autopilot->SetVelocity(1.0)) exit(1);
  // std::cout << "Press Enter." << std::endl;
  // std::cin.get();

  while (ros::ok()) {
    ros::spinOnce();
  }

  if (!autopilot->Land()) autopilot->Shutdown();

  return 0;
}
