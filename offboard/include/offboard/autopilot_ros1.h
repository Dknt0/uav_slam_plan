/**
 * 飞控类 ROS 1 接口
 *
 * Dknt 2023.12
 */

#ifndef OFFBOARD_AUTOPOLOT_ROS1_H
#define OFFBOARD_AUTOPOLOT_ROS1_H

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include "autopilot.h"

class AutopilotROS1 : public Autopilot {
 public:
  AutopilotROS1(ros::NodeHandle nh, const std::string port = "udp://:14550");


 private:
  void OdometryPub();

  ros::NodeHandle nh_;
  ros::Publisher odometry_pub_;
  ros::Publisher camera_pose_pub_;
};

#endif  // OFFBOARD_AUTOPOLOT_ROS1_H
