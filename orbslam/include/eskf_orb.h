#ifndef ESKF_ORB_H
#define ESKF_ORB_H

#include <iostream>
#include <string>
#include <vector>
#include <queue>
#include <thread>
#include <mutex>

#include <ros/ros.h>
#include <geometry_msgs/TransformStamped.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/Imu.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_broadcaster.h>

#include <cv_bridge/cv_bridge.h>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <sophus/so3.hpp>

#include "System.h"

class ESKF_ORB {
 public:
  ESKF_ORB(ros::NodeHandle nh, const std::string &strVocFil, const std::string &strSettingsFile);

  bool ImuCallback(const sensor_msgs::Imu::ConstPtr &msg);

  void SyncVslamThread();

  bool EskfPrediction(const Eigen::Vector3d acc_m, const Eigen::Vector3d omega_m, const double dt);
  bool EskfCorrection(const Eigen::Vector3d delta_p, const Eigen::Vector3d delta_theta);

 protected:
  ros::NodeHandle nh_;

  ros::Subscriber color_image_sub_;
  ros::Subscriber depth_image_sub_;
  ros::Subscriber imu_sub_;

  ros::Publisher pose_pub_;
  tf2_ros::TransformBroadcaster* tf_broadcaster_;

  std::queue<sensor_msgs::Image::ConstPtr> color_image_queue_;
  std::queue<sensor_msgs::Image::ConstPtr> depth_image_queue_;
  std::mutex image_lock_;

  // Constants
  Eigen::Vector3d sigma_a_;
  Eigen::Vector3d sigma_g_;
  Eigen::Vector3d tbc_;
  // Nominal state variable
  Eigen::Vector3d p_;
  Eigen::Vector3d v_;
  Eigen::Matrix3d Rwb_;
  Eigen::Vector3d ba_;
  Eigen::Vector3d bg_;
  Eigen::Vector3d g_;
  // Error state variable
  Eigen::Matrix<double, 18, 1> delta_;
  // Covariance
  Eigen::Matrix<double, 18, 18> P_;
  Eigen::Matrix<double, 3, 3> K_a;  // Cov matrix of accelerometer
  Eigen::Matrix<double, 3, 3> K_g;
  double last_imu_time_;
  std::mutex state_lock_;
  
  // ORB-SLAM
  ORB_SLAM2::System *orbslam_;
  
  // Threads
  std::thread *vslam_thread_;
};

#endif
