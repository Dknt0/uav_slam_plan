/**
 * 飞控类 ROS 1 接口
 *
 * Dknt 2023.12
 */

#include "autopilot_ros1.h"

AutopilotROS1::AutopilotROS1(ros::NodeHandle nh, const std::string port)
    : nh_(nh), Autopilot(port) {
  odometry_pub_ = nh_.advertise<nav_msgs::Odometry>("autopilot/odom", 10);
  camera_pose_pub_ =
      nh_.advertise<geometry_msgs::PoseStamped>("autopilot/camera_pose", 10);

  // 重新注册数传回调
  telemetry_->subscribe_odometry([this](mavsdk::Telemetry::Odometry odometry) {
    data_lock_.lock();
    this->odometry_ = odometry;
    data_lock_.unlock();

    OdometryPub();
  });
}

/// @brief 发布里程计、相机位姿
void AutopilotROS1::OdometryPub() {
  data_lock_.lock();
  Eigen::Vector3f position_frd{odometry_.position_body.x_m,
                               odometry_.position_body.y_m,
                               odometry_.position_body.z_m};
  Eigen::Quaternionf orientation_frd{odometry_.q.w, odometry_.q.x,
                                     odometry_.q.y, odometry_.q.z};
  Eigen::Vector3f velocity_frd{odometry_.velocity_body.x_m_s,
                               odometry_.velocity_body.y_m_s,
                               odometry_.velocity_body.z_m_s};
  Eigen::Vector3f angle_velocity_frd{
      odometry_.angular_velocity_body.roll_rad_s,
      odometry_.angular_velocity_body.pitch_rad_s,
      odometry_.angular_velocity_body.yaw_rad_s};
  data_lock_.unlock();

  // 发布里程计
  nav_msgs::Odometry odom_msg;
  odom_msg.header.stamp = ros::Time::now();  // ROS 消息中统一使用 ROS 时间
  odom_msg.header.frame_id = "autopilot_odom";
  odom_msg.child_frame_id = "drone_frame";

  Eigen::AngleAxisf flu_frd(M_PI, Eigen::Vector3f::UnitX());

  Eigen::Isometry3f T_Wfrd_Bfrd(orientation_frd);
  T_Wfrd_Bfrd.pretranslate(position_frd);

  Eigen::Isometry3f T_Wflu_Wfrd(
      Eigen::AngleAxisf(M_PI, Eigen::Vector3f::UnitX()));
  Eigen::Isometry3f T_Bfrd_Bflu(
      Eigen::AngleAxisf(-M_PI, Eigen::Vector3f::UnitX()));
  Eigen::Isometry3f T_Wflu_Bflu = T_Wflu_Wfrd * T_Wfrd_Bfrd * T_Bfrd_Bflu;

  Eigen::Vector3f position_flu = T_Wflu_Bflu.translation();
  Eigen::Quaternionf orientation_flu(T_Wflu_Bflu.rotation());
  Eigen::Vector3f velocity_flu = T_Wflu_Wfrd * velocity_frd;
  Eigen::Vector3f angle_velocity_flu = T_Wflu_Wfrd * angle_velocity_frd;

  odom_msg.pose.pose.position.x = position_flu[0];
  odom_msg.pose.pose.position.y = position_flu[1];
  odom_msg.pose.pose.position.z = position_flu[2];

  odom_msg.pose.pose.orientation.w = orientation_flu.w();
  odom_msg.pose.pose.orientation.x = orientation_flu.x();
  odom_msg.pose.pose.orientation.y = orientation_flu.y();
  odom_msg.pose.pose.orientation.z = orientation_flu.z();

  odom_msg.twist.twist.linear.x = velocity_flu[0];
  odom_msg.twist.twist.linear.y = velocity_flu[1];
  odom_msg.twist.twist.linear.z = velocity_flu[2];

  odom_msg.twist.twist.angular.x = angle_velocity_flu[0];
  odom_msg.twist.twist.angular.y = angle_velocity_flu[1];
  odom_msg.twist.twist.angular.z = angle_velocity_flu[2];

  odometry_pub_.publish(odom_msg);

  // 发布相机位姿  z 前
  Eigen::Isometry3f Tbc(Eigen::Quaternionf(-0.5, 0.5, -0.5, 0.5));
  Tbc.pretranslate(Eigen::Vector3f(0.1, 0, 0));

  Eigen::Isometry3f Twc = T_Wflu_Bflu * Tbc;
  Eigen::Vector3f t_wc = Twc.translation();
  Eigen::Quaternionf q_wc(Twc.rotation());

  geometry_msgs::PoseStamped camera_pose;

  camera_pose.header.stamp = ros::Time::now();
  camera_pose.header.frame_id = "autopilot_odom";  // 相对里程计坐标系
  
  camera_pose.pose.position.x = t_wc[0];
  camera_pose.pose.position.y = t_wc[1];
  camera_pose.pose.position.z = t_wc[2];

  camera_pose.pose.orientation.w = q_wc.w();
  camera_pose.pose.orientation.x = q_wc.x();
  camera_pose.pose.orientation.y = q_wc.y();
  camera_pose.pose.orientation.z = q_wc.z();

  camera_pose_pub_.publish(camera_pose);
}
