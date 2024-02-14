#include "eskf_orb.h"

using namespace std::chrono_literals;

ESKF_ORB::ESKF_ORB(ros::NodeHandle nh, const std::string& strVocFil,
                   const std::string& strSettingsFile) {
  // Initialize state variables
  p_.setZero();
  v_.setZero();
  Rwb_.setIdentity();
  ba_.setZero();
  bg_.setZero();
  g_ = Eigen::Vector3d(0, 0, -9.8);
  delta_.setZero();
  P_.setIdentity();

  // Set sensor Cov
  sigma_g_ = Eigen::Vector3d(0.01, 0.01, 0.01);
  sigma_a_ = Eigen::Vector3d(0.01, 0.01, 0.01);
  K_g.setZero();
  K_a.setZero();
  K_g(0, 0) = sigma_g_(0) * sigma_g_(0);
  K_g(1, 1) = sigma_g_(1) * sigma_g_(1);
  K_g(2, 2) = sigma_g_(2) * sigma_g_(2);
  K_a(0, 0) = sigma_a_(0) * sigma_a_(0);
  K_a(1, 1) = sigma_a_(1) * sigma_a_(1);
  K_a(2, 2) = sigma_a_(2) * sigma_a_(2);

  tbc_ = Eigen::Vector3d(0.1, 0, 0);  // Camera is located 10cm in front of the drone

  last_imu_time_ = ros::Time::now().toSec();  // This makes no sense

  orbslam_ = new ORB_SLAM2::System(strVocFil, strSettingsFile,
                                   ORB_SLAM2::System::RGBD, false);

  // Initialize ROS pubs and subs
  color_image_sub_ = nh.subscribe<sensor_msgs::Image>(
      "/rgb/image_raw", 10, [&](const sensor_msgs::Image::ConstPtr& msg) {
        std::unique_lock<std::mutex> lock(image_lock_);
        color_image_queue_.push(msg);
        return true;
      });

  depth_image_sub_ = nh.subscribe<sensor_msgs::Image>(
      "/depth/image_raw", 10, [&](const sensor_msgs::Image::ConstPtr& msg) {
        std::unique_lock<std::mutex> lock(image_lock_);
        depth_image_queue_.push(msg);
        return true;
      });

  imu_sub_ = nh.subscribe<sensor_msgs::Imu>(
      "/imu_uav", 10,
      std::bind(&ESKF_ORB::ImuCallback, this, std::placeholders::_1));

  tf_broadcaster_ = new tf2_ros::TransformBroadcaster;

  // Launch threads
  vslam_thread_ = new std::thread(&ESKF_ORB::SyncVslamThread, this);
}

void ESKF_ORB::SyncVslamThread() {
  while (ros::ok()) {
    cv::Mat color_img, depth_img;
    double time;
    {
      std::unique_lock<std::mutex> lock(image_lock_);

      if (color_image_queue_.empty() || depth_image_queue_.empty()) {
        continue;
      }
      double time_color = color_image_queue_.front()->header.stamp.toSec();
      double time_depth = depth_image_queue_.front()->header.stamp.toSec();

      if (time_color > time_depth + 0.003) {
        depth_image_queue_.pop();
        std::cout << "Throw depth image." << std::endl;
        continue;
      }

      if (time_depth > time_color + 0.003) {
        color_image_queue_.pop();
        std::cout << "Throw color image." << std::endl;
        continue;
      }

      time = color_image_queue_.front()->header.stamp.toSec();

      cv_bridge::CvImagePtr cv_ptr;
      cv_ptr = cv_bridge::toCvCopy(color_image_queue_.front(), "bgr8");
      color_img = cv_ptr->image;
      cv_ptr = cv_bridge::toCvCopy(depth_image_queue_.front(),
                                   sensor_msgs::image_encodings::TYPE_32FC1);
      depth_img = cv_ptr->image;
      color_image_queue_.pop();
      depth_image_queue_.pop();
    }

    cv::Mat res = orbslam_->TrackRGBD(color_img, depth_img, time);

    // Wait for VSLAM is stable
    if (res.data == nullptr) {
      continue;
    }

    Eigen::Matrix3d Rcw;  // Rwc
    Eigen::Vector3d tcw;  // twc
    Rcw << res.at<float>(0, 0), res.at<float>(0, 1), res.at<float>(0, 2),
        res.at<float>(1, 0), res.at<float>(1, 1), res.at<float>(1, 2),
        res.at<float>(2, 0), res.at<float>(2, 1), res.at<float>(2, 2);
    tcw << res.at<float>(0, 3), res.at<float>(1, 3), res.at<float>(2, 3);
    Eigen::Isometry3d T_cw(Rcw);
    T_cw.pretranslate(tcw);

    Eigen::Isometry3d T_wc = T_cw.inverse();  // Camera Pose, Oz front

    Eigen::Isometry3d T_bc(
        Eigen::AngleAxisd(-M_PI_2, Eigen::Vector3d::UnitX()) *
        Eigen::AngleAxisd(M_PI_2, Eigen::Vector3d::UnitY()));
    T_wc = T_bc * T_wc;  // Camera Pose, Ox front

    Eigen::Matrix3d R_bc(Eigen::AngleAxisd(-M_PI_2, Eigen::Vector3d::UnitX()) *
                         Eigen::AngleAxisd(M_PI_2, Eigen::Vector3d::UnitY()));

    Eigen::Quaterniond q(T_wc.rotation());
    Eigen::Vector3d twc = T_wc.translation();

    Eigen::Vector3d delta_p;
    Eigen::Vector3d delta_theta;
    delta_p = twc - Rwb_ * tbc_ - p_;
    delta_theta =
        Sophus::SO3d(Eigen::Quaterniond(Rwb_.transpose() * T_wc.rotation() *
                                        R_bc.transpose())
                         .normalized())
            .log();
    // ESKF Correction
    EskfCorrection(delta_p, delta_theta);

    geometry_msgs::TransformStamped camera_pose;
    camera_pose.header.frame_id = "world";
    camera_pose.header.stamp = ros::Time::now();
    camera_pose.child_frame_id = "camera_link";
    camera_pose.transform.translation.x = twc[0];
    camera_pose.transform.translation.y = twc[1];
    camera_pose.transform.translation.z = twc[2];
    camera_pose.transform.rotation.w = q.w();
    camera_pose.transform.rotation.x = q.x();
    camera_pose.transform.rotation.y = q.y();
    camera_pose.transform.rotation.z = q.z();
    tf_broadcaster_->sendTransform(camera_pose);
    std::this_thread::sleep_for(10ms);
  }
}

bool ESKF_ORB::ImuCallback(const sensor_msgs::Imu::ConstPtr& msg) {
  static bool init_flag = true;

  if (init_flag) {
    last_imu_time_ = msg->header.stamp.toSec();
    init_flag = false;

    // TODO: initial bais estimate
    return true;
  }

  // IMU angle rate
  Eigen::Vector3d omega_m;
  omega_m[0] = msg->angular_velocity.x;
  omega_m[1] = msg->angular_velocity.y;
  omega_m[2] = msg->angular_velocity.z;

  // IMU acceleration
  Eigen::Vector3d acc_m;
  acc_m[0] = msg->linear_acceleration.x;
  acc_m[1] = msg->linear_acceleration.y;
  acc_m[2] = msg->linear_acceleration.z;

  double time = msg->header.stamp.toSec();
  double dt = time - last_imu_time_;

  // ESKF Prediction
  EskfPrediction(acc_m, omega_m, dt);

  last_imu_time_ = time;
  return true;
}

bool ESKF_ORB::EskfPrediction(const Eigen::Vector3d acc_m,
                              const Eigen::Vector3d omega_m, const double dt) {
  std::unique_lock<std::mutex> lock(state_lock_);

  // Motion function matrix
  Eigen::Matrix<double, 18, 18> F;
  F.setIdentity();
  F.block<3, 3>(0, 3) = Eigen::Matrix3d::Identity() * dt;
  F.block<3, 3>(3, 6) = -Rwb_ * Sophus::SO3d::hat((acc_m - ba_) * dt);
  F.block<3, 3>(3, 12) = -Rwb_ * dt;
  F.block<3, 3>(3, 15) = Eigen::Matrix3d::Identity() * dt;
  F.block<3, 3>(6, 6) = Sophus::SO3d::exp(-(omega_m - bg_) * dt).matrix();
  F.block<3, 3>(6, 9) = -Eigen::Matrix3d::Identity() * dt;

  // Update cov matrix
  Eigen::Matrix<double, 18, 18> Q;
  Q.setZero();
  Q.block<3, 3>(3, 3) = K_a * dt * dt;
  Q.block<3, 3>(6, 6) = K_g * dt * dt;
  P_ = F * P_ * F.transpose() + Q;

  // Update nominal states
  p_ = p_ + v_ * dt + 0.5 * (Rwb_ * (acc_m - ba_) + g_) * dt * dt;
  v_ = v_ + (Rwb_ * (acc_m - ba_) + g_) * dt;
  Rwb_ = Rwb_ * Sophus::SO3d::exp((omega_m - bg_) * dt).matrix();

  Eigen::Quaterniond qwb(Rwb_);

  // std::cout << "dt: " << dt << std::endl;
  // std::cout << "Pos: " << p_.transpose() << " Ori: " <<
  // qwb.coeffs().transpose()
  //           << std::endl;

  geometry_msgs::TransformStamped body_pose;
  body_pose.header.frame_id = "world";
  body_pose.header.stamp = ros::Time::now();
  body_pose.child_frame_id = "my_imu_link";
  body_pose.transform.translation.x = p_[0];
  body_pose.transform.translation.y = p_[1];
  body_pose.transform.translation.z = p_[2];
  body_pose.transform.rotation.w = qwb.w();
  body_pose.transform.rotation.x = qwb.x();
  body_pose.transform.rotation.y = qwb.y();
  body_pose.transform.rotation.z = qwb.z();
  tf_broadcaster_->sendTransform(body_pose);

  return true;
}

bool ESKF_ORB::EskfCorrection(const Eigen::Vector3d delta_p_m,
                              const Eigen::Vector3d delta_theta_m) {
  std::unique_lock<std::mutex> lock_s(state_lock_);

  // Measurement
  Eigen::Matrix<double, 6, 1> z;
  Eigen::Map<Eigen::Vector3d> z_p_e(z.data());
  Eigen::Map<Eigen::Vector3d> z_theta_e(z.data() + 3);
  z_theta_e = delta_theta_m;
  z_p_e = delta_p_m;

  // Predicted measurement
  Eigen::Matrix<double, 6, 1> z_pred;
  z_pred.setZero();

  // std::cout << z_pred.transpose() << std::endl;
  // std::cout << z.transpose() << std::endl;

  // Measurement matrix
  Eigen::Matrix<double, 6, 18> H;
  H.setZero();
  H.block<3, 3>(0, 0).setIdentity();
  H.block<3, 3>(3, 6).setIdentity();

  // Cov of ORB-SLAM
  // The lower VSLAM covariance, the faster ESKF converges
  Eigen::Matrix<double, 6, 6> V;
  V = Eigen::Matrix<double, 6, 6>::Identity() * 0.0000001;

  // Kalman gain matrix
  Eigen::Matrix<double, 18, 6> K;
  K.setZero();
  K = P_ * H.transpose() * (H * P_ * H.transpose() + V).inverse();

  // Error states
  delta_ = K * (z - z_pred);
  Eigen::Map<Eigen::Vector3d> delta_p(delta_.data());
  Eigen::Map<Eigen::Vector3d> delta_v(delta_.data() + 3);
  Eigen::Map<Eigen::Vector3d> delta_theta(delta_.data() + 6);
  Eigen::Map<Eigen::Vector3d> delta_bg(delta_.data() + 9);
  Eigen::Map<Eigen::Vector3d> delta_ba(delta_.data() + 12);
  Eigen::Map<Eigen::Vector3d> delta_g(delta_.data() + 15);

  // Update nominal state
  p_ = p_ + delta_p;
  v_ = v_ + delta_v;
  Rwb_ = Rwb_ * Sophus::SO3d::exp(delta_theta).matrix();
  bg_ = bg_ + delta_bg;
  ba_ = ba_ + delta_ba;
  g_ = g_ + delta_g;

  // Update cov. matrix
  P_ = (Eigen::Matrix<double, 18, 18>::Identity() - K * H) * P_;

  // Reset error states
  delta_.setZero();

  return true;
}
