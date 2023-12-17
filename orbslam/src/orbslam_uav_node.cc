/**
 * ORBSLAM UAV ROS Node
 *
 * Dknt 2023.12
 */

#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/TransformStamped.h>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_broadcaster.h>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <chrono>
#include <future>
#include <mutex>
#include <opencv2/core.hpp>
#include <queue>
#include <thread>

#include "System.h"

std::queue<sensor_msgs::ImageConstPtr> color_queue;
std::mutex color_img_lock;
std::queue<sensor_msgs::ImageConstPtr> depth_queue;
std::mutex depth_img_lock;

tf2_ros::TransformBroadcaster* tf_broadcaster;

bool ColorImgCallback(const sensor_msgs::Image::ConstPtr& msg) {
  std::unique_lock<std::mutex> lock(color_img_lock);
  color_queue.push(msg);

  return true;
}

bool DepthImgCallback(const sensor_msgs::Image::ConstPtr& msg) {
  std::unique_lock<std::mutex> lock(depth_img_lock);
  depth_queue.push(msg);

  return true;
}

// 同步线程
void SyncThread(ORB_SLAM2::System* orbslam, std::future<void>* exit_future) {
  while (exit_future->wait_for(std::chrono::milliseconds(1)) ==
         std::future_status::timeout) {
    std::unique_lock<std::mutex> lock_1(color_img_lock);
    std::unique_lock<std::mutex> lock_2(depth_img_lock);

    if (!color_queue.empty() && !depth_queue.empty()) {
      double time_color = color_queue.front()->header.stamp.toSec();
      double time_depth = depth_queue.front()->header.stamp.toSec();

      if (time_color > time_depth + 0.003) {
        depth_queue.pop();
        std::cout << "Throw depth image." << std::endl;
        continue;
      }

      if (time_depth > time_color + 0.003) {
        color_queue.pop();
        std::cout << "Throw color image." << std::endl;
        continue;
      }

      cv::Mat color_img, depth_img;
      double time = color_queue.front()->header.stamp.toSec();

      cv_bridge::CvImagePtr cv_ptr;
      cv_ptr = cv_bridge::toCvCopy(color_queue.front(), "bgr8");
      color_img = cv_ptr->image;
      cv_ptr = cv_bridge::toCvCopy(depth_queue.front(),
                                   sensor_msgs::image_encodings::TYPE_32FC1);
      depth_img = cv_ptr->image;
      color_queue.pop();
      depth_queue.pop();

      
      cv::Mat res = orbslam->TrackRGBD(color_img, depth_img, time);

      // 判断是否失效
      if (res.data == nullptr) {
        continue;
      }

      Eigen::Matrix3d R;
      Eigen::Vector3d t;
      R << res.at<float>(0, 0), res.at<float>(0, 1), res.at<float>(0, 2),
          res.at<float>(1, 0), res.at<float>(1, 1), res.at<float>(1, 2),
          res.at<float>(2, 0), res.at<float>(2, 1), res.at<float>(2, 2);
      t << res.at<float>(0, 3), res.at<float>(1, 3), res.at<float>(2, 3);
      Eigen::Isometry3d T_cw(R);
      T_cw.pretranslate(t);

      Eigen::Isometry3d T_wc = T_cw.inverse();

      Eigen::Isometry3d T_bc(
          Eigen::AngleAxisd(-M_PI_2, Eigen::Vector3d::UnitX()) *
          Eigen::AngleAxisd(M_PI_2, Eigen::Vector3d::UnitY()));
      T_wc = T_bc * T_wc;

      Eigen::Quaterniond q(T_wc.rotation());
      t = T_wc.translation();

      // 发布位姿消息
      geometry_msgs::TransformStamped camera_pose;
      camera_pose.header.frame_id = "world";
      camera_pose.header.stamp = ros::Time::now();
      camera_pose.child_frame_id = "camera_link";
      camera_pose.transform.translation.x = t[0];
      camera_pose.transform.translation.y = t[1];
      camera_pose.transform.translation.z = t[2];
      camera_pose.transform.rotation.w = q.w();
      camera_pose.transform.rotation.x = q.x();
      camera_pose.transform.rotation.y = q.y();
      camera_pose.transform.rotation.z = q.z();
      tf_broadcaster->sendTransform(camera_pose);

      // std::cout << res << std::endl;
    }
  }
}

std::string path_to_vocabulary =
    "/home/dknt/Projects/uav_ws/src/orbslam/Vocabulary/ORBvoc.txt";
std::string path_to_settings =
    "/home/dknt/Projects/uav_ws/src/orbslam/config/uav_sim.yaml";

int main(int argc, char** argv) {
  ros::init(argc, argv, "orbslam_uav");
  ros::NodeHandle nh;

  ros::Subscriber color_sub =
      nh.subscribe<sensor_msgs::Image>("/rgb/image_raw", 10, ColorImgCallback);
  ros::Subscriber depth_sub = nh.subscribe<sensor_msgs::Image>(
      "/depth/image_raw", 10, DepthImgCallback);

  ORB_SLAM2::System orbslam(path_to_vocabulary, path_to_settings,
                            ORB_SLAM2::System::RGBD, true);
  tf_broadcaster = new tf2_ros::TransformBroadcaster;

  std::promise<void> exit_promise;
  std::future<void> exit_future = exit_promise.get_future();

  std::thread slam_thread(SyncThread, &orbslam, &exit_future);

  ros::spin();

  exit_promise.set_value();

  orbslam.Shutdown();

  delete tf_broadcaster;

  return 0;
}
