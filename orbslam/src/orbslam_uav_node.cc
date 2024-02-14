/**
 * ORBSLAM UAV ROS Node
 *
 * Dknt 2024.1
 */

#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/TransformStamped.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
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

/* VSLAM */
const std::string path_to_vocabulary =
    "/home/dknt/Projects/uav_ws/src/orbslam/Vocabulary/ORBvoc.txt";
const std::string path_to_settings =
    "/home/dknt/Projects/uav_ws/src/orbslam/config/uav_sim.yaml";
std::queue<sensor_msgs::ImageConstPtr> color_queue;
std::mutex color_img_lock;
std::queue<sensor_msgs::ImageConstPtr> depth_queue;
std::mutex depth_img_lock;

tf2_ros::TransformBroadcaster* tf_broadcaster;
ros::Publisher cloud_pub;

/* PC process */
std::mutex cloud_lock;
std::queue<cv::Mat> pc_color_queue;  // no need for this
std::queue<cv::Mat> pc_depth_queue;
std::queue<Eigen::Isometry3d> pc_pose_queue;

/// Functions for SLAM

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

void SyncThread(ORB_SLAM2::System* orbslam, std::future<void>* exit_future) {
  // 这里可以使用 ros::ok   不需要用 future
  while (exit_future->wait_for(std::chrono::milliseconds(10)) ==
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

      bool is_keyframe = false;
      cv::Mat res =
          orbslam->TrackRGBD_KF(color_img, depth_img, time, is_keyframe);
      // cv::Mat res = orbslam->TrackRGBD(color_img, depth_img, time);

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

      Eigen::Isometry3d T_wc = T_cw.inverse();  // Camera Pose, Oz front

      Eigen::Isometry3d T_bc(
          Eigen::AngleAxisd(-M_PI_2, Eigen::Vector3d::UnitX()) *
          Eigen::AngleAxisd(M_PI_2, Eigen::Vector3d::UnitY()));
      T_wc = T_bc * T_wc;  // Camera Pose, Ox front

      // If this frame is a keyframe, insert it into queue to update local
      // pointcloud
      if (is_keyframe) {
        std::unique_lock<std::mutex> lock(cloud_lock);
        pc_color_queue.push(color_img);
        pc_depth_queue.push(depth_img);
        pc_pose_queue.push(T_wc);
      }

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

/// Functions for local pointcloud

// Intrinsic Parameters
const double fx = 454.6857718666893;
const double fy = 454.6857718666893;
const double cx = 424.5;
const double cy = 240.5;

void LocalPcThread(std::future<void>* exit_future) {
  pcl::PointCloud<pcl::PointXYZ>::Ptr pc_total(
      new pcl::PointCloud<pcl::PointXYZ>());
  int counter = 0;

  // 体素滤波
  pcl::VoxelGrid<pcl::PointXYZ> voxelFilter;
  float resolution = 0.1;
  voxelFilter.setLeafSize(resolution, resolution, resolution);
  // 外点去除滤波
  pcl::StatisticalOutlierRemoval<pcl::PointXYZ> statisticalFilter;
  statisticalFilter.setMeanK(50.0);
  statisticalFilter.setStddevMulThresh(2.0);

  bool flag_filter = false;
  std::promise<void> quit_promise;
  auto filter_quit_future = quit_promise.get_future();
  std::mutex cloud_filter_lock;

  std::thread filter_thread([&]() {
    while (filter_quit_future.wait_for(std::chrono::seconds(3)) ==
           std::future_status::timeout) {
      if (flag_filter) {
        std::unique_lock<std::mutex> lock(cloud_filter_lock);
        voxelFilter.setInputCloud(pc_total);
        voxelFilter.filter(*pc_total);
        sensor_msgs::PointCloud2 pc_msg;
        pcl::toROSMsg(*pc_total, pc_msg);
        pc_msg.header.stamp = ros::Time::now();
        pc_msg.header.frame_id = "map";
        cloud_pub.publish(pc_msg);
        ROS_INFO_STREAM("Filter once. Total size: " << pc_total->size());
        flag_filter = false;
      }
    }
  });

  while (exit_future->wait_for(std::chrono::milliseconds(10)) ==
         std::future_status::timeout) {
    cv::Mat color_img;
    cv::Mat depth_img;
    Eigen::Isometry3d T_wc;
    {
      std::unique_lock<std::mutex> lock(cloud_lock);
      if (!pc_pose_queue.empty()) {
        // 防止队列满
        if (pc_pose_queue.size() > 5) {
          pc_color_queue.pop();
          pc_depth_queue.pop();
          pc_pose_queue.pop();
          continue;
        }
        color_img = pc_color_queue.front();
        depth_img = pc_depth_queue.front();
        T_wc = pc_pose_queue.front();
      } else {
        continue;
      }
    }
    counter++;

    pcl::PointCloud<pcl::PointXYZ>::Ptr pc_temp(
        new pcl::PointCloud<pcl::PointXYZ>());

    for (int y = 0; y + 5 < depth_img.rows; y += 5) {
      auto depth_row = depth_img.ptr<float>(y);
      for (int x = 0; x + 5 < depth_img.cols; x += 5) {
        if (depth_row[x] > 0 && depth_row[x] < 5.0) {
          Eigen::Vector3d position;
          position[2] = depth_row[x];
          position[0] = (x - cx) * position[2] / fx;
          position[1] = (y - cy) * position[2] / fy;
          position = T_wc * position;

          pcl::PointXYZ point;
          point.x = position[0];
          point.y = position[1];
          point.z = position[2];
          pc_temp->push_back(point);
        }
      }
    }
    // statisticalFilter.setInputCloud(pc_temp);
    // statisticalFilter.filter(*pc_temp);

    std::unique_lock<std::mutex> lock(cloud_filter_lock);
    *pc_total += *pc_temp;
    sensor_msgs::PointCloud2 pc_msg;

    pcl::toROSMsg(*pc_total, pc_msg);
    pc_msg.header.stamp = ros::Time::now();
    pc_msg.header.frame_id = "map";

    std::cout << "Insert a KeyFrame in local PC. " << counter
              << " Size: " << pc_total->size() << std::endl;
    cloud_pub.publish(pc_msg);

    flag_filter = true;

    pc_color_queue.pop();
    pc_depth_queue.pop();
    pc_pose_queue.pop();
  }

  quit_promise.set_value();
}

/// Main function

int main(int argc, char** argv) {
  ros::init(argc, argv, "orbslam_uav");
  ros::NodeHandle nh;

  ros::Subscriber color_sub =
      nh.subscribe<sensor_msgs::Image>("/rgb/image_raw", 10, ColorImgCallback);
  ros::Subscriber depth_sub = nh.subscribe<sensor_msgs::Image>(
      "/depth/image_raw", 10, DepthImgCallback);
  cloud_pub = nh.advertise<sensor_msgs::PointCloud2>("local_pointcloud", 10);

  ORB_SLAM2::System orbslam(path_to_vocabulary, path_to_settings,
                            ORB_SLAM2::System::RGBD, true);
  tf_broadcaster = new tf2_ros::TransformBroadcaster;

  std::promise<void> exit_promise;
  std::promise<void> exit_promise_pc;
  std::future<void> exit_future = exit_promise.get_future();
  std::future<void> exit_future_pc = exit_promise_pc.get_future();

  std::thread slam_thread(SyncThread, &orbslam, &exit_future);
  std::thread pc_thread(LocalPcThread, &exit_future_pc);

  ros::spin();

  exit_promise.set_value();
  exit_promise_pc.set_value();

  orbslam.Shutdown();

  delete tf_broadcaster;

  return 0;
}
