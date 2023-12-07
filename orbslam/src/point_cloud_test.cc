/**
 * Depth image and PCL test
 *
 * Dknt 2023.12
 */

#include <cv_bridge/cv_bridge.h>
#include <pcl/point_cloud.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>

#include <mutex>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>

// Intrinsic Parameters
const double fx = 454.6857718666893;
const double fy = 454.6857718666893;
const double cx = 424.5;
const double cy = 240.5;

cv::Mat color_img;
double color_time;
std::mutex color_img_lock;
cv::Mat depth_img;
double depth_time;
std::mutex depth_img_lock;

pcl::visualization::PCLVisualizer viewer("viewer");

bool ColorImgCallback(const sensor_msgs::Image::ConstPtr& msg) {
  std::unique_lock<std::mutex> lock(color_img_lock);
  cv_bridge::CvImagePtr cv_ptr;
  cv_ptr = cv_bridge::toCvCopy(msg, "bgr8");
  color_img = cv_ptr->image;
  color_time = cv_ptr->header.stamp.toSec();

  // std::cout << "Color size: " << color_img.rows << " " << color_img.cols <<
  // std::endl;

  cv::imshow("color", color_img);
  cv::waitKey(1);

  return true;
}

bool DepthImgCallback(const sensor_msgs::Image::ConstPtr& msg) {
  std::unique_lock<std::mutex> lock(depth_img_lock);
  cv_bridge::CvImagePtr cv_ptr;
  cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_32FC1);
  depth_img = cv_ptr->image;
  depth_time = cv_ptr->header.stamp.toSec();

  // std::cout << "Depth size: " << depth_img.rows << " " << depth_img.cols <<
  // std::endl;

  cv::imshow("depth", depth_img);
  cv::waitKey(1);

  return true;
}

void TimerCallback(const ros::TimerEvent& event) {
  std::unique_lock<std::mutex> lock(depth_img_lock);
  std::unique_lock<std::mutex> lock_1(color_img_lock);

  if (color_img.data == nullptr || depth_img.data == nullptr) return;
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(
      new pcl::PointCloud<pcl::PointXYZRGB>());

  for (int y = 0; y < depth_img.rows; y++) {
    auto depth_row = depth_img.ptr<float>(y);
    auto color_row = color_img.ptr<uchar>(y);
    for (int x = 0; x < depth_img.cols; x++) {
      if (depth_row[x] > 0 && depth_row[x] < 50.0) {
        pcl::PointXYZRGB point;
        point.z = depth_row[x];
        point.x = (x - cx) * point.z / fx;
        point.y = (y - cy) * point.z / fy;
        // point.z = 3.0 + 0.01 * depth_row[x];
        point.b = color_row[x * 3];
        point.g = color_row[x * 3 + 1];
        point.r = color_row[x * 3 + 2];
        cloud->push_back(point);
      }
    }
  }

  viewer.removeAllPointClouds();
  viewer.removeCoordinateSystem();

  viewer.addPointCloud(cloud, "cloud");
  viewer.setPointCloudRenderingProperties(
      pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "cloud");
  // viewer.addCoordinateSystem(1.0, "cloud", 0);

  viewer.spinOnce();
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "point_clout_test");
  ros::NodeHandle nh;

  ros::Subscriber color_sub =
      nh.subscribe<sensor_msgs::Image>("/rgb/image_raw", 10, ColorImgCallback);
  ros::Subscriber depth_sub = nh.subscribe<sensor_msgs::Image>(
      "/depth/image_raw", 10, DepthImgCallback);
  ros::Timer timer =
      nh.createTimer(ros::Duration(0.05), TimerCallback, false, false);

  timer.start();

  ros::spin();

  return 0;
}
