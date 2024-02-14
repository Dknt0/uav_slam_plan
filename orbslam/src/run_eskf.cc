#include <ros/ros.h>
#include "eskf_orb.h"

const std::string path_to_vocabulary =
    "/home/dknt/Projects/uav_ws/src/orbslam/Vocabulary/ORBvoc.txt";
const std::string path_to_settings =
    "/home/dknt/Projects/uav_ws/src/orbslam/config/uav_sim.yaml";

int main(int argc, char** argv) {
  ros::init(argc, argv, "eskf_node");
  ros::NodeHandle nh;

  ESKF_ORB eskf(nh, path_to_vocabulary, path_to_settings);

  ros::spin();

  return 0;
}
