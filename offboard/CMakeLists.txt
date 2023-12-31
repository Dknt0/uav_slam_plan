cmake_minimum_required(VERSION 3.0.2)
project(offboard)


set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)


find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
  sensor_msgs
  nav_msgs
)
set(MAVSDK_DIR "/home/dknt/Library/mavsdk_install/lib/cmake/MAVSDK")
find_package(MAVSDK REQUIRED)
find_package(Eigen3 REQUIRED)


catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES offboard
#  CATKIN_DEPENDS roscpp std_msgs
#  DEPENDS system_lib
)


include_directories(
  ${catkin_INCLUDE_DIRS}
  "include/offboard"
  "/home/dknt/Library/mavsdk_install/include/mavsdk"
  "/home/dknt/Library/mavsdk_install/include"
  "/usr/include/eigen3"
)


add_executable(simple_test 
  src/simple_test.cc
  src/autopilot.cc
  src/autopilot_ros1.cc
)
target_link_libraries(simple_test
  ${catkin_LIBRARIES}
  MAVSDK::mavsdk
)

add_executable(position_test 
  src/position_test.cc
  src/autopilot.cc
  src/autopilot_ros1.cc
)
target_link_libraries(position_test
  ${catkin_LIBRARIES}
  MAVSDK::mavsdk
)

add_executable(path_follow 
  src/path_follow.cc
  src/autopilot.cc
  src/autopilot_ros1.cc
)
target_link_libraries(path_follow
  ${catkin_LIBRARIES}
  MAVSDK::mavsdk
)
