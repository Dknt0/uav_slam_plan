cmake_minimum_required(VERSION 3.1.0)
project(orbslam)

SET(CMAKE_BUILD_TYPE Release)
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -O3")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14")


find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  cv_bridge
  sensor_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
)
find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Pangolin REQUIRED)
find_package(PCL REQUIRED)
find_package(fmt REQUIRED)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES offboard
#  CATKIN_DEPENDS roscpp std_msgs
#  DEPENDS system_lib
)

include_directories(
  ${OpenCV_INCLUDE}
  ${catkin_INCLUDE_DIRS}
  ${PROJECT_SOURCE_DIR}
  ${PROJECT_SOURCE_DIR}/include
  /usr/include/eigen3
  ${Pangolin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  /home/dknt/Library/sophus_install/include
)

# # set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/lib)
# add_library(orbslam SHARED
#   src/System.cc
#   src/Tracking.cc
#   src/LocalMapping.cc
#   src/LoopClosing.cc
#   src/ORBextractor.cc
#   src/ORBmatcher.cc
#   src/FrameDrawer.cc
#   src/Converter.cc
#   src/MapPoint.cc
#   src/KeyFrame.cc
#   src/Map.cc
#   src/MapDrawer.cc
#   src/Optimizer.cc
#   src/PnPsolver.cc
#   src/Frame.cc
#   src/KeyFrameDatabase.cc
#   src/Sim3Solver.cc
#   src/Initializer.cc
#   src/Viewer.cc
# )
# target_link_libraries(orbslam
#   ${OpenCV_LIBS}
#   ${Pangolin_LIBRARIES}
#   ${PROJECT_SOURCE_DIR}/Thirdparty/DBoW2/lib/libDBoW2.so
#   ${PROJECT_SOURCE_DIR}/Thirdparty/g2o/lib/libg2o.so
# )


# add_executable(orb_test src/orb_test.cc)
# target_link_libraries(orb_test
#   ${catkin_LIBRARIES}    
#   # ${PROJECT_SOURCE_DIR}/lib/liborbslam.so
#   orbslam
#   ${OpenCV_LIBS}
#   ${Pangolin_LIBRARIES}
#   ${PROJECT_SOURCE_DIR}/Thirdparty/DBoW2/lib/libDBoW2.so
#   ${PROJECT_SOURCE_DIR}/Thirdparty/g2o/lib/libg2o.so
#   # ${PCL_LIBRARIES}
# )

# add_executable(orbslam_uav src/orbslam_uav_node.cc)
# target_link_libraries(orbslam_uav
#   ${catkin_LIBRARIES}    
#   ${OpenCV_LIBS}
#   ${Pangolin_LIBRARIES}
#   ${PROJECT_SOURCE_DIR}/Thirdparty/DBoW2/lib/libDBoW2.so
#   ${PROJECT_SOURCE_DIR}/Thirdparty/g2o/lib/libg2o.so
#   ${PCL_LIBRARIES}
#   orbslam
# )

# add_executable(point_cloud_test src/point_cloud_test.cc)
# target_link_libraries(point_cloud_test
#   ${catkin_LIBRARIES}
#   ${OpenCV_LIBS}
#   ${PCL_LIBRARIES}
# )

add_executable(eskf_orb src/eskf_orb.cc src/run_eskf.cc)
target_link_libraries(eskf_orb
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS}
  ${Pangolin_LIBRARIES}
  ${PROJECT_SOURCE_DIR}/Thirdparty/DBoW2/lib/libDBoW2.so
  ${PROJECT_SOURCE_DIR}/Thirdparty/g2o/lib/libg2o.so
  # ${PCL_LIBRARIES}
  # orbslam
  /home/dknt/Projects/uav_ws/devel/lib/liborbslam.so
  fmt::fmt
)
