cmake_minimum_required(VERSION 2.8.3)
project(road_motion_c)

find_package(catkin REQUIRED COMPONENTS
  cmake_modules
  cv_bridge
  pcl_ros
  roscpp
  rospy
  std_msgs
  velodyne_msgs
  velodyne_pointcloud
)

 find_package(Boost REQUIRED COMPONENTS system)

 find_package(Eigen REQUIRED)

find_package(OpenCV REQUIRED
COMPONENTS
  opencv_highgui
CONFIG)

catkin_package(
  LIBRARIES road_motion_c
  CATKIN_DEPENDS cmake_modules cv_bridge pcl_ros roscpp rospy std_msgs velodyne_msgs velodyne_pointcloud
  DEPENDS system_lib Eigen
)


set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIRS}
)
include_directories(${OpenCV_INCLUDE_DIRS})

add_executable(${PROJECT_NAME} src/road_motion.cpp)

target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

