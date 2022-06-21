cmake_minimum_required(VERSION 2.8.3)
project(pc_adder)

find_package(catkin REQUIRED COMPONENTS
  cmake_modules
  cv_bridge
  pcl_ros
  roscpp
  rospy
  gmsl_frame_msg
  std_msgs
  ouster_ros
)

 find_package(Boost REQUIRED COMPONENTS system)

 find_package(Eigen REQUIRED)

find_package(OpenCV REQUIRED
COMPONENTS
  opencv_highgui
CONFIG)

catkin_package(
  LIBRARIES pc_adder ouster_ros
  CATKIN_DEPENDS cmake_modules cv_bridge pcl_ros roscpp rospy std_msgs
  DEPENDS system_lib Eigen
)


set(CMAKE_CXX_STANDARD 14)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${_ouster_ros_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIRS}
)
include_directories(${OpenCV_INCLUDE_DIRS})

add_executable(${PROJECT_NAME} src/pc_adder.cpp)

target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

install(TARGETS ${PROJECT_NAME}
  pc_adder
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Mark other files for installation (e.g. launch and bag files, etc.)
install(DIRECTORY
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
