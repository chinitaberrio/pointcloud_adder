#include <ros/ros.h>

#include <rosbag/bag.h>
#include <rosbag/view.h>

#include <thread>
#include <functional>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>

#include "ouster/lidar_scan.h"
#include "ouster/types.h"
#include "ouster_ros/OSConfigSrv.h"
#include "ouster_ros/PacketMsg.h"
#include "ouster_ros/ros.h"


// Global variables
rosbag::Bag bag_out;
std::string directory, metadata;
ros::Time vel_time;

using PacketMsg = ouster_ros::PacketMsg;
using Cloud = ouster_ros::Cloud;
using Point = ouster_ros::Point;
namespace sensor = ouster::sensor;

int main(int argc, char **argv)
{
  ros::init(argc, argv, "pc_adder");
  ros::NodeHandle nh;

  std::cout << "parsing \n";
  //Parsing arguments

  for (int i = 1; i < argc; i++) {
    if (i + 1 != argc){
        if (std::strcmp(argv[i], "-bag") == 0) {     //bagname
           directory = argv[i + 1];
        } 
        else if (std::strcmp(argv[i], "-metadata") == 0) {
          metadata = argv[i + 1];
        }
     }
 }
 
 //Ouster info
 
  auto tf_prefix = nh.param("tf_prefix", std::string{});
  if (!tf_prefix.empty() && tf_prefix.back() != '/') tf_prefix.append("/");
  auto sensor_frame = tf_prefix + "os_sensor";
  auto imu_frame = tf_prefix + "os_imu";
  auto lidar_frame = tf_prefix + "os_lidar";

  auto info = sensor::metadata_from_json(metadata);
  uint32_t H = info.format.pixels_per_column;
  uint32_t W = info.format.columns_per_frame;
  auto udp_profile_lidar = info.format.udp_profile_lidar;

  const int n_returns =
      (udp_profile_lidar == sensor::UDPProfileLidar::PROFILE_LIDAR_LEGACY)
          ? 1
          : 2;
  auto pf = sensor::get_format(info);

  auto img_suffix = [](int ind) {
      if (ind == 0) return std::string();
      return std::to_string(ind + 1);  // need second return to return 2
  };

  auto xyz_lut = ouster::make_xyz_lut(info);

  ouster::LidarScan ls{W, H, udp_profile_lidar};
  Cloud cloud{W, H};

  ouster::ScanBatcher batch(W, pf);

  //Read-write bag
  rosbag::Bag bag;
  std::string bagname =  directory ;
  std::string bagname_out =  directory.replace(directory.end()-4,directory.end()-3,"_with_ouster."); //"/media/stephany/Samsung_T5/2022-04-04_chippendale/bag_new.bag" ;
  std::cout << "bag directory " << bagname <<"\n";
  std::cout << "out_bag directory " << bagname_out <<"\n";
  bag.open(bagname, rosbag::bagmode::Read);
  bag_out.open(bagname_out, rosbag::bagmode::Write);

  for(rosbag::MessageInstance const m: rosbag::View(bag)){

    if (m.getTopic() == "/os_node/lidar_packets")
    {
      PacketMsg::ConstPtr pm = m.instantiate<PacketMsg>();
      if (batch(pm->buf.data(), ls)) {
          auto h = std::find_if(
              ls.headers.begin(), ls.headers.end(), [](const auto& h) {
                  return h.timestamp != std::chrono::nanoseconds{0};
              });
          if (h != ls.headers.end()) {
              for (int i = 0; i < n_returns; i++) {
                  scan_to_cloud(xyz_lut, h->timestamp, ls, cloud, i);
                  bag_out.write("/lidar_point_cloud", m.getTime(), ouster_ros::cloud_to_cloud_msg(
                      cloud, h->timestamp, sensor_frame));
              }
          }
      }
    }
    else if(m.getTopic() == "/os_node/imu_packets")
    {
      PacketMsg::ConstPtr p = m.instantiate<PacketMsg>();
      bag_out.write("/imu_lidar", m.getTime(), ouster_ros::packet_to_imu_msg(*p, imu_frame, pf));
    }
    
    bag_out.write(m.getTopic(), m.getTime(), m, m.getConnectionHeader());

  }

 bag_out.close();
 bag.close();
 return 0;
}