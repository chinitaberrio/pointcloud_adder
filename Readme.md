This rosnode adds the point cloud and imu ros messages to a rosbag that recorded the ouster lidar and imu packets 

e.g.

rosrun pc_adder pc_adder -bag '/media/stephany/Samsung_T5/2022-04-04_chippendale/2022-04-04-10-09-04_chippendale.bag' -metadata '/media/stephany/Samsung_T5/metadata.json' 