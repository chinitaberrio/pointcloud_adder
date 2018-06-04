 This project conducts the correction for lidar points when the vehicle is moving relative to the static world frame.
 
 This code publishes the corrected pointcloud relative to the base_link frame and also conducts the correction for lidar to camera projection.
 
Installation**

cd catkin_ws/src 

git clone https://gitlab.acfr.usyd.edu.au/charika/motion_correction.git

cd .. 

cd catkin_make 

Operation **


rosrun motion_correction cor       path_to_save_images  path_to_bag_file_with_lidar_data

 This saved images can be used for visualising the accuracy of the corrected points.
 seq_number.png - the original projection without correction
 seq_number_cor.png- projection with correction
 
 The corrected lidar points correspond to each scan are published as a sensor_msgs::PointCloud2 msg with reference to the base_link
 
 Note:
 For lidar to camera correction algorithm is extremely sensitive to intrinsic parameters of the cameras. Please make sure you choose the acurate intrinsic parameters.