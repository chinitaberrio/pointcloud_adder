#define PCL_NO_PRECOMPILE
#include <iostream>
#include <map>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/passthrough.h>
//#include <pcl/surface/concave_hull.h>
#include "pcl_ros/transforms.h"
#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sensor_msgs/PointCloud2.h>
#include <ros/console.h>
#include <boost/shared_ptr.hpp>
#include <algorithm>
#include <vector>
#include <math.h>
#include <string>
#include <ros/package.h>
#include <geometry_msgs/QuaternionStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Transform.h>
#include <geometry_msgs/PolygonStamped.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <sensor_msgs/CameraInfo.h>
#include <gmsl_frame_msg/FrameInfo.h>
#include <ros/ros.h>
#include <tf/tf.h>
#include <tf2/convert.h>
#include <tf_conversions/tf_eigen.h>
#include <nav_msgs/Odometry.h>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <unsupported/Eigen/MatrixFunctions>
#include <velodyne_msgs/VelodynePacket.h>
#include <velodyne_msgs/VelodyneScan.h>
#include <velodyne_pointcloud/rawdata.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <rosbag/player.h>
#include "rosbag/message_instance.h"
using namespace Eigen;
using namespace message_filters;
//Global variables
ros::Publisher pub;
tf::Quaternion current_attitude, base_vel;
Quaterniond eigen_attitude;
Matrix4d prev1, Matrix_bl_bf, Matrix_bl_vl, Matrix_bf_vl;
ros::Time prev_time;
std::string path;
tf::Matrix3x3 Rotation_bl_bf, Rotation_bl_vl, Rotation_bf_vl;
tf::Vector3 current_position, traslation_bl_vl, traslation_bl_bf, traslation_bf_vl;
float angle_past;
float angle_past_xy;
std::vector<std::vector<float> > position;
std::vector<std::vector<float> > obs_points;
static sensor_msgs::PointCloud2 PCacumulada;
static sensor_msgs::PointCloud2 PCacumuladaPreFilter;
static sensor_msgs::PointCloud2 PCObstaculos;
static pcl::PCLPointCloud2 pclPCacumulada;
static pcl::PCLPointCloud2 pclPCacumuladaFiltered;
ros::Publisher publicador;
ros::Publisher publicador_ext;
ros::Publisher publicador_obs;
ros::Publisher publicador_conv;
ros::Publisher pub_pol;
ros::ServiceServer Publish_Boundary;
velodyne_rawdata::RawData data;
int numMessages = 0 ;
float lim_left = 9;
float lim_right = 9;

template <class M>
   class BagSubscriber : public message_filters::SimpleFilter<M>
   {
   public:
     void newMessage(const boost::shared_ptr<M const> &msg)
     {
       this->signalMessage(msg);
     }
   };
   struct SecondColumnOnlyCmp
   {
       bool operator()(const std::vector<float>& lhs,
                       const std::vector<float>& rhs) const
       {
           return (lhs[1] < rhs[1]);
       }
   };
   struct sortlesstomore {
     bool operator() (float i,float j) { return (i<j);}
   };
   int Middle_point(std::vector<std::vector<float> > matrixPC, float value){
     int middle;
     float middle_f = 100000;
     for (int i=0; i< (matrixPC.size()-1); i=i+1){
       if ((std::abs(matrixPC[i][1]-value))<middle_f){
         middle = i;
         middle_f = std::abs(matrixPC[i][1]-value);
       }
     }
     return (middle);
   }
  float PathProjection(double YawR, double YawL, float RingDistance)
   {
     float H, W, R, C, D, d, dist, value;
     H = 1.775;//1.775; // distancia entre ejes
     W = 1.290; // distancia entre las llantas
     C = 0.168; // distancia projectada del eje del velodyne
     dist = C + RingDistance;
     if (std::abs(YawR)<0.05 && std::abs(YawL)<0.05){ //va casi derecho
       value = 0;
     } else if (std::abs(YawR) > std::abs(YawL)){ // voltea a la derecha
       R = -((H/tan(YawR)) + (W/2));
       D = sqrt(R*R +1.061);
       d = 0.25*sqrt((D+R+dist)*(D+R-dist)*(D-R+dist)*(-D+R+dist));
       value = 1.03/2 + 1.03*(R*R-pow(dist,2))/(2*pow(D,2))-R*d/(pow(D,2))+0.3; // -(1.03/2 - 1.03*(R*R-pow(dist,2))/(2*pow(D,2))-R*d/(pow(D,2)))-1.2*W;
     //  cout << "Voltea derecha .\n";
     } else if (std::abs(YawL) > std::abs(YawR)){ // voltea a la izquierda
       R = (H/tan(YawL)) + (W/2);
       D = sqrt(R*R +1.061);
       d = 0.25*sqrt((D+R+dist)*(D+R-dist)*(D-R+dist)*(-D+R+dist));
       value = -1.03/2 - 1.03*(R*R-pow(dist,2))/(2*pow(D,2))+R*d/(pow(D,2))-0.3; //-(1.03/2 - 1.03*(R*R-pow(dist,2))/(2*pow(D,2))-R*d/(pow(D,2)))-W;
     //  cout << "Voltea Izquierda.\n";
       //cout << "YawL " << YawL << ".\n";
       //cout << "YawR " << YawR << ".\n";
     } else {
       value = 0;
     }
    // cout << "Value " << value << ".\n";
     return (value);
   }

  std::vector<std::vector<float> > Extract_edges(std::vector<std::vector<float> > matrixPC, float AngleThreshold, float Angle_d_Threshold, float IntensityThreshold, float Intensity, int middle_intensity_index, int points) {
     std::vector<std::vector<float> > edges_points;
     int n=20;
     int inclination_change=0;
     float angle;
     float anglexy;
     //Busqueda al lado Derecho
     bool obs_N_det= true;
     angle_past_xy = atan2((matrixPC[middle_intensity_index+points][0]-matrixPC[middle_intensity_index][0]),((matrixPC[middle_intensity_index+points][1]-matrixPC[middle_intensity_index][1])))*180/3.14159265;
     angle_past = atan2((matrixPC[middle_intensity_index+points][2]-matrixPC[middle_intensity_index][2]),((matrixPC[middle_intensity_index+points][1]-matrixPC[middle_intensity_index][1])))*180/3.14159265;
     for (int i=middle_intensity_index; i< (matrixPC.size()-points); i=i+1)
     {
        std::vector<float> edge_h;
        anglexy=atan2((matrixPC[i+points][0]-matrixPC[i][0]),((matrixPC[i+points][1]-matrixPC[i][1])))*180/3.14159265;
        angle=atan2((matrixPC[i+points][2]-matrixPC[i][2]),((matrixPC[i+points][1]-matrixPC[i][1])))*180/3.14159265;
       // cout << "Angle D " << angle << ".\n";
       //comparing angle
        if ((std::abs(angle))<AngleThreshold && (std::abs(angle-angle_past))<Angle_d_Threshold && obs_N_det ){
              edge_h.push_back(matrixPC[i][0]);edge_h.push_back(matrixPC[i][1]);edge_h.push_back(matrixPC[i][2]);edge_h.push_back(matrixPC[i][3]);
              edges_points.push_back(edge_h);
              angle_past = angle;
              angle_past_xy =anglexy;
         } else {
          obs_N_det= false;
         // cout << "Derecha .\n";
         // cout << "Angle I " << angle << ".\n";
        // cout << "Angle Diff " << (angle-angle_past) << ".\n";
         //break;
          edge_h.push_back(matrixPC[i][0]);edge_h.push_back(matrixPC[i][1]);edge_h.push_back(matrixPC[i][2]);edge_h.push_back(matrixPC[i][3]);
          obs_points.push_back(edge_h);
       }
     }
     obs_N_det= true;
     angle_past = atan2((matrixPC[middle_intensity_index][2]-matrixPC[middle_intensity_index-points][2]),((matrixPC[middle_intensity_index][1]-matrixPC[middle_intensity_index-points][1])))*180/3.14159265;
     for (int i=middle_intensity_index; i> points; i=i-1)
     {
        std::vector<float> edge_h;
        angle=atan2((matrixPC[i][2]-matrixPC[i-points][2]),((matrixPC[i][1]-matrixPC[i-points][1])))*180/3.14159265;
       //comparing angle
      //  cout << "Angle I " << angle << ".\n";
        if ((std::abs(angle))<AngleThreshold && (std::abs(angle-angle_past))<Angle_d_Threshold && obs_N_det ){
              edge_h.push_back(matrixPC[i][0]);edge_h.push_back(matrixPC[i][1]);edge_h.push_back(matrixPC[i][2]);edge_h.push_back(matrixPC[i][3]);
              edges_points.push_back(edge_h);
              angle_past=angle;
         } else {
             obs_N_det= false;
           //  cout << "Izquierda .\n";
           //  cout << "Angle I " << angle << ".\n";
           //  cout << "Angle Diff " << (angle-angle_past) << ".\n";
             edge_h.push_back(matrixPC[i][0]);edge_h.push_back(matrixPC[i][1]);edge_h.push_back(matrixPC[i][2]);edge_h.push_back(matrixPC[i][3]);
             obs_points.push_back(edge_h);
        // break;
       }
     }
    // cout << "Fin .\n";
     return edges_points;
   }
   pcl::PointCloud<pcl::PointXYZI> Mat_2_PCL(std::vector<std::vector<float> > matrixPC){
     pcl::PointCloud<pcl::PointXYZI> Edges;
     for (int i=0;i < matrixPC.size(); i=i+1){
       pcl::PointXYZI point;
       point.x = matrixPC[i][0];
       point.y = matrixPC[i][1];
       point.z = matrixPC[i][2];
       point.intensity = matrixPC[i][3];
       Edges.push_back(point);
     }
     return Edges;
   }
   std::vector<std::vector<float> > median (std::vector<std::vector<float> > matrixPC, int coord, int window){
     //Filtro de media// filtrar la coordenada 0
     std::vector<float> vector (window,matrixPC[0][coord]);
     int med = (window-1)/2;
     int l=0;
     for (int i=0;i < matrixPC.size(); i=i+1){
       l=0;
       int minimo1=std::min(vector.size(),matrixPC.size()-i);
       for (int j=med; j<minimo1;j++){
         vector[j]=matrixPC[i+l][coord];
         l++;
       }
       sort(vector.begin(),vector.end(),sortlesstomore());
       matrixPC[i][coord]=vector[med];
       int minimo= std::min((i+1),med);
       l=0;
       for (int j=med-1; j>minimo;j--){
         vector[j]=matrixPC[i-l][coord];
         l++;
       }
     }
     return matrixPC;
   }
void callback(const velodyne_msgs::VelodyneScan::ConstPtr &v_scan,  const nav_msgs::Odometry::ConstPtr &odo)
{
    // Mensajes de nubes de puntos
    pcl::PointCloud<velodyne_pointcloud::PointXYZIR>::Ptr cloudall(new  pcl::PointCloud<velodyne_pointcloud::PointXYZIR>());
    sensor_msgs::PointCloud2 object_viz;
    //tiempo entre la anterior lectura de posicion y la actual
    ros::Duration dura;
    dura=odo->header.stamp-prev_time;  // diferencia de tiempos, aprox 1/10 ya que el sensor mas lento es el lidar
    prev_time=odo->header.stamp;
    tf::quaternionMsgToTF(odo->pose.pose.orientation, current_attitude);  // quaternion de orientacion actual
    tf::Matrix3x3 rot1(current_attitude);
    Matrix4d cur1, T1;
    cur1 << rot1.getRow(0)[0],rot1.getRow(0)[1] ,rot1.getRow(0)[2],current_position.getX(),
            rot1.getRow(1)[0],rot1.getRow(1)[1] ,rot1.getRow(1)[2],current_position.getY(),
            rot1.getRow(2)[0],rot1.getRow(2)[1] ,rot1.getRow(2)[2],current_position.getZ(),
            0,0,0,1; // matriz de transformacion homegenea del odom frame a la posicion actual

    T1=prev1.pow(-1)*cur1; //matriz de transformacion homegenea del estado actual al anterior
    prev1=cur1;
    //convert the /velodyne_packets to /velodyne_points

    /*data.setParameters(0.4,130.0,0.0,6.283);
    std::stringstream ss;
    ss << ros::package::getPath("velodyne_pointcloud") << "/params/VLP16db.yaml";
    std::string s = ss.str();
    data.setupOffline( s,130.0,0.4);*/
    for(int i=0;i<v_scan->packets.size();i++)
    {
        pcl::PointCloud<velodyne_pointcloud::PointXYZIR> v_point_cloud, v_transform;
        data.unpack(v_scan->packets[i],v_point_cloud); //points in the packet
        //extrinsic transformation
        ros::Duration offst;
        offst=v_scan->header.stamp-v_scan->packets[i].stamp;
        Matrix4d Tfd2;
        Tfd2=(T1.pow((-offst.toSec()/dura.toSec())))*Matrix_bf_vl; //portion of the transformation corresponding to the packet
        Eigen::Affine3d transform_A2 = Eigen::Affine3d::Identity();
        transform_A2.matrix() = Tfd2;
        pcl::transformPointCloud (v_point_cloud,  v_transform ,transform_A2);  // realiza la transformacion
        *cloudall=(*cloudall)+(v_transform); //acumula la nube de puntos
    }
    //Filtros respecto a el eje X
    pcl::PointCloud<velodyne_pointcloud::PointXYZIR>::Ptr cloud_filtered(new  pcl::PointCloud<velodyne_pointcloud::PointXYZIR>());
    pcl::PointCloud<velodyne_pointcloud::PointXYZIR>::Ptr cloud_filtered1(new  pcl::PointCloud<velodyne_pointcloud::PointXYZIR>());
    pcl::PassThrough<velodyne_pointcloud::PointXYZIR> pass (false);
    pass.setInputCloud(cloudall);
    pass.setFilterFieldName ("x");
    pass.setFilterLimits(2, 100);
    pass.setFilterLimitsNegative(false);
    pass.filter (*cloud_filtered);
    pcl::StatisticalOutlierRemoval<velodyne_pointcloud::PointXYZIR> sor;
    sor.setInputCloud (cloud_filtered);
    sor.setMeanK (10);
    sor.setStddevMulThresh (.1);
    sor.filter (*cloud_filtered1);
    //Nube de puntos en vectores
    std::vector<std::vector<float> >  Ring1;
    std::vector<std::vector<float> >  Ring2;
    std::vector<std::vector<float> >  Ring3;
    std::vector<std::vector<float> >  Ring4;
    std::vector<std::vector<float> >  Ring5;
    std::vector<std::vector<float> >  Ring6;
    std::vector<std::vector<float> >  Ring7;
    std::vector<std::vector<float> >  Ring8;
   //Seleccion de los anillos de la nube de puntos
   for (pcl::PointCloud<velodyne_pointcloud::PointXYZIR>::const_iterator it = cloud_filtered1->begin(); it != cloud_filtered1->end(); it++){
      std::vector<float> row1;
      row1.push_back(it->x); row1.push_back(it->y); row1.push_back(it->z); row1.push_back(it->intensity);
      if(it->ring == 0){
          Ring1.push_back(row1);
      }   else if(it->ring == 1){
          Ring2.push_back(row1);
      }   else if(it->ring == 2){
          Ring3.push_back(row1);
      }   else if(it->ring == 3){
          Ring4.push_back(row1);
      }   else if(it->ring == 4){
          Ring5.push_back(row1);
      }   else if(it->ring == 5){
          Ring6.push_back(row1);
      }   else if(it->ring == 6){
          Ring7.push_back(row1);
      }   else if(it->ring == 7){
          Ring8.push_back(row1);
      }
    }

  // cout << "anillos .\n";
   //Ordenar los puntos segun su posicion en el eje Y
    std::sort(Ring1.begin(), Ring1.end(), SecondColumnOnlyCmp());
    std::sort(Ring2.begin(), Ring2.end(), SecondColumnOnlyCmp());
    std::sort(Ring3.begin(), Ring3.end(), SecondColumnOnlyCmp());
    std::sort(Ring4.begin(), Ring4.end(), SecondColumnOnlyCmp());
    std::sort(Ring5.begin(), Ring5.end(), SecondColumnOnlyCmp());
    std::sort(Ring6.begin(), Ring6.end(), SecondColumnOnlyCmp());
    Ring1 = median(Ring1,0,7);
   // Ring1 = median(Ring1,2,7);
    Ring2 = median(Ring2,0,7);
   // Ring2 = median(Ring2,2,7);
    Ring3 = median(Ring3,0,7);
   // Ring3 = median(Ring3,2,7);
    Ring4 = median(Ring4,0,5);
   // Ring4 = median(Ring4,2,5);
    Ring5 = median(Ring5,0,5);
    //Ring5 = median(Ring5,2,5);
    // Proyeccion del camino
    float pathRing1 = 0; // PathProjection(yawR, yawL, 3.39);
    float pathRing2 =  0; //PathProjection(yawR, yawL, 3.76);
    float pathRing3 =  0; //PathProjection(yawR, yawL, 4.22);
    //Obtengo el punto medio, donde Y = 0 y presuntamente esta frente al carro
    int middleR1 = Middle_point(Ring1,pathRing1); //indice
    int middleR2 = Middle_point(Ring2,pathRing2);
    int middleR3 = Middle_point(Ring3,pathRing3);
    int middleR4 = Middle_point(Ring4,pathRing3);
    int middleR5 = Middle_point(Ring5,pathRing3);
    int middleR6 = Middle_point(Ring6,pathRing3);
    //ensayo
   // cout << "Middle " << middleR1 << ".\n";
   // cout << "value " << Ring1[middleR1][1] << ".\n";
    pcl::PointCloud<pcl::PointXYZI> Ensayo;
    pcl::PointXYZI point;
    point.x = Ring1[middleR1][0];
    point.y = Ring1[middleR1][1];
    point.z = Ring1[middleR1][2];
    point.intensity = Ring1[middleR1][3];
    Ensayo.push_back(point);
    pcl::PointXYZI point2;
    point2.x = Ring2[middleR2][0];
    point2.y = Ring2[middleR2][1];
    point2.z = Ring2[middleR2][2];
    point2.intensity = Ring2[middleR2][3];
    Ensayo.push_back(point2);
    pcl::PointXYZI point3;
    point3.x = Ring3[middleR3][0];
    point3.y = Ring3[middleR3][1];
    point3.z = Ring3[middleR3][2];
    point3.intensity = Ring3[middleR3][3];
    Ensayo.push_back(point3);
    pcl::PointXYZI point4;
    point4.x = Ring4[middleR4][0];
    point4.y = Ring4[middleR4][1];
    point4.z = Ring4[middleR4][2];
    point4.intensity = Ring4[middleR4][3];
    Ensayo.push_back(point4);
    pcl::PointXYZI point5;
    point5.x = Ring5[middleR5][0];
    point5.y = Ring5[middleR5][1];
    point5.z = Ring5[middleR5][2];
    point5.intensity = Ring5[middleR5][3];
    Ensayo.push_back(point5);
    sensor_msgs::PointCloud2 EnsayoPC;
    pcl::PCLPointCloud2 EnsayoPCL;
    pcl::toPCLPointCloud2(Ensayo,EnsayoPCL);
    pcl_conversions::fromPCL(EnsayoPCL, EnsayoPC);// Pasa a mensaje
    EnsayoPC.header = v_scan->header;
    EnsayoPC.header.frame_id = "/base_footprint";
    //ensayo
    //Extraer los puntos contenidos dentro los bordes, con un thresh hold de angulo de 3 y un thresh hold de intensidad de 9, tambien se provee el punto medio
    std::vector<std::vector<float> > edge1 = Extract_edges(Ring1, 15, 10, 9, Ring1[middleR1][3], middleR1, 15);// 12 10
    std::vector<std::vector<float> > edge2 = Extract_edges(Ring2, 15, 10, 9, Ring2[middleR2][3], middleR2, 14);//10 6
    std::vector<std::vector<float> > edge3 = Extract_edges(Ring3, 15, 10, 9, Ring3[middleR3][3], middleR3, 10);//10 6
    std::vector<std::vector<float> > edge4 = Extract_edges(Ring4, 15, 10, 9, Ring4[middleR4][3], middleR4, 8);//9 7
    std::vector<std::vector<float> > edge5 = Extract_edges(Ring5, 15, 10, 9, Ring5[middleR5][3], middleR5, 8);//8 7
    std::vector<std::vector<float> > edge6 = Extract_edges(Ring6, 15, 10, 9, Ring6[middleR6][3], middleR6, 8);//8 7
    //conversion de los puntos de vectores a pointXYZI, pointcloud2 y
    pcl::PointCloud<pcl::PointXYZI> edge_pc1 = Mat_2_PCL(edge1);//edge1
    pcl::PointCloud<pcl::PointXYZI> edge_pc2 = Mat_2_PCL(edge2);
    pcl::PointCloud<pcl::PointXYZI> edge_pc3 = Mat_2_PCL(edge3);
    pcl::PointCloud<pcl::PointXYZI> edge_pc4 = Mat_2_PCL(edge4);
    pcl::PointCloud<pcl::PointXYZI> edge_pc5 = Mat_2_PCL(edge5);
    pcl::PointCloud<pcl::PointXYZI> edge_pc6 = Mat_2_PCL(edge6);
    pcl::PointCloud<pcl::PointXYZI> obs = Mat_2_PCL(obs_points);
    pcl::PCLPointCloud2 cloudR1;
    pcl::PCLPointCloud2 cloudR2;
    pcl::PCLPointCloud2 cloudR3;
    pcl::PCLPointCloud2 cloudR4;
    pcl::PCLPointCloud2 cloudR5;
    pcl::PCLPointCloud2 cloudR6;
    pcl::PCLPointCloud2 cloudFinal;
    pcl::PCLPointCloud2 obs_pc;
    pcl::toPCLPointCloud2(edge_pc1,cloudR1);
    pcl::toPCLPointCloud2(edge_pc2,cloudR2);
    pcl::toPCLPointCloud2(edge_pc3,cloudR3);
    pcl::toPCLPointCloud2(edge_pc4,cloudR4);
    pcl::toPCLPointCloud2(edge_pc5,cloudR5);
    pcl::toPCLPointCloud2(edge_pc6,cloudR6);
    pcl::toPCLPointCloud2(obs,obs_pc);
    float height2 = 0.37;
    float height3 = 0.42;
    float height4 = 0.47;
    float height5 = 0.54;
    float height6 = 0.64;
    if (Ring2[middleR2][2]<height2){
      pcl::concatenatePointCloud (cloudR2, cloudR1, cloudFinal);
      if (Ring3[middleR3][2]<height3){
        pcl::concatenatePointCloud (cloudFinal, cloudR3, cloudFinal);
        if (Ring4[middleR4][2]<height4){
          pcl::concatenatePointCloud (cloudFinal, cloudR4, cloudFinal);
          if (Ring5[middleR5][2]<height5){
            pcl::concatenatePointCloud (cloudFinal, cloudR5, cloudFinal);
            if (Ring6[middleR6][2]<height6){
              pcl::concatenatePointCloud (cloudFinal, cloudR6, cloudFinal);
            }
          }
        }
      }
    }
    //cout << "concatenado .\n";
    //Si se acumula
   /* if (acumular)
    {
      static sensor_msgs::PointCloud2 PrePCacumulada;
      pcl_conversions::fromPCL(cloudFinal, PrePCacumulada); //Puntos sin acumular en mensaje
      PrePCacumulada.header.frame_id = "/base_footprint";//Recordar que estan en base_footprint frame
      sensor_msgs::PointCloud2 PCmsg_trans;
      pcl_ros::transformPointCloud("/map",transform1,PrePCacumulada,PCmsg_trans);// Transformar la nube de puntos al odom frame
      pcl::PCLPointCloud2 curr_pcl_trans;
      pcl_conversions::toPCL(PCmsg_trans,curr_pcl_trans); //Point cloud transformada en PCL library
      pcl::concatenatePointCloud (pclPCacumulada, curr_pcl_trans, pclPCacumulada); //Concatenacion de los puntos con una variable anterior
      pcl_conversions::fromPCL(pclPCacumulada, PCacumulada); //Convercion a mensaje
      PCacumulada.header = PCmsg.header; //misma cabezera que el mensaje original
      PCacumulada.header.frame_id = "/map"; //con odom frame
    } else {*/
      pcl_conversions::fromPCL(cloudFinal, PCacumulada);// Pasa a mensaje
      PCacumulada.header = v_scan->header;
      PCacumulada.header.frame_id = "/base_footprint";

      pcl_conversions::fromPCL(obs_pc, PCObstaculos);// Pasa a mensaje
      PCObstaculos.header = v_scan->header;
      PCObstaculos.header.frame_id = "/base_footprint";

      obs_points.clear();
    //}
    //

      pcl::toROSMsg (*cloudall, object_viz);
      object_viz.header.frame_id="base_footprint";
      object_viz.header.stamp= v_scan->header.stamp;
      pub.publish (object_viz);

      publicador_obs.publish(PCObstaculos);
      publicador.publish(PCacumulada);

   // pcl::PointCloud<pcl::PointXYZ>::Ptr HullConcave(new pcl::PointCloud<pcl::PointXYZ>);
   // pcl::fromROSMsg (PCacumulada, *HullConcave);
 /*   pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_hull (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::ConcaveHull<pcl::PointXYZ> chull;
    chull.setInputCloud (HullConcave);
    chull.setAlpha (1.5); //change here
    chull.reconstruct (*cloud_hull);
    std::cerr << "Concave hull has: " << cloud_hull->points.size ()
                << " data points." << std::endl;
    geometry_msgs::PolygonStamped Poligono;
    for (pcl::PointCloud<pcl::PointXYZ>::const_iterator it = cloud_hull->begin(); it > cloud_hull->end(); it++){
       geometry_msgs::Point32 tl;
       tl.x=it->x;
       tl.y=it->y;
       tl.z=it->z;
       Poligono.polygon.points.push_back(tl);
     }
    Poligono.header.frame_id = "/base_footprint";
    pub_pol.publish(Poligono);
    std::cerr << "Poligon has: " << Poligono.polygon.points.size()
                << " data points." << std::endl;*/
    //
   // static sensor_msgs::PointCloud2 c_hull;
   // pcl::toROSMsg(*cloud_hull, c_hull);
   // publicador_conv.publish(c_hull);
    //Publicacion
   // publicador_ext.publish(EnsayoPC);
}
int main (int argc, char** argv)
{
    ros::init(argc, argv, "road_motion_c");
    ros::NodeHandle nh;
    prev_time=ros::Time::now();
    prev1 = Matrix4d::Identity();
    Matrix_bl_bf = Matrix4d::Identity();
    Matrix_bl_vl = Matrix4d::Identity();
    Matrix_bf_vl = Matrix4d::Identity();
    pub = nh.advertise<sensor_msgs::PointCloud2> ("/velodyne_corrected", 10);
    publicador= nh.advertise<sensor_msgs::PointCloud2>("/velodyne_points_road",10);
    publicador_ext= nh.advertise<sensor_msgs::PointCloud2>("/velodyne_points_path",10);
    publicador_obs= nh.advertise<sensor_msgs::PointCloud2>("/velodyne_points_obs",10);
    publicador_conv= nh.advertise<sensor_msgs::PointCloud2>("/velodyne_points_conv",10);

    ros::Publisher camera_info_p = nh.advertise<sensor_msgs::CameraInfo>("/gmsl/A0/camera_info", 100);
    ros::Publisher frame_info_p = nh.advertise<gmsl_frame_msg::FrameInfo>("/gmsl/A0/frame_info", 100);
    ros::Publisher odometry_p = nh.advertise<nav_msgs::Odometry>("/vn100/odometry", 100);
    ros::Publisher tf = nh.advertise<tf2_msgs::TFMessage>("/tf_static", 100);

    data.setParameters(0.4,130.0,0.0,6.283);
    std::stringstream ss;
    ss << ros::package::getPath("velodyne_pointcloud") << "/params/VLP16db.yaml";
    std::string s = ss.str();
    data.setupOffline( s,130.0,0.4);

    while (nh.ok())
    {
     rosbag::Bag bag;  // rosbag directory
     bag.open(argv[1], rosbag::bagmode::Read);
     std::string odo               =  "/vn100/odometry";
     std::string velodyne          =  "/velodyne/front/velodyne_packets";
     std::string tf_msg_static     =  "/tf_static";
     std::string camera_info       =  "/gmsl/A0/camera_info";
     std::string frame_info        =  "/gmsl/A0/frame_info";
     // Topics to load
     std::vector<std::string> topics;
     topics.push_back(odo);
     topics.push_back(velodyne);
     topics.push_back(tf_msg_static);
     topics.push_back(camera_info);
     topics.push_back(frame_info);
     rosbag::View view(bag, rosbag::TopicQuery(topics));
     // Set up fake subscribers to capture frameinfo and velodyne pc
     BagSubscriber<velodyne_msgs::VelodyneScan> velodyne_sub;
     BagSubscriber<nav_msgs::Odometry> odo_sub;
     // Use ApproximateTime synchronizer to make sure we get properly synchronized images / pointcloud
     typedef message_filters::sync_policies::ApproximateTime<velodyne_msgs::VelodyneScan,nav_msgs::Odometry> MySyncPolicy;
     message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(30), velodyne_sub, odo_sub);
     sync.registerCallback(boost::bind(&callback, _1, _2));
     std::map<std::string, ros::Publisher> publishers_;
     // Load all messages into our stereo dataset
     BOOST_FOREACH(rosbag::MessageInstance const m, view)
     {
       if (m.getTopic() == velodyne || ("/" + m.getTopic() == velodyne))
       {
         velodyne_msgs::VelodyneScan::ConstPtr vel = m.instantiate<velodyne_msgs::VelodyneScan>();
         if (vel != NULL)
           velodyne_sub.newMessage(vel);
       }
      if (m.getTopic() == odo || ("/" + m.getTopic() == odo ))
      {
         nav_msgs::Odometry::ConstPtr od = m.instantiate<nav_msgs::Odometry>();
        if (od != NULL)
        {
          odo_sub.newMessage(od);
          odometry_p.publish(od);
        }
      }

      if (m.getTopic() == tf_msg_static || ("/" + m.getTopic() == tf_msg_static))
      {
        tf2_msgs::TFMessage::ConstPtr tf_info = m.instantiate<tf2_msgs::TFMessage>();
        if (tf_info != NULL)
        {
          for(int i=0; i < tf_info->transforms.size();i++){
            if (tf_info->transforms[i].header.frame_id == "base_link"){
              if (tf_info->transforms[i].child_frame_id == "base_footprint"){
                Rotation_bl_bf.setRotation(tf::Quaternion(tf_info->transforms[i].transform.rotation.x, tf_info->transforms[i].transform.rotation.y, tf_info->transforms[i].transform.rotation.z,tf_info->transforms[i].transform.rotation.w));
                traslation_bl_bf = tf::Vector3(tf_info->transforms[i].transform.translation.x,tf_info->transforms[i].transform.translation.y,tf_info->transforms[i].transform.translation.z);
                Matrix_bl_bf << Rotation_bl_bf.getRow(0)[0],Rotation_bl_bf.getRow(0)[1] ,Rotation_bl_bf.getRow(0)[2],traslation_bl_bf.getX(),
                                Rotation_bl_bf.getRow(1)[0],Rotation_bl_bf.getRow(1)[1] ,Rotation_bl_bf.getRow(1)[2],traslation_bl_bf.getY(),
                                Rotation_bl_bf.getRow(2)[0],Rotation_bl_bf.getRow(2)[1] ,Rotation_bl_bf.getRow(2)[2],traslation_bl_bf.getZ(),
                                0,0,0,1;
              }
              if (tf_info->transforms[i].child_frame_id == "velodyne_front_link"){
                Rotation_bl_vl.setRotation(tf::Quaternion(tf_info->transforms[i].transform.rotation.x, tf_info->transforms[i].transform.rotation.y, tf_info->transforms[i].transform.rotation.z,tf_info->transforms[i].transform.rotation.w));
                traslation_bl_vl = tf::Vector3(tf_info->transforms[i].transform.translation.x,tf_info->transforms[i].transform.translation.y,tf_info->transforms[i].transform.translation.z);
                Matrix_bl_vl << Rotation_bl_vl.getRow(0)[0],Rotation_bl_vl.getRow(0)[1] ,Rotation_bl_vl.getRow(0)[2],traslation_bl_vl.getX(),
                                Rotation_bl_vl.getRow(1)[0],Rotation_bl_vl.getRow(1)[1] ,Rotation_bl_vl.getRow(1)[2],traslation_bl_vl.getY(),
                                Rotation_bl_vl.getRow(2)[0],Rotation_bl_vl.getRow(2)[1] ,Rotation_bl_vl.getRow(2)[2],traslation_bl_vl.getZ(),
                                0,0,0,1;
              }
              Matrix_bf_vl = Matrix_bl_bf.pow(-1) * Matrix_bl_vl;
              }
            }
          tf.publish(tf_info);
          }        
        }

      if (m.getTopic() == camera_info || ("/" + m.getTopic() == camera_info ))
      {
        sensor_msgs::CameraInfo::ConstPtr c_i = m.instantiate<sensor_msgs::CameraInfo>();
        if (c_i != NULL)
          camera_info_p.publish(c_i);
      }
      if (m.getTopic() == frame_info || ("/" + m.getTopic() == frame_info ))
      {
        gmsl_frame_msg::FrameInfo::ConstPtr f_i = m.instantiate<gmsl_frame_msg::FrameInfo>();
        if (f_i != NULL)
          frame_info_p.publish(f_i);
      }


      }

    bag.close();

    return (0);
   }
}
