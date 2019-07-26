#include <ros/ros.h>
#include <pcl/point_cloud.h>
#include <pcl_ros/point_cloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <visualization_msgs/Marker.h>
#include <descriptor_test.h>
#include <visualizer.h>
#include <fstream>
#include <iostream>


pcl::PointCloud<pcl::PointXYZI>::Ptr input_pc (new pcl::PointCloud<pcl::PointXYZI>);

void pc_callback(sensor_msgs::PointCloud2 input)
{
  pcl::fromROSMsg(input, *input_pc);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "intensity_test");
  ros::NodeHandle n;
  ros::Rate loop_rate(10);

  ros::Subscriber pc_sub = n.subscribe("/velodyne_points", 1000, pc_callback);
  ros::Publisher split_pc_pub = n.advertise<sensor_msgs::PointCloud2>("split_pc", 1000);
  ros::Publisher RF_pub = n.advertise<visualization_msgs::Marker>("RF", 1000);

  descriptor_test desc;
  visualizer visu;

  std::ofstream log;
  log.open("/home/amsl/log.csv",std::ios::trunc);
  if(!log.is_open()){
    std::cout << "NOT OPEN FILE" << std::endl;
    exit(1);
  }

  while(ros::ok()){
    double Sx_p = 0;
    double Sx_m = 0;
    double Sz_p = 0;
    double Sz_m = 0;
    double Sy_p = 0;
    double Sy_m = 0;

    if(input_pc -> points.size() > 0){
      desc.itst_descriptor(input_pc, Sx_p, Sx_m, Sy_p, Sy_m, Sz_p, Sz_m);
      visu.vis_split_pc(n, split_pc_pub, desc.split_pc);
      visu.vis_RF(n, RF_pub, desc.eigenvector);
      //log << abs(Sx_p - Sx_m) << "," << abs(Sy_p - Sy_m) << "," << abs(Sz_p - Sz_m) << std::endl;
      log << Sx_p << "," << Sx_m << "," << Sy_p << "," << Sy_m << "," << Sz_p << "," << Sz_m << std::endl;
    }
    ros::spinOnce();
    loop_rate.sleep();
  }
  log.close();
  return(0);
}
