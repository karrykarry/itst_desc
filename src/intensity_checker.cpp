#include <ros/ros.h>
#include <pcl/point_cloud.h>
#include <pcl_ros/point_cloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <fstream>
#include <iostream>

#define N 256

pcl::PointCloud<pcl::PointXYZI>::Ptr input_pc (new pcl::PointCloud<pcl::PointXYZI>);
size_t input_pc_size;

void pc_callback(sensor_msgs::PointCloud2 input)
{
  pcl::fromROSMsg(input, *input_pc);
  input_pc_size = input_pc -> points.size();
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "intensity_checker");
  ros::NodeHandle n;
  ros::Rate loop_rate(5);

  ros::Subscriber pc_sub = n.subscribe("/velodyne_points", 1000, pc_callback);

  std::ofstream log;
  log.open("/home/amsl/histogram/test/intensity_check.csv",std::ios::trunc);
  if(!log.is_open()){
    std::cout << "NOT OPEN FILE" << std::endl;
    exit(1);
  }

  int hist[N] = {0};
  while(ros::ok()){
    //int hist[N] = {0};
    if(input_pc_size != 0){
      for(size_t i = 0; i < input_pc_size; i++){
        hist[(int)input_pc -> points[i].intensity] += 1;
        std::cout << "intensity -> " << input_pc -> points[i].intensity << std::endl;
      }
    }
    
    ros::spinOnce();
    loop_rate.sleep();
  }

  for(int i = 0; i < N; i++){
    log << hist[i] << std::endl;
  }
  log.close();
  return(0);
}
