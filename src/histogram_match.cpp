//histogram_match.cpp

#include <ros/ros.h>
#include <pcl/point_cloud.h>
#include <pcl_ros/point_cloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <visualization_msgs/Marker.h>
#include <descriptor.h>
#include <fileope.h>
#include <histogram_operation.h>

pcl::PointCloud<pcl::PointXYZI>::Ptr input_pc (new pcl::PointCloud<pcl::PointXYZI>);
bool start_flag = true;

using namespace std;

void pc_callback(sensor_msgs::PointCloud2 input)
{
  pcl::fromROSMsg(input, *input_pc);
  //size_t input_size = input_pc->points.size();
  //cout << "pc_size" << input_size << endl;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "histogram_match");
  ros::NodeHandle n;
  ros::Rate loop_rate(10);

  ros::Subscriber pc_sub = n.subscribe("/velodyne_points", 1000, pc_callback);

  descriptor desc;
  file_operation file_ope;
  histogram_operation hist_ope;

  std::vector<std::vector<int> > histogram;
  ros::Time ros_begin;

  hist_ope.read_ref_histogram_f();
  hist_ope.read_ref_histogram_b();
  hist_ope.match_histogram_one("b", "b");
/*
  while(ros::ok()){
    if(input_pc -> points.size() > 0){
      if(start_flag){
        ros_begin  = ros::Time::now();
        start_flag = false;
      }
      desc.itst_descriptor(input_pc, histogram);
      file_ope.output_hist(histogram, ros_begin);
    }
    ros::spinOnce();
    loop_rate.sleep();
  }
*/
  return(0);
}
