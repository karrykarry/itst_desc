//intensity_match.cpp
//
//

#include <ros/ros.h>
#include <pcl/point_cloud.h>
#include <pcl_ros/point_cloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Empty.h>
#include <visualization_msgs/Marker.h>
#include <descriptor.h>
#include <visualizer.h>
#include <fileope.h>
#include <localize.h>
#include <histogram_operation.h>


pcl::PointCloud<pcl::PointXYZI>::Ptr input_pc (new pcl::PointCloud<pcl::PointXYZI>);
nav_msgs::Odometry odometry;
sensor_msgs::PointCloud2 buffer_pc;

bool start_flag = true;
bool odometry_flag = false;
// bool odometry_flag = true;
double ref_distance = 0;

using namespace std;

void pc_callback(const sensor_msgs::PointCloud2ConstPtr& input)
{
  pcl::fromROSMsg(*input, *input_pc);

  buffer_pc = *input;
}

void odom_callback(nav_msgs::Odometry input)
{
  odometry = input;
  odometry_flag = true;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "intensity_match");
  ros::NodeHandle n;
  ros::NodeHandle priv_nh("~");
  ros::Rate loop_rate(10);

  ros::Subscriber pc_sub = n.subscribe("/velodyne_points", 1000, pc_callback);
  ros::Subscriber odom_sub = n.subscribe("/lcl_imu", 100, odom_callback);
  
  ros::Publisher split_pc_pub = n.advertise<sensor_msgs::PointCloud2>("split_pc", 1000);
  ros::Publisher RF_pub = n.advertise<visualization_msgs::Marker>("RF", 10);
  
  ros::Publisher PR_pub = n.advertise<std_msgs::Empty>("/pr/saveflag", 10);
  ros::Publisher PR_pc_pub = n.advertise<sensor_msgs::PointCloud2>("/pr/pc", 10);

  descriptor desc;
  visualizer visu;
  file_operation file_ope(n,priv_nh);
  localize local;
  std::vector<std::vector<int> > histogram;
  std_msgs::Empty PR_flag;
  ros::Time ros_begin;

  while(ros::ok()){
    if(input_pc -> points.size() > 0){
      if(start_flag){
        // ros_begin  = ros::Time::now();
        start_flag = false;
      }
      desc.itst_descriptor(input_pc, histogram);

      if(odometry_flag){
        if(local.split_metre(odometry, file_ope.output_dist)){
          file_ope.output_hist_dist_split(histogram);
			PR_pub.publish(PR_flag); 
			PR_pc_pub.publish(buffer_pc); 
        }
        odometry_flag = false;
      }
      //file_ope.output_hist_time(histogram, ros_begin);
      visu.vis_split_pc(n, split_pc_pub, desc.split_pc_f);
      visu.vis_RF(n, RF_pub, desc.eigenvector);
      input_pc -> points.clear();
    }
    ros::spinOnce();
    loop_rate.sleep();
  }

  return(0);
}
