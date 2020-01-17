#ifndef _VISUALIZER_H_
#define _VISUALIZER_H_

#include <ros/ros.h>
#include <pcl/point_cloud.h>
#include <pcl_ros/point_cloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <vector>

class visualizer
{
  public:
    void vis_split_pc(ros::NodeHandle n, ros::Publisher split_pc_pub,
                    std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr > split_pc);
    
	void vis_split_pc(ros::NodeHandle n, ros::Publisher split_pc_pub,
                    std::vector<pcl::PointCloud<pcl::PointXYZINormal>::Ptr > split_pc);
    
	void vis_RF(ros::NodeHandle n, ros::Publisher RF_pub,
              std::vector<Eigen::Vector3d> eigenvector);

  private:
    const std::string frame_id_split_pc = "/velodyne";
    const std::string frame_id_RF = "/velodyne";
};

#endif
