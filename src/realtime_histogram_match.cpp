//histogram_match.cpp
//
//

#include "realtime_histogram_match.hpp"

H_match::H_match(ros::NodeHandle n,ros::NodeHandle private_nh_)
{	
	pc_sub = n.subscribe<sensor_msgs::PointCloud2>("/velodyne_points", 1, &H_match::pcCallback, this);
  
	desc = new descriptor();
		
	hist_ope = new histogram_operation();
	hist_ope->read_ref_histogram();

}


void 
H_match::pcCallback(const sensor_msgs::PointCloud2ConstPtr &msg)
{
	pcl::PointCloud<pcl::PointXYZI>::Ptr input_pc (new pcl::PointCloud<pcl::PointXYZI>);
	std::vector<std::vector<int> > histogram;

  	pcl::fromROSMsg(*msg, *input_pc);
   
	desc->itst_descriptor(input_pc, histogram);
	hist_ope->match_histogram_pc(histogram);
}
