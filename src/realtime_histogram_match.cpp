//histogram_match.cpp
//
//
#include "realtime_histogram_match.hpp"

H_match::H_match(ros::NodeHandle n,ros::NodeHandle private_nh_):
	input_txtfile("/home/amsl/Pictures/ros_catkin_ws/scan_context/save_pr/list.txt")
{	
	pr_num_vis_pub = n.advertise<visualization_msgs::MarkerArray>("/pr/num/vis_", 10, true);
	score_vis_pub = n.advertise<std_msgs::Float64MultiArray>("/score/vis", 10);
	score_best_pub = n.advertise<std_msgs::Int32>("/score/best", 10);
	
	pc_sub = n.subscribe<sensor_msgs::PointCloud2>("/velodyne_points", 1, &H_match::pcCallback, this);

	desc = new descriptor();
		
	hist_ope = new histogram_operation(n, private_nh_);
	hist_ope->read_ref_histogram();

}


void 
H_match::pcCallback(const sensor_msgs::PointCloud2ConstPtr &msg)
{
	pcl::PointCloud<pcl::PointXYZI>::Ptr input_pc (new pcl::PointCloud<pcl::PointXYZI>);
	std::vector<std::vector<int> > histogram;
	std_msgs::Float64MultiArray score;
	std_msgs::Int32 best_score;

  	pcl::fromROSMsg(*msg, *input_pc);
   
	desc->itst_descriptor(input_pc, histogram);
	hist_ope->match_histogram_pc(histogram, score, best_score);
	
	score_vis_pub.publish(score);
	score_best_pub.publish(best_score);

}
