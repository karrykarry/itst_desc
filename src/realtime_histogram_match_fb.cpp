//realtime_histogram_match.cpp
//
//自分の向きではなく
//点群の主成分から判定するようになった
//20190913 version 
//
//事前情報のfront と back を用いてnodeの番号を調べる
//
//
#include "realtime_histogram_match_fb.hpp"

H_match::H_match(ros::NodeHandle n,ros::NodeHandle private_nh_)
{	
	score_vis_pub = n.advertise<std_msgs::Float64MultiArray>("/score/vis", 10);
	score_best_pub = n.advertise<std_msgs::Int32>("/score/best", 10);
	
	pc_sub = n.subscribe<sensor_msgs::PointCloud2>("/velodyne_points", 1, &H_match::pcCallback, this);

	desc = new descriptor();
		
	hist_ope = new histogram_operation(n, private_nh_);
	// hist_ope->read_ref_histogram();

}


void 
H_match::pcCallback(const sensor_msgs::PointCloud2ConstPtr &msg)
{
	pcl::PointCloud<pcl::PointXYZI>::Ptr input_pc (new pcl::PointCloud<pcl::PointXYZI>);
	std::vector<std::vector<int> > histogram_f;	//front のみで判定
	std::vector<double> results;
	
	std_msgs::Float64MultiArray score;
	std_msgs::Int32 best_score;

  	pcl::fromROSMsg(*msg, *input_pc);
   
	desc->itst_descriptor_one(input_pc, histogram_f);
	
	hist_ope->research_match_pubscore(histogram_f, score, best_score);
	
	
	// score_vis_pub.publish(score);
	score_best_pub.publish(best_score);

}

