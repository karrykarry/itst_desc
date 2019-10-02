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
	score_vis_pub = n.advertise<std_msgs::Float64MultiArray>("/score/vis", 10); //score	を可視化するため
	score_best_pub = n.advertise<std_msgs::Int32>("/score/best", 10);//	No.1
	score_better_pub = n.advertise<std_msgs::Int32MultiArray>("/score/better", 10);//候補も含めたもの
	
	pc_sub = n.subscribe<sensor_msgs::PointCloud2>("/velodyne_points", 1, &H_match::pcCallback, this);
	private_nh_.param("Number_of_candidate", NUM_CANDIDATE, {5});

	desc = new descriptor();
		
	hist_ope = new histogram_operation(n, private_nh_);

}


void 
H_match::pcCallback(const sensor_msgs::PointCloud2ConstPtr &msg)
{
	pcl::PointCloud<pcl::PointXYZI>::Ptr input_pc (new pcl::PointCloud<pcl::PointXYZI>);
	std::vector<std::vector<int> > histogram_f;	//front のみで判定
	std::vector<double> results;

	std::vector<int> better_scores;
	better_scores.resize(NUM_CANDIDATE);
	
	std_msgs::Float64MultiArray score;
	std_msgs::Int32 best_score_num;
	std_msgs::Int32MultiArray better_score_num;

  	pcl::fromROSMsg(*msg, *input_pc);
   
	desc->itst_descriptor_one(input_pc, histogram_f);
	
	// hist_ope->research_match_pubscore(histogram_f, score, best_score);
	
	hist_ope->research_match_pubscore_n(histogram_f, better_scores);
	
	best_score_num.data = better_scores[0];
	
	score_best_pub.publish(best_score_num);
	better_scores.erase(better_scores.begin());

	for(auto better_score : better_scores){
		better_score_num.data.push_back(better_score);
	}
	
	score_better_pub.publish(better_score_num);


	// score_vis_pub.publish(score);

}

