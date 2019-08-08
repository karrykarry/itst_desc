#ifndef _REALTIME_HISTOGRAM_MATCH_PF_HPP_
#define _REALTIME_HISTOGRAM_MATCH_PF_HPP_

#include <ros/ros.h>
#include <iostream>
#include <string>

#include<sensor_msgs/PointCloud2.h>
#include<std_msgs/Int32.h>
#include<std_msgs/Float64MultiArray.h>
#include<visualization_msgs/Marker.h>
#include<visualization_msgs/MarkerArray.h>
#include<geometry_msgs/Pose.h>
#include<geometry_msgs/PoseStamped.h>


#include <pcl/point_cloud.h>
#include <pcl_ros/point_cloud.h>

#include "histogram_operation.h"
#include "descriptor.h"

using namespace std;


// struct Particle{
// 	double x;
// 	double y;
// 	double weight;
// };


class H_match_pf{
	private:
		ros::Publisher pr_num_vis_pub;
		ros::Publisher score_vis_pub;
		ros::Publisher score_best_pub;
		
		ros::Publisher pf_best_pub;

		ros::Subscriber pc_sub;

  		histogram_operation* hist_ope;
		descriptor* desc;

		std::string input_txtfile;
		std::ifstream reading_file;

		double P_RANGE, P_INTER, P_NUM;
		vector<descriptor::Particle> pf_cloud;	

		std::vector<std::string> split(const std::string &str, char sep);
		visualization_msgs::Marker text_vis(const double now_x,const double now_y,const double now_z);
		void pr_trajectory_vis();
		void init_pf();
		void pub_pf(descriptor::Particle pf);

		struct FINAL_ANS{	
			float score;
			int num;
		};


	public:
		H_match_pf(ros::NodeHandle n,ros::NodeHandle priv_nh);
		
		void pcCallback(const sensor_msgs::PointCloud2ConstPtr &msg);

};


#endif

