#ifndef _REALTIME_HISTOGRAM_MATCH_HPP_
#define _REALTIME_HISTOGRAM_MATCH_HPP_

#include <ros/ros.h>
#include <iostream>
#include <string>

#include<sensor_msgs/PointCloud2.h>
#include<std_msgs/Float64MultiArray.h>
#include<visualization_msgs/Marker.h>
#include<visualization_msgs/MarkerArray.h>

#include <pcl/point_cloud.h>
#include <pcl_ros/point_cloud.h>

#include "histogram_operation.h"
#include "descriptor.h"

// using namespace std;

class H_match{
	private:
		ros::Publisher pr_num_vis_pub;
		ros::Publisher score_vis_pub;
		ros::Subscriber pc_sub;

  		histogram_operation* hist_ope;
		descriptor* desc;

		std::string input_txtfile;
		std::ifstream reading_file;


		std::vector<std::string> split(const std::string &str, char sep);
		visualization_msgs::Marker text_vis(const double now_x,const double now_y,const double now_z);
		void pr_trajectory_vis();
	public:
		H_match(ros::NodeHandle n,ros::NodeHandle priv_nh);
		
		void pcCallback(const sensor_msgs::PointCloud2ConstPtr &msg);

};


#endif
