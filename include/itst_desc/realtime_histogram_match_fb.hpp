#ifndef _REALTIME_HISTOGRAM_MATCH_FB_HPP_
#define _REALTIME_HISTOGRAM_MATCH_FB_HPP_

#include <ros/ros.h>
#include <iostream>
#include <string>

#include<sensor_msgs/PointCloud2.h>
#include<std_msgs/Int32.h>
#include<std_msgs/Int32MultiArray.h>
#include<std_msgs/Float64MultiArray.h>
#include<visualization_msgs/Marker.h>
#include<visualization_msgs/MarkerArray.h>

#include <pcl/point_cloud.h>
#include <pcl_ros/point_cloud.h>

#include "histogram_operation.h"
#include "descriptor.h"

class H_match{
	private:
		ros::Publisher score_vis_pub;
		ros::Publisher score_best_pub;
		ros::Publisher score_better_pub;
		ros::Publisher score_pub;

		ros::Subscriber pc_sub;

  		histogram_operation* hist_ope;
		descriptor* desc;

		int NUM_CANDIDATE;
	
		std::ifstream reading_file;

	public:
		H_match(ros::NodeHandle n,ros::NodeHandle priv_nh);
		
		void pcCallback(const sensor_msgs::PointCloud2ConstPtr &msg);

};


#endif
