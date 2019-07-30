#ifndef _REALTIME_HISTOGRAM_MATCH_HPP_
#define _REALTIME_HISTOGRAM_MATCH_HPP_

#include <ros/ros.h>
#include <iostream>

#include <sensor_msgs/PointCloud2.h>

#include <pcl/point_cloud.h>
#include <pcl_ros/point_cloud.h>

#include "histogram_operation.h"
#include "descriptor.h"

class H_match{
	private:
		ros::Subscriber pc_sub;

  		histogram_operation* hist_ope;
		descriptor* desc;

	public:
		H_match(ros::NodeHandle n,ros::NodeHandle priv_nh);
		
		void pcCallback(const sensor_msgs::PointCloud2ConstPtr &msg);

};


#endif
