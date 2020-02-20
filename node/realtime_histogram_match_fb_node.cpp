//histogram_match.cpp
//
//
//intensity_matchで作られた記述子を総当り戦でどうなるかをやっているプログラム
//

#include <ros/ros.h>
#include <iostream>
#include "realtime_histogram_match_fb.hpp"


int main(int argc, char** argv)
{
	ros::init(argc, argv, "histogram_match");
	ros::NodeHandle n;
	ros::NodeHandle priv_nh("~");
	// ros::Rate loop(10);

	ROS_INFO("\033[1;32m---->\033[0m realtime run Started.");

	H_match h_match(n,priv_nh);	

	ros::spin();

	return(0);
}


