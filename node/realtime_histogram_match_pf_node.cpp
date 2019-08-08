//histogram_match.cpp
//
//
//intensity_matchで作られた記述子を総当り戦でどうなるかをやっているプログラム
//

#include <ros/ros.h>
#include <iostream>
#include "realtime_histogram_match_pf.hpp"


int main(int argc, char** argv)
{
	ros::init(argc, argv, "histogram_match_pf");
	ros::NodeHandle n;
	ros::NodeHandle priv_nh("~");
	// ros::Rate loop(10);

	ROS_INFO("\033[1;32m---->\033[0m realtime run Started.");

	H_match_pf h_match_pf(n,priv_nh);	
	ros::Rate loop(1);

	// ros::spin();
	while(ros::ok()){
	
		// if(pf_view.flag) break;
		loop.sleep();
		ros::spinOnce();
	}
	// ros::spin();

	return(0);
}



