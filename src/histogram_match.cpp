//histogram_match.cpp
//
//
//intensity_matchで作られた記述子を総当り戦でどうなるかをやっているプログラム
//

#include <ros/ros.h>
#include <iostream>
#include <histogram_operation.h>


int main(int argc, char** argv)
{
  ros::init(argc, argv, "histogram_match");
  ros::NodeHandle n;
  ros::NodeHandle priv_nh("~");
  
  histogram_operation hist_ope(n,priv_nh);

  hist_ope.read_ref_histogram();
  hist_ope.match_histogram_all();

  // ros::spin();
  
  return(0);
}
