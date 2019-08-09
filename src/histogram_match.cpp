//histogram_match.cpp
//
//
//intensity_match$B$G:n$i$l$?5-=R;R$rAmEv$j@o$G$I$&$J$k$+$r$d$C$F$$$k%W%m%0%i%`(B
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
