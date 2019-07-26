#ifndef _LOCALIZE_H_
#define _LOCALIZE_H_

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>

class localize
{
  public:
    localize();
    ~localize();
    double calc_distance_r(nav_msgs::Odometry input);  //併用
    double calc_distance_n(nav_msgs::Odometry input);  //不可
    bool split_metre(nav_msgs::Odometry input, double output_dist);

  private:
    bool first_odom_flag;
    nav_msgs::Odometry last_odom_r;
    nav_msgs::Odometry last_odom_n;
    double distance;

};

#endif
