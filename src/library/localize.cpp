//localize.cpp

#include <localize.h>

localize::localize()
{
  first_odom_flag = true;
  distance = 0;
};

localize::~localize()
{};

double localize::calc_distance_r(nav_msgs::Odometry input)
{
  if(first_odom_flag){
    last_odom_r = input;
    first_odom_flag = false;
  }else{
    double temp_x = input.pose.pose.position.x - last_odom_r.pose.pose.position.x;
    double temp_y = input.pose.pose.position.y - last_odom_r.pose.pose.position.y;
    distance += sqrt(pow(temp_x ,2) + pow(temp_y, 2));
    std::cout << "distance " << distance << std::endl;
    last_odom_r = input;
  } 
  return distance;
}

double localize::calc_distance_n(nav_msgs::Odometry input)
{
  double temp_x = input.pose.pose.position.x - last_odom_n.pose.pose.position.x;
  double temp_y = input.pose.pose.position.y - last_odom_n.pose.pose.position.y;
  distance = sqrt(pow(temp_x ,2) + pow(temp_y, 2));
  std::cout << "distance " << distance << std::endl;

  return distance;
}

bool localize::split_metre(nav_msgs::Odometry input, double output_dist)
{
  if(first_odom_flag){
    last_odom_n = input;
    first_odom_flag = false;
    return true;
  }
  else{
    double dist_ = calc_distance_n(input);
  	// std::cout << dist_ - output_dist << std::endl;
    if(dist_ >= output_dist){
      last_odom_n = input;
      return true;
    }
  }
  return false;
}

