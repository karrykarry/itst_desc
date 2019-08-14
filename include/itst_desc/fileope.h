#ifndef _FILEOPE_H_
#define _FILEOPE_H_

#include <ros/ros.h>
#include <fstream>
#include <iostream>
#include <string.h>
#include <vector>

class file_operation
{
  public:
	file_operation(ros::NodeHandle n,ros::NodeHandle private_nh_);
    ~file_operation();
    void output_hist_dist(std::vector<std::vector<int> > histogram, double distance);
    void output_hist_dist_split(std::vector<std::vector<int> > histogram);
    void output_hist_time(std::vector<std::vector<int> > histogram, ros::Time ros_begin);
    void input_histogram(std::vector<std::vector<int> > &histogram_tmp, int input_count);
    void output_hist_result(std::vector<std::vector<double> > result);  //for test
    int file_count();

    const double output_time = 5.0;  //histogramを算出する間隔 time
    const double output_dist = 5.0;  //histogramを算出する間隔 distance
  
  private:
    void name_files(std::string &file_name, int count);
    void open_output_log(std::string output_file);
    void write_histogram_dist(std::vector<std::vector<int> > histogram, double distance);
    void write_histogram_time(std::vector<std::vector<int> > histogram, ros::Time ros_begin);

    void open_input_log(std::string input_file);
    void read(std::vector<std::vector<int> > &histogram_tmp);

    std::ofstream output_log;
    int output_count;
    // const std::string file_dir = "/home/amsl/Pictures/ros_catkin_ws/itst_desc";
    // const std::string file_dir2 = "/test";
    // const std::string file_ext = ".csv";
	std::string file_dir, file_dir2, file_dir3, file_ext;

    std::ifstream input_log;
    constexpr static int histogram_vol = 16;  //読み込むhistogramの個数

};

#endif
