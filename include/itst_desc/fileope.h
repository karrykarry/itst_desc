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
    file_operation();
    ~file_operation();
    //距離によってヒストグラム出力
    void output_hist_dist(std::vector<std::vector<int> > histogram, double distance);
    //ノード距離によってヒストグラム出力
    void output_hist_dist_s_f(std::vector<std::vector<int> > histogram);
    void output_hist_dist_s_b(std::vector<std::vector<int> > histogram);
    //時間によってヒストグラム作成
    void output_hist_time_f(std::vector<std::vector<int> > histogram, ros::Time ros_begin);
    void output_hist_time_b(std::vector<std::vector<int> > histogram, ros::Time ros_begin);
    //ヒストグラム読み込み
    void input_histogram(std::vector<std::vector<int> > &histogram_tmp, int input_count, std::string forb);
    void output_hist_result(std::vector<std::vector<double> > result);  //for test
    int file_count(std::string forb);

    const double output_time = 5.0;  //histogramを算出する間隔 time
    const double output_dist = 5.0;  //histogramを算出する間隔 distance
  
  private:
    void name_files(std::string &file_name, int count, std::string forb);
    void open_output_log(std::string output_file);
    void write_histogram_dist(std::vector<std::vector<int> > histogram, double distance);
    void write_histogram_time(std::vector<std::vector<int> > histogram, ros::Time ros_begin);

    void open_input_log(std::string input_file);
    void read(std::vector<std::vector<int> > &histogram_tmp);

    std::ofstream output_log;
    int output_count_f;
    int output_count_b;
    const std::string file_dir = "/home/amsl/histogram";
    const std::string file_dir2 = "/test";
    const std::string file_ext = ".csv";

    std::ifstream input_log;
    const int histogram_vol = 16;  //読み込むhistogramの個数

};

#endif
