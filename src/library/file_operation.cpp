//file_operation.cpp

#include <ros/ros.h>
#include <fileope.h>
#include <stdlib.h>
#include <sstream>
#include <sys/stat.h>

file_operation::file_operation(ros::NodeHandle n,ros::NodeHandle private_nh_) :
	output_count(0)
{
	private_nh_.param("ITST_DESC/FILE_DIR", file_dir, {"/home/amsl/Pictures/ros_catkin_ws/ref_data_manager"});
	private_nh_.param("ITST_DESC/FILE_DIR2", file_dir2, {"/sample"});
	private_nh_.param("ITST_DESC/FILE_DIR3", file_dir3, {"/itst_desc"});
	private_nh_.param("ITST_DESC/FILE_EXT", file_ext, {".csv"});

	std::cout << histogram_vol << std::endl;
}

file_operation::~file_operation()
{}

void file_operation::name_files(std::string &file_name, int count)
{
  std::string histogram_n = std::to_string(count);
  file_name = file_dir;
  file_name.append(file_dir2);
  file_name.append(file_dir3);
  file_name.append("/");
  file_name.append(histogram_n);
  file_name.append(file_ext);
}

void file_operation::open_output_log(std::string output_file){
  output_log.open(output_file, std::ios::trunc);
  if(!output_log.is_open()){
    std::cout << "NOT OPEN FILE" << std::endl;
    exit(1);
  }
}

void file_operation::write_histogram_dist(std::vector<std::vector<int> > histogram,
                                          double distance)
{
  for(size_t i = 0; i < histogram.size(); i++){
    for(size_t j = 0; j < histogram[i].size(); j++){
      output_log << histogram[i][j];
      if(j != histogram[i].size() - 1){
        output_log << ",";
      }
    }
    output_log << std::endl;
  }
  output_log << "Distance = " << distance << std::endl;
}

void file_operation::write_histogram_time(std::vector<std::vector<int> > histogram,
                           ros::Time ros_begin)
{
  for(size_t i = 0; i < histogram.size(); i++){
    for(size_t j = 0; j < histogram[i].size(); j++){
      output_log << histogram[i][j];
      if(j != histogram[i].size() - 1){
        output_log << ",";
      }
    }
    output_log << std::endl;
  }
  output_log << "Time = " << ros::Time::now().sec - ros_begin.sec << std::endl;
}

void file_operation::output_hist_dist(std::vector<std::vector<int> > histogram,
                                      double distance)
{
  if(fmod(distance, output_dist) <= 0.3 && distance >= output_count * output_dist){
    std::cout << "create histogram dist: " << distance << std::endl;
    std::string file_name;
    name_files(file_name, output_count);
    output_count++;
    std::cout << "output filename  " << file_name << std::endl;
    open_output_log(file_name);
    write_histogram_dist(histogram, distance);
    output_log.close();
  }
}

void file_operation::output_hist_dist_split(std::vector<std::vector<int> > histogram)
{
  std::cout << "create histogram number: " << output_count << std::endl;
  std::string file_name;
  name_files(file_name, output_count);
  output_count++;
  std::cout << "output filename  " << file_name << std::endl;
  open_output_log(file_name);
  write_histogram_dist(histogram, 0);
  output_log.close();
}

void file_operation::output_hist_time(std::vector<std::vector<int> > histogram,
                                 ros::Time ros_begin)
{
  if(fmod(ros::Time::now().sec  - ros_begin.sec, output_time) <= 1e-3
          && (int)(ros::Time::now().sec - ros_begin.sec) >= output_count * output_time){
    std::cout << "create histogram  sec: " << ros::Time::now().sec - ros_begin.sec << std::endl;
    std::string file_name;
    name_files(file_name, output_count);
    output_count++;
    std::cout << "output filename  " << file_name << std::endl;
    open_output_log(file_name);
    write_histogram_time(histogram, ros_begin);
    output_log.close();
  }
}

void file_operation::open_input_log(std::string file_name)
{
  input_log.open(file_name, std::ios::in);
  if(!input_log.is_open()){
    std::cout << "NOT OPEN FILE" << std::endl;
    exit(1);
  }
}

void file_operation::read(std::vector<std::vector<int> > &histogram_tmp)
{
  std::string str = "";
  for(int i = 0; i < histogram_vol; i++){
    getline(input_log, str);
    std::vector<int> histogram_tmp_one;
    std::string tmp = "";
    std::istringstream stream(str);
    while (getline(stream, tmp, ',')){
      histogram_tmp_one.push_back(stoi(tmp));
    }
    histogram_tmp.push_back(histogram_tmp_one);
  }
}

void file_operation::input_histogram(std::vector<std::vector<int> > &histogram_tmp, int input_count)
{
  std::string file_name;
  name_files(file_name, input_count);
  open_input_log(file_name);
  //std::cout << "input filename  " << file_name << std::endl;
  read(histogram_tmp);
  input_log.close();
}

void file_operation::output_hist_result(std::vector<std::vector<double> > result)
{
  std::string file_name;
  file_name = file_dir;
  file_name.append(file_dir2);
  file_name.append("/");
  file_name.append("histogram_result");
  file_name.append(file_ext);
  open_output_log(file_name);

  for(size_t i = 0; i < result.size(); i++){
    for(size_t j = 0; j < result[i].size(); j++){
      output_log << result[i][j];
      if(j != result[i].size() - 1){
        output_log << ",";
      }
    }
    output_log << std::endl;
  }
  std::cout << "output complete file -> " << file_name << std::endl;
  std::cout << "finish" << std::endl;
  output_log.close();
}

int file_operation::file_count(void)
{
  int i = 0;
  while(1){
    std::string file_name;
    name_files(file_name, i);
    const char *file = file_name.c_str();
    struct stat statbuf;

    if(stat(file, &statbuf) == 0){
      i++;
    }else{
      break;
    }
  }
  //std::cout << "file volume = " << i << std::endl;
  return i;
}















//
// //// 20190914 add ////
//
// void file_operation::output_hist_dist_s_f(std::vector<std::vector<int> > histogram,
//                                           nav_msgs::Odometry odometry)
// {
//   std::cout << "create histogram number: " << output_count_f << "  FRONT" << std::endl;
//   std::string file_name;
//   name_files(file_name, output_count_f, "f");
//   output_count_f++;
//   std::cout << "output filename  " << file_name << std::endl;
//   open_output_log(file_name);
//   write_histogram_coordinate(histogram, odometry);
//   output_log.close();
// }
//
// void file_operation::output_hist_dist_s_b(std::vector<std::vector<int> > histogram,
//                                           nav_msgs::Odometry odometry)
// {
//   std::cout << "create histogram number: " << output_count_b << "  BACK" << std::endl;
//   std::string file_name;
//   name_files(file_name, output_count_b, "b");
//   output_count_b++;
//   std::cout << "output filename  " << file_name << std::endl;
//   open_output_log(file_name);
//   write_histogram_coordinate(histogram, odometry);
//   output_log.close();
// }
//
//
//
//
