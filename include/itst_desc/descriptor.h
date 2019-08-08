#ifndef _DESCRIPTOR_H_
#define _DESCRIPTOR_H_

#include <pcl/point_cloud.h>
#include <pcl_ros/point_cloud.h>
#include <vector>
#include <math.h>
#include <Eigen/Core>
#include <Eigen/Dense>

#include <fstream>
#include <iostream>


class descriptor
{
  public:
  descriptor();
  struct Particle{
	  double x;
	  double y;
	  double weight;
  };
 
  
  void itst_descriptor(const pcl::PointCloud<pcl::PointXYZI>::Ptr input_pc,
                       std::vector<std::vector<int> > &histogram);
  
  void itst_descriptor(const pcl::PointCloud<pcl::PointXYZI>::Ptr input_pc,
                       std::vector<std::vector<int> > &histogram,
                       Particle pf);
 
  std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr > split_pc;
  std::vector<Eigen::Vector3d> eigenvector;
  //std::vector<std::vector<int> > histogram;


  private:
  void calibration(pcl::PointCloud<pcl::PointXYZI>::Ptr input_pc); //not used
  void split_pcs(pcl::PointCloud<pcl::PointXYZI>::Ptr input_pc);  //記述子用の点群分割
  void init_pc_vec(void);  //pc_vecの初期化
  double tidy_rad(double rad_in);  //角度を整える 
  void calc_RF(const pcl::PointCloud<pcl::PointXYZI>::Ptr input_pc);  //RFの算出
  
  void split_pcs(pcl::PointCloud<pcl::PointXYZI>::Ptr input_pc, Particle pf);  //記述子用の点群分割
  void calc_RF(const pcl::PointCloud<pcl::PointXYZI>::Ptr input_pc, Particle pf);  //RFの算出
  
  
  void init_calc_RF(void);  //変数の初期化
  void sort_eigen(void);  //固有値の小さい順に並べ替え
  void calc_histogram(std::vector<std::vector<int> > &histogram);

  size_t input_pc_size;
  const int split_pc_n_v = 4;  //pcの縦分割数
  const int split_pc_n_b = 2;  //pcの球分割数
  const int split_r1 = 10;  //外側の球の半径
  const int split_r2 = 5;  //内側の球の半径
  int intensity_max;
  int intensity_min;
  Eigen::Matrix3d M;
  std::vector<double> eigenvalue;
  static const int bin_num = 256;  //histogramのbinの幅
 
};


#endif
