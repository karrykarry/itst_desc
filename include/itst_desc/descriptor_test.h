#ifndef _DESCRIPTOR_TEST_H_
#define _DESCRIPTOR_TEST_H_

#include <pcl/point_cloud.h>
#include <pcl_ros/point_cloud.h>
#include <fstream>
#include <iostream>
#include <vector>

class descriptor_test
{
  public:
  void itst_descriptor(pcl::PointCloud<pcl::PointXYZI>::Ptr input_pc, double &Sx_p,
                       double &Sx_m, double &Sy_p, double &Sy_m, double &Sz_p, double &Sz_m);
 
  std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr > split_pc;
  std::vector<Eigen::Vector3d> eigenvector;
 
  private:
  void calibration(pcl::PointCloud<pcl::PointXYZI>::Ptr input_pc); //not used
  void split_pcs(pcl::PointCloud<pcl::PointXYZI>::Ptr input_pc);  //記述子用の点群分割
  void init_pc_vec(void);  //pc_vecの初期化
  double tidy_rad(double rad_in);  //角度を整える
  //test1 内積の正負で決定 
  void calc_RF_test1(pcl::PointCloud<pcl::PointXYZI>::Ptr input_pc, double &Sx_p, double &Sx_m, double &Sy_p, double &Sy_m, double &Sz_p, double &Sz_m);
  //test2 内積の値を使用
  void calc_RF_test2(pcl::PointCloud<pcl::PointXYZI>::Ptr input_pc, double &Sx_p, double &Sx_m, double &Sy_p, double &Sy_m, double &Sz_p, double &Sz_m);
  //test3 法線情報を使用する予定
  void calc_RF_test3(pcl::PointCloud<pcl::PointXYZI>::Ptr input_pc, double &Sx_p, double &Sx_m, double &Sy_p, double &Sy_m, double &Sz_p, double &Sz_m);
  //RFの算出
  void init_calc_RF(void);  //変数の初期化
  void sort_eigen(void);  //固有値の小さい順に並べ替え

  size_t input_pc_size;
  //std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr > split_pc;
  static const int split_pc_n_v = 4;  //pcの縦分割数
  static const int split_pc_n_b = 2;  //pcの球分割数
  static const int split_r1 = 30;  //外側の球の半径
  static const int split_r2 = 5;  //内側の球の半径
  Eigen::Matrix3d M;
  std::vector<double> eigenvalue;
  //std::vector<Eigen::Vector3d> eigenvector;

};


#endif
