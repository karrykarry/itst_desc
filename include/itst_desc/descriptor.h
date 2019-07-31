#ifndef _DESCRIPTOR_H_
#define _DESCRIPTOR_H_

#include <pcl/point_cloud.h>
#include <pcl_ros/point_cloud.h>
#include <vector>

class descriptor
{
  public:
  void itst_descriptor_one(pcl::PointCloud<pcl::PointXYZI>::Ptr input_pc,
                       std::vector<std::vector<int> > &histogram);
  void itst_descriptor(pcl::PointCloud<pcl::PointXYZI>::Ptr input_pc,
                       std::vector<std::vector<int> > &histogram_f,
                       std::vector<std::vector<int> > &histogram_b);
 
  std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr > split_pc_f;  //正方向
  std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr > split_pc_b;  //負方向
  std::vector<Eigen::Vector3d> eigenvector;
  //std::vector<std::vector<int> > histogram;
 
  private:
  void calibration(pcl::PointCloud<pcl::PointXYZI>::Ptr input_pc); //not used
  void split_pcs(pcl::PointCloud<pcl::PointXYZI>::Ptr input_pc,
                 std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr > &split_pc,
                 double split_angle);  //記述子用の点群分割
  void init_pc_vec(std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr > &split_pc);  //pc_vecの初期化
  double tidy_rad(double rad_in);  //角度を整える 
  void calc_RF(pcl::PointCloud<pcl::PointXYZI>::Ptr input_pc);  //RFの算出
  void init_calc_RF(void);  //変数の初期化
  void sort_eigen(void);  //固有値の小さい順に並べ替え
  double calc_split_angle(void);
  void calc_histogram(std::vector<std::vector<int> > &histogram,
                          std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr > split_pc_f);

  size_t input_pc_size;
  const int split_pc_n_v = 4;  //pcの縦分割数
  const int split_pc_n_b = 2;  //pcの球分割数
  const int split_r1 = 10;  //外側の球の半径
  const int split_r2 = 5;  //内側の球の半径
  Eigen::Matrix3d M;
  std::vector<double> eigenvalue;
  const int bin_num = 256;  //histogramのbinの数
 
};


#endif
