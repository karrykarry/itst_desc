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

	typedef pcl::PointXYZI PointXYZINormal;
	typedef pcl::PointCloud<PointXYZINormal> CloudXYZINormal;
	typedef pcl::PointCloud<PointXYZINormal>::Ptr CloudXYZINormalPtr;


	descriptor(ros::NodeHandle n,ros::NodeHandle private_nh_);
	void itst_descriptor_one(CloudXYZINormalPtr input_pc,
			std::vector<std::vector<int> > &histogram);
	void itst_descriptor(CloudXYZINormalPtr input_pc,
			std::vector<std::vector<int> > &histogram_f,
			std::vector<std::vector<int> > &histogram_b);


	std::vector<CloudXYZINormalPtr> split_pc_f;  //正方向
	std::vector<CloudXYZINormalPtr> split_pc_b;  //負方向
	std::vector<Eigen::Vector3d> eigenvector;
	//std::vector<std::vector<int> > histogram;


	private:

	void calibration(CloudXYZINormalPtr input_pc); //not used
	void split_pcs(CloudXYZINormalPtr input_pc,
			std::vector<CloudXYZINormalPtr> &split_pc,
			double split_angle);  //記述子用の点群分割
	void init_pc_vec(std::vector<CloudXYZINormalPtr> &split_pc);  //pc_vecの初期化
	double tidy_rad(double rad_in);  //角度を整える 
	void calc_RF(CloudXYZINormalPtr input_pc);  //RFの算出
	void init_calc_RF(void);  //変数の初期化
	void sort_eigen(void);  //固有値の小さい順に並べ替え
	double calc_split_angle(void);
	void calc_histogram(std::vector<std::vector<int> > &histogram,
			std::vector<CloudXYZINormalPtr> split_pc_f);

	size_t input_pc_size;
	const int split_pc_n_v = 4;  //pcの縦分割数
	const int split_pc_n_b = 2;  //pcの球分割数
	int split_r1;  //外側の球の半径
	int split_r2;  //内側の球の半径
	Eigen::Matrix3d M;
	std::vector<double> eigenvalue;
	int bin_num;  //histogramのbinの数
};


#endif
