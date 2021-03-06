//////////////////////////
//descriptor.cpp

#include "descriptor_normal.h"

descriptor::descriptor(ros::NodeHandle n,ros::NodeHandle private_nh_){
	
	private_nh_.param("bin_num", bin_num, {11});
	private_nh_.param("split_r1", split_r1, {60});
	private_nh_.param("split_r2", split_r2, {20});
}

void descriptor::calibration(CloudXYZINormalPtr input_pc)
{}  //not used

void descriptor::init_pc_vec(std::vector<CloudXYZINormalPtr> &split_pc)
{
  //前半内球後半外球  0radから順に配列割り振り 
  std::vector<CloudXYZINormalPtr>().swap(split_pc);
  int pc_vec_size = 2 * split_pc_n_v * split_pc_n_b;
  for(int i = 0; i < pc_vec_size; i++){
    CloudXYZINormalPtr init_pc (new CloudXYZINormal);
    split_pc.push_back(init_pc);
  }
}

double descriptor::tidy_rad(double rad_in)
{
  if(rad_in > 2.0 * M_PI){
    rad_in = rad_in - 2 * M_PI;
  }else if(rad_in < -2.0 * M_PI){
    rad_in = rad_in + 4 * M_PI;
  }else if(rad_in < 0){
    rad_in = rad_in + 2.0 * M_PI;
  }
  return rad_in;
}

void descriptor::split_pcs(CloudXYZINormalPtr input_pc,
                           std::vector<CloudXYZINormalPtr> &split_pc,
                           double split_angle)
{
  init_pc_vec(split_pc);

  for(size_t i = 0; i < input_pc_size; i++){
    PointXYZINormal temp;
    temp.x = input_pc -> points[i].x;
    temp.y = input_pc -> points[i].y;
    temp.z = input_pc -> points[i].z;
    temp.intensity = input_pc -> points[i].intensity;
    temp.curvature  = input_pc -> points[i].curvature;
		
    double pc_rad = atan2(temp.y ,temp.x) - split_angle;
    pc_rad = tidy_rad(pc_rad);
    //std::cout << "pc_theta" << pc_rad * 180 / M_PI << std::endl;

    double pc_d = sqrt(pow(temp.x, 2) + pow(temp.y, 2) + pow(temp.z, 2));
    //std::cout << "pc_d" << pc_d << std::endl;

    int split_n = pc_rad / (2 * M_PI) * split_pc_n_v;
    //std::cout << "n_orig/ " << split_n << std::endl;

    if(pc_d <= split_r2 && temp.z >= 0){  //内球上半分
      split_pc[split_n] -> points.push_back(temp);
    }else if(pc_d <= split_r2 && temp.z < 0){  //内球下半分
      split_n += split_pc_n_v;
      split_pc[split_n] -> points.push_back(temp);
    }else if(pc_d > split_r2 && pc_d < split_r1 && temp.z >= 0){  //外球上半分
      split_n += split_pc_n_v * 2;
      split_pc[split_n] -> points.push_back(temp);
    }else if(pc_d > split_r2 && pc_d < split_r1 && temp.z < 0){  //外球下半分
      split_n += split_pc_n_v * 3;
      split_pc[split_n] -> points.push_back(temp);
    }

  }

  //for(int i=0; i<16; i++){
  //  std::cout << "n= " << i << " -> " << split_pc[i] -> points.size() << std::endl;
  //}
}

void descriptor::init_calc_RF(void)
{
  M << Eigen::MatrixXd::Zero(3,3);
  eigenvalue.clear();
  eigenvector.clear();
}

void descriptor::sort_eigen(void)
{
  for(size_t j = 0; j < eigenvalue.size(); j++){
    for(size_t i = 0; i < eigenvalue.size(); i++){
      if(eigenvalue[i] > eigenvalue[i+1]){
        eigenvalue.push_back(eigenvalue[i]);
        eigenvalue.erase(eigenvalue.begin() + i);
        eigenvector.push_back(eigenvector[i]);
        eigenvector.erase(eigenvector.begin() + i);
      }
    }
  }

  //for(int i = 0; i < eigenvalue.size(); i++){
  //  std::cout << "sort va " << eigenvalue[i] << std::endl;
  //  std::cout << "sort v " << eigenvector[i] << std::endl;
  //}
}

void descriptor::calc_RF(CloudXYZINormalPtr input_pc)
{
  init_calc_RF();
  double R2di_sum = 0;

  for(size_t i = 0; i < input_pc_size; i++){
    double R2di = split_r1 - sqrt(pow(input_pc -> points[i].x, 2) + pow(input_pc -> points[i].y, 2) + pow(input_pc -> points[i].z, 2));
    if(R2di >= 0){
      R2di_sum += R2di;

      Eigen::MatrixXd p(3,1);
      Eigen::MatrixXd p2(1,3);
      p << input_pc -> points[i].x,
           input_pc -> points[i].y,
           input_pc -> points[i].z;
      p2 << input_pc -> points[i].x, input_pc -> points[i].y, input_pc -> points[i].z;

      Eigen::Matrix3d pp2;
      pp2 = R2di * p * p2;
      M += pp2;
    }
  }
  
  M /= R2di_sum;
  //std::cout << "M/ " << M << std::endl;
  
  Eigen::EigenSolver<Eigen::Matrix3d> es(M);
  for(int i = 0; i < 3; i++){
    eigenvalue.push_back(es.eigenvalues()(i).real());
    eigenvector.push_back(es.eigenvectors().col(i).real());
  }
  //std::cout << "The eigenvalues:\n" << es.eigenvalues() << std::endl;
  //std::cout << "eigenvectors: \n" << es.eigenvectors() << std::endl;
  
  sort_eigen();

  double Sx_p = 0;
  double Sx_m = 0;
  double Sz_p = 0;
  double Sz_m = 0;
  double Sy_p = 0;
  double Sy_m = 0;
  for(size_t i = 0; i < input_pc_size; i++){
    double R2di = split_r1 - sqrt(pow(input_pc -> points[i].x, 2) + pow(input_pc -> points[i].y, 2) + pow(input_pc -> points[i].z, 2));
    
    if(R2di >= 0){
      Eigen::Vector3d p;
      p << input_pc -> points[i].x,
           input_pc -> points[i].y,
           input_pc -> points[i].z;
      double pdotx_p = p.dot(eigenvector[0]);  
      double pdotx_m = p.dot(-1 * eigenvector[0]);
      double pdotz_p = p.dot(eigenvector[2]);  
      double pdotz_m = p.dot(-1 * eigenvector[2]);
      double pdoty_p = p.dot(eigenvector[1]);  
      double pdoty_m = p.dot(-1 * eigenvector[1]);
      
      if(pdotx_p >= 0){
        Sx_p++;
      }else if(pdotx_m > 0){
        Sx_m++;
      }

      if(pdotz_p >= 0){
        Sz_p++;
      }else if(pdotz_m > 0){
        Sz_m++;
      }

      if(pdoty_p >= 0){
        Sy_p++;
      }else if(pdoty_m > 0){
        Sy_m++;
      }
    }
  } 
  // std::cout << "Sx_p " << Sx_p << "Sx_m " << Sx_m << std::endl;
  // std::cout << "Sz_p " << Sz_p << "Sz_m " << Sz_m << std::endl;
  // std::cout << "Sy_p " << Sy_p << "Sy_m " << Sy_m << std::endl;

  if(Sx_p < Sx_m){
    eigenvector[0] = -eigenvector[0];
  }
  if(Sz_p < Sz_m){
    eigenvector[2] = -eigenvector[2];
  }
  eigenvector[1] = eigenvector[2].cross(eigenvector[0]);
   
  //for(size_t i = 0; i < eigenvalue.size(); i++){
  //  std::cout << "final vec " << i << " : " << eigenvector[i] << std::endl;
  //}
  //std::cout << "final vec " << 1 << " : " << eigenvector[1] << std::endl;

}

double descriptor::calc_split_angle(void)
{
  double angle = atan2(eigenvector[1](1), eigenvector[1](0));
  angle = tidy_rad(angle);
  //std::cout << "angle " << angle << std::endl;
  return angle;
}

void descriptor::calc_histogram(std::vector<std::vector<int> > &histogram,
                                    std::vector<CloudXYZINormalPtr> split_pc)
{
  histogram.clear();
  for(size_t i = 0; i < split_pc.size(); i++){
    std::vector<int> histogram_temp(bin_num);
    for(size_t j = 0; j < split_pc[i] -> points.size(); j++){
      int index = (int)split_pc[i] -> points[j].intensity;
      histogram_temp[index] ++;
		//sq2用
	   // int index = (int)split_pc[i] -> points[j].intensity / 16.0;
	   // if(index < bin_num)histogram_temp[index] ++;
    }
    histogram.push_back(histogram_temp);
  }
}


void descriptor::calc_histogram_curvature(std::vector<std::vector<int> > &histogram,
                                    std::vector<CloudXYZINormalPtr> split_pc)
{
  histogram.clear();
  for(size_t i = 0; i < split_pc.size(); i++){
    std::vector<int> histogram_temp(bin_num);
    for(size_t j = 0; j < split_pc[i] -> points.size(); j++){
      int index = (int)(split_pc[i] -> points[j].curvature*10);
      histogram_temp[index] ++;
		//sq2用
	   // int index = (int)split_pc[i] -> points[j].intensity / 16.0;
	   // if(index < bin_num)histogram_temp[index] ++;
    }
    histogram.push_back(histogram_temp);
  }
}


void descriptor::itst_descriptor_one(CloudXYZINormalPtr input_pc,
                                 std::vector<std::vector<int> > &histogram)
{
  input_pc_size = input_pc -> points.size();
  calc_RF(input_pc);
  double split_angle = calc_split_angle();
  split_pcs(input_pc, split_pc_f, split_angle);
  calc_histogram_curvature(histogram, split_pc_f);
}

void descriptor::itst_descriptor(CloudXYZINormalPtr input_pc,
                                 std::vector<std::vector<int> > &histogram_f,
                                 std::vector<std::vector<int> > &histogram_b)
{
  input_pc_size = input_pc -> points.size();
  calc_RF(input_pc);
  double split_angle = calc_split_angle();
  split_pcs(input_pc, split_pc_f, split_angle);
  split_pcs(input_pc, split_pc_b, split_angle + M_PI); 
  calc_histogram_curvature(histogram_f, split_pc_f);
  calc_histogram_curvature(histogram_b, split_pc_b);
}
