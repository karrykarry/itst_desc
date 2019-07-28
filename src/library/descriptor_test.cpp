#include <descriptor_test.h>
#include <math.h>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <fstream>
#include <iostream>


void descriptor_test::calibration(pcl::PointCloud<pcl::PointXYZI>::Ptr input_pc)
{}  //not used

void descriptor_test::init_pc_vec(void)
{
  //前半内球後半外球  0radから順に配列割り振り 
  std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr >().swap(split_pc);
  int pc_vec_size = 2 * split_pc_n_v * split_pc_n_b;
  for(int i = 0; i < pc_vec_size; i++){
    pcl::PointCloud<pcl::PointXYZI>::Ptr init_pc (new pcl::PointCloud<pcl::PointXYZI>);
    split_pc.push_back(init_pc);
  }
}

double descriptor_test::tidy_rad(double rad_in)
{
  if(rad_in < 0){
    rad_in = 2 * M_PI + rad_in;
  }
  return rad_in;
}

void descriptor_test::split_pcs(pcl::PointCloud<pcl::PointXYZI>::Ptr input_pc)
{
  input_pc_size = input_pc -> points.size();
  init_pc_vec();

  for(size_t i = 0; i < input_pc_size; i++){
    pcl::PointXYZI temp;
    temp.x = input_pc -> points[i].x;
    temp.y = input_pc -> points[i].y;
    temp.z = input_pc -> points[i].z;
    temp.intensity = input_pc -> points[i].intensity;

    double pc_rad = atan2(temp.y ,temp.x);
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

void descriptor_test::init_calc_RF(void)
{
  M << Eigen::MatrixXd::Zero(3,3);
  eigenvalue.clear();
  eigenvector.clear();
}

void descriptor_test::sort_eigen(void)
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

void descriptor_test::calc_RF_test1(pcl::PointCloud<pcl::PointXYZI>::Ptr input_pc, double &Sx_p, double &Sx_m, double &Sy_p, double &Sy_m, double &Sz_p, double &Sz_m)
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
  std::cout << "The eigenvalues:\n" << es.eigenvalues() << std::endl;
  std::cout << "eigenvectors: \n" << es.eigenvectors() << std::endl;
  
  sort_eigen();

  for(size_t i = 0; i < input_pc_size; i++){
    double R2di = split_r1 - sqrt(pow(input_pc -> points[i].x, 2) + pow(input_pc -> points[i].y, 2) + pow(input_pc -> points[i].z, 2));
    
    if(R2di >= 0){
      Eigen::Vector3d p;
      p << input_pc -> points[i].x,
           input_pc -> points[i].y,
           input_pc -> points[i].z;
      double pdotx_p = p.dot(eigenvector[0]);  
      double pdotx_m = p.dot(-1 * eigenvector[0]);
      if(pdotx_p >= 0){
        Sx_p += 1.0;
      }else if(pdotx_m > 0){
        Sx_m += 1.0;
      }
      
      if(p[2] > -0.6){    //地面除去
        double pdotz_p = p.dot(eigenvector[2]);  
        double pdotz_m = p.dot(-1 * eigenvector[2]);
        double pdoty_p = p.dot(eigenvector[1]);  
        double pdoty_m = p.dot(-1 * eigenvector[1]);
        if(pdotz_p >= 0){
          Sz_p += 1.0;
        }else if(pdotz_m > 0){
          Sz_m += 1.0;
        }

        if(pdoty_p >= 0){
          Sy_p += 1.0;
        }else if(pdoty_m > 0){
          Sy_m += 1.0;
        }
      }
    }
  } 
  //std::cout << "Sx_p " << Sx_p << std::endl;
  //std::cout << "Sx_m " << Sx_m << std::endl;
  //std::cout << "Sz_p " << Sz_p << std::endl;
  //std::cout << "Sz_m " << Sz_m << std::endl;
  //std::cout << "Sy_p " << Sy_p << std::endl;
  //std::cout << "Sy_m " << Sy_m << std::endl;

  if(Sx_p < Sx_m){
    eigenvector[0] = -eigenvector[0];
  }
  if(fabs(Sy_p - Sy_m) > fabs(Sz_p - Sz_m)){
    if(Sy_p < Sy_m){
      eigenvector[1] = -eigenvector[1];
    }
    eigenvector[1] = eigenvector[2].cross(eigenvector[0]);
  }else{
    if(Sz_p < Sz_m){
      eigenvector[2] = -eigenvector[2];
    }
    eigenvector[2] = eigenvector[0].cross(eigenvector[1]);
  }
   
  for(size_t i = 0; i < eigenvalue.size(); i++){
    std::cout << "final vec " << eigenvector[i] << std::endl;
  }
}

void descriptor_test::calc_RF_test2(pcl::PointCloud<pcl::PointXYZI>::Ptr input_pc, double &Sx_p, double &Sx_m, double &Sy_p, double &Sy_m, double &Sz_p, double &Sz_m)
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
  std::cout << "The eigenvalues:\n" << es.eigenvalues() << std::endl;
  std::cout << "eigenvectors: \n" << es.eigenvectors() << std::endl;
  
  sort_eigen();

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
        Sx_p += pdotx_p;
      }else if(pdotx_m > 0){
        Sx_m += pdotx_m;
      }

      if(pdotz_p >= 0){
        Sz_p += pdotz_p;
      }else if(pdotz_m > 0){
        Sz_m += pdotz_m;
      }

      if(pdoty_p >= 0){
        Sy_p += pdoty_p;
      }else if(pdoty_m > 0){
        Sy_m += pdoty_m;
      }
    }
  } 
  //std::cout << "Sx_p " << Sx_p << std::endl;
  //std::cout << "Sx_m " << Sx_m << std::endl;
  //std::cout << "Sz_p " << Sz_p << std::endl;
  //std::cout << "Sz_m " << Sz_m << std::endl;
  //std::cout << "Sy_p " << Sy_p << std::endl;
  //std::cout << "Sy_m " << Sy_m << std::endl;

  if(Sx_p < Sx_m){
    eigenvector[0] = -eigenvector[0];
  }
  if(Sy_p < Sy_m){
    eigenvector[2] = -eigenvector[2];
  }
  //eigenvector[2] = eigenvector[0].cross(eigenvector[1]);
  eigenvector[1] = eigenvector[2].cross(eigenvector[0]);
   
  for(size_t i = 0; i < eigenvalue.size(); i++){
    std::cout << "final vec " << eigenvector[i] << std::endl;
  }
}

void descriptor_test::calc_RF_test3(pcl::PointCloud<pcl::PointXYZI>::Ptr input_pc, double &Sx_p, double &Sx_m, double &Sy_p, double &Sy_m, double &Sz_p, double &Sz_m)
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
  std::cout << "The eigenvalues:\n" << es.eigenvalues() << std::endl;
  std::cout << "eigenvectors: \n" << es.eigenvectors() << std::endl;
  
  sort_eigen();

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
        Sx_p += pdotx_p;
      }else if(pdotx_m > 0){
        Sx_m += pdotx_m;
      }

      if(pdotz_p >= 0){
        Sz_p += pdotz_p;
      }else if(pdotz_m > 0){
        Sz_m += pdotz_m;
      }

      if(pdoty_p >= 0){
        Sy_p += pdoty_p;
      }else if(pdoty_m > 0){
        Sy_m += pdoty_m;
      }
    }
  } 
  //std::cout << "Sx_p " << Sx_p << std::endl;
  //std::cout << "Sx_m " << Sx_m << std::endl;
  //std::cout << "Sz_p " << Sz_p << std::endl;
  //std::cout << "Sz_m " << Sz_m << std::endl;
  //std::cout << "Sy_p " << Sy_p << std::endl;
  //std::cout << "Sy_m " << Sy_m << std::endl;

  if(Sx_p < Sx_m){
    eigenvector[0] = -eigenvector[0];
  }
  if(Sy_p < Sy_m){
    eigenvector[2] = -eigenvector[2];
  }
  //eigenvector[2] = eigenvector[0].cross(eigenvector[1]);
  eigenvector[1] = eigenvector[2].cross(eigenvector[0]);
   
  for(size_t i = 0; i < eigenvalue.size(); i++){
    std::cout << "final vec " << eigenvector[i] << std::endl;
  }
}

void descriptor_test::itst_descriptor(pcl::PointCloud<pcl::PointXYZI>::Ptr input_pc, double &Sx_p, double &Sx_m, double &Sy_p, double &Sy_m, double &Sz_p, double &Sz_m)
{
  split_pcs(input_pc);
  calc_RF_test1(input_pc, Sx_p, Sx_m, Sy_p, Sy_m, Sz_p, Sz_m);
  //calc_RF_test2(input_pc, Sx_p, Sx_m, Sy_p, Sy_m, Sz_p, Sz_m);

}
