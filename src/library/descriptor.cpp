//////////////////////////
//descriptor.cpp

#include <descriptor.h>
descriptor::descriptor(){

}

void descriptor::calibration(pcl::PointCloud<pcl::PointXYZI>::Ptr input_pc)
{}  //not used

void descriptor::init_pc_vec(void)
{
  //前半内球後半外球  0radから順に配列割り振り 
  std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr >().swap(split_pc);
  int pc_vec_size = 2 * split_pc_n_v * split_pc_n_b;
  for(int i = 0; i < pc_vec_size; i++){
    pcl::PointCloud<pcl::PointXYZI>::Ptr init_pc (new pcl::PointCloud<pcl::PointXYZI>);
    split_pc.push_back(init_pc);
  }
}

double descriptor::tidy_rad(double rad_in)
{
  if(rad_in < 0){
    rad_in = 2 * M_PI + rad_in;
  }
  return rad_in;
}

void descriptor::split_pcs(pcl::PointCloud<pcl::PointXYZI>::Ptr input_pc)
{
  input_pc_size = input_pc -> points.size();
  init_pc_vec();
  intensity_max = 0;  
  intensity_min = 255;  

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

    if(intensity_max < temp.intensity) intensity_max = temp.intensity;  
    if(intensity_min > temp.intensity) intensity_min = temp.intensity;  
  }

  //for(int i=0; i<16; i++){
  //  std::cout << "n= " << i << " -> " << split_pc[i] -> points.size() << std::endl;
  //}
  //std::cout << "intensity_max " << intensity_max << std::endl; 
  //std::cout << "intensity_min " << intensity_min << std::endl; 
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

void descriptor::calc_RF(const pcl::PointCloud<pcl::PointXYZI>::Ptr input_pc)
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
  //std::cout << "Sx_p " << Sx_p << std::endl;
  //std::cout << "Sx_m " << Sx_m << std::endl;
  //std::cout << "Sz_p " << Sz_p << std::endl;
  //std::cout << "Sz_m " << Sz_m << std::endl;
  //std::cout << "Sy_p " << Sy_p << std::endl;
  //std::cout << "Sy_m " << Sy_m << std::endl;

  if(Sx_p < Sx_m){
    eigenvector[0] = -eigenvector[0];
  }
  if(Sz_p < Sz_m){
    eigenvector[2] = -eigenvector[2];
  }
  eigenvector[1] = eigenvector[2].cross(eigenvector[0]);
   
  //for(size_t i = 0; i < eigenvalue.size(); i++){
  //  std::cout << "final vec " << eigenvector[i] << std::endl;
  //}
}

void descriptor::calc_histogram(std::vector<std::vector<int> > &histogram)
{
  histogram.clear();
  for(size_t i = 0; i < split_pc.size(); i++){
    std::vector<int> histogram_temp(bin_num);
    for(size_t j = 0; j < split_pc[i] -> points.size(); j++){
      int index = (int)split_pc[i] -> points[j].intensity;
      histogram_temp[index] ++;
    }
    histogram.push_back(histogram_temp);
  }
}

void descriptor::itst_descriptor(const pcl::PointCloud<pcl::PointXYZI>::Ptr input_pc,
                                 std::vector<std::vector<int> > &histogram)
{
  split_pcs(input_pc);
  calc_RF(input_pc);
  calc_histogram(histogram);
  
}

//////////////////   Particle      /////////////////////////


void descriptor::split_pcs(pcl::PointCloud<pcl::PointXYZI>::Ptr input_pc, Particle pf)
{
  input_pc_size = input_pc -> points.size();
  init_pc_vec();
  intensity_max = 0;  
  intensity_min = 255;  

  for(size_t i = 0; i < input_pc_size; i++){
    pcl::PointXYZI temp;
    temp.x = input_pc -> points[i].x - pf.x;
    temp.y = input_pc -> points[i].y - pf.y;
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

    if(intensity_max < temp.intensity) intensity_max = temp.intensity;  
    if(intensity_min > temp.intensity) intensity_min = temp.intensity;  
  }

  //for(int i=0; i<16; i++){
  //  std::cout << "n= " << i << " -> " << split_pc[i] -> points.size() << std::endl;
  //}
  //std::cout << "intensity_max " << intensity_max << std::endl; 
  //std::cout << "intensity_min " << intensity_min << std::endl; 
}



void descriptor::calc_RF(const pcl::PointCloud<pcl::PointXYZI>::Ptr input_pc, Particle pf)
{
	init_calc_RF();
	double R2di_sum = 0;

	for(size_t i = 0; i < input_pc_size; i++){

		pcl::PointXYZI temp;
		temp.x = input_pc -> points[i].x - pf.x;
		temp.y = input_pc -> points[i].y - pf.y;
		temp.z = input_pc -> points[i].z;
		temp.intensity = input_pc -> points[i].intensity;


		double R2di = split_r1 - sqrt(pow(temp.x, 2) + pow(temp.y, 2) + pow(temp.z, 2));
		if(R2di >= 0){
			R2di_sum += R2di;

			Eigen::MatrixXd p(3,1);
			Eigen::MatrixXd p2(1,3);
			p << temp.x,
			  temp.y,
			  temp.z;
			p2 << temp.x, temp.y, temp.z;

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

		pcl::PointXYZI temp;
		temp.x = input_pc -> points[i].x - pf.x;
		temp.y = input_pc -> points[i].y - pf.y;
		temp.z = input_pc -> points[i].z;
		temp.intensity = input_pc -> points[i].intensity;


		double R2di = split_r1 - sqrt(pow(temp.x, 2) + pow(temp.y, 2) + pow(temp.z, 2));

		if(R2di >= 0){
			Eigen::Vector3d p;
			p << temp.x,
			  temp.y,
			  temp.z;
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
	//std::cout << "Sx_p " << Sx_p << std::endl;
	//std::cout << "Sx_m " << Sx_m << std::endl;
	//std::cout << "Sz_p " << Sz_p << std::endl;
	//std::cout << "Sz_m " << Sz_m << std::endl;
	//std::cout << "Sy_p " << Sy_p << std::endl;
	//std::cout << "Sy_m " << Sy_m << std::endl;

	if(Sx_p < Sx_m){
		eigenvector[0] = -eigenvector[0];
	}
	if(Sz_p < Sz_m){
		eigenvector[2] = -eigenvector[2];
	}
	eigenvector[1] = eigenvector[2].cross(eigenvector[0]);

	//for(size_t i = 0; i < eigenvalue.size(); i++){
	//  std::cout << "final vec " << eigenvector[i] << std::endl;
	//}
}


void descriptor::itst_descriptor(const pcl::PointCloud<pcl::PointXYZI>::Ptr input_pc,
                                 std::vector<std::vector<int> > &histogram,
								 Particle pf)
{
  split_pcs(input_pc, pf);
  calc_RF(input_pc, pf);
  calc_histogram(histogram);
  
}





// /////////// 20190914 add //////////
// //
//
//
// double descriptor::calc_split_angle(void)
// {
//   double angle = atan2(eigenvector[1](1), eigenvector[1](0));
//   angle = tidy_rad(angle);
//   //std::cout << "angle " << angle << std::endl;
//   return angle;
// }
//
//
// void descriptor::split_pcs(pcl::PointCloud<pcl::PointXYZI>::Ptr input_pc,
//                            std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr > &split_pc,
//                            double split_angle)
// {
//   init_pc_vec(split_pc);
//
//   for(size_t i = 0; i < input_pc_size; i++){
//     pcl::PointXYZI temp;
//     temp.x = input_pc -> points[i].x;
//     temp.y = input_pc -> points[i].y;
//     temp.z = input_pc -> points[i].z;
//     temp.intensity = input_pc -> points[i].intensity;
//
//     double pc_rad = atan2(temp.y ,temp.x) - split_angle;
//     pc_rad = tidy_rad(pc_rad);
//     //std::cout << "pc_theta" << pc_rad * 180 / M_PI << std::endl;
//
//     double pc_d = sqrt(pow(temp.x, 2) + pow(temp.y, 2) + pow(temp.z, 2));
//     //std::cout << "pc_d" << pc_d << std::endl;
//
//     int split_n = pc_rad / (2 * M_PI) * split_pc_n_v;
//     //std::cout << "n_orig/ " << split_n << std::endl;
//
//     if(pc_d <= split_r2 && temp.z >= 0){  //内球上半分
//       split_pc[split_n] -> points.push_back(temp);
//     }else if(pc_d <= split_r2 && temp.z < 0){  //内球下半分
//       split_n += split_pc_n_v;
//       split_pc[split_n] -> points.push_back(temp);
//     }else if(pc_d > split_r2 && pc_d < split_r1 && temp.z >= 0){  //外球上半分
//       split_n += split_pc_n_v * 2;
//       split_pc[split_n] -> points.push_back(temp);
//     }else if(pc_d > split_r2 && pc_d < split_r1 && temp.z < 0){  //外球下半分
//       split_n += split_pc_n_v * 3;
//       split_pc[split_n] -> points.push_back(temp);
//     }
//
//   }
//
//   //for(int i=0; i<16; i++){
//   //  std::cout << "n= " << i << " -> " << split_pc[i] -> points.size() << std::endl;
//   //}
// }
//
// void descriptor::calc_histogram(std::vector<std::vector<int> > &histogram,
//                                     std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr > split_pc)
// {
//   histogram.clear();
//   for(size_t i = 0; i < split_pc.size(); i++){
//     std::vector<int> histogram_temp(bin_num);
//     for(size_t j = 0; j < split_pc[i] -> points.size(); j++){
//       int index = (int)split_pc[i] -> points[j].intensity;
//       histogram_temp[index] ++;
//     }
//     histogram.push_back(histogram_temp);
//   }
// }
//
// void descriptor::itst_descriptor(pcl::PointCloud<pcl::PointXYZI>::Ptr input_pc,
//                                  std::vector<std::vector<int> > &histogram_f,
//                                  std::vector<std::vector<int> > &histogram_b)
// {
//   input_pc_size = input_pc -> points.size();
//   calc_RF(input_pc);
//   double split_angle = calc_split_angle();
//   split_pcs(input_pc, split_pc_f, split_angle);
//   split_pcs(input_pc, split_pc_b, split_angle + M_PI);
//   calc_histogram(histogram_f, split_pc_f);
//   calc_histogram(histogram_b, split_pc_b);
// }
//
//
//
