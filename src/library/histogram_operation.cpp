//histogram_operation.cpp

#include "histogram_operation.h"
#include "fileope.h"

histogram_operation::histogram_operation(ros::NodeHandle n,ros::NodeHandle private_nh_):
	ref_hist_vol_f(0), ref_hist_vol_b(0)
{
	file_ope = new file_operation(n, private_nh_);

	read_ref_histogram_f();
	read_ref_histogram_b();
	joint_histogram();
	cal_norm();
}

histogram_operation::~histogram_operation()
{
	std::vector<std::vector<std::vector<int> > >().swap(ref_histogram_f);
	std::vector<std::vector<std::vector<int> > >().swap(ref_histogram_b);
	std::vector<std::vector<std::vector<int> > >().swap(ref_histogram_all);
	std::vector<std::vector<double> >().swap(ref_histogram_norm);
	std::vector<std::vector<double> >().swap(histogram_result);
	std::vector<std::pair<int, std::string> >().swap(candidate_hist);
	std::vector<std::pair<double, size_t> >().swap(close_nodes);

	delete file_ope;
}

void histogram_operation::read_ref_histogram_f(void)
{
	ref_histogram_f.clear();
	ref_hist_vol_f = file_ope->file_count("f");
	for(int i = 0; i < ref_hist_vol_f; i++){
		std::vector<std::vector<int> > ref_histogram_tmp;
		file_ope->input_histogram(ref_histogram_tmp, i, "f");
		ref_histogram_f.push_back(ref_histogram_tmp);
	}
}

void histogram_operation::read_ref_histogram_b(void)
{
	ref_histogram_b.clear();
	const int ref_hist_vol_b = file_ope->file_count("b");
	for(int i = 0; i < ref_hist_vol_b; i++){
		std::vector<std::vector<int> > ref_histogram_tmp;
		file_ope->input_histogram(ref_histogram_tmp, i, "b");
		ref_histogram_b.push_back(ref_histogram_tmp);
	}
}

void histogram_operation::cal_norm(void){
	std::vector<double> average;
	for(auto one_node : ref_histogram_all){
		average.clear();
		for(auto one_bin : one_node){
			double one_sum = 0;
			for(auto one_descriptor : one_bin){
				one_sum += pow(one_descriptor, 2);
			}
			average.push_back(pow(one_sum, 0.5));
		}
		ref_histogram_norm.push_back(average);
	}
	for(auto ref_ : ref_histogram_norm){
		for(auto rr : ref_){
			std::cout<<rr<<", "<<std::flush;
		}
		std::cout<<std::endl;
	}
}

void histogram_operation::match_histogram_cos(std::vector<std::vector<std::vector<int> > >  ref_histogram, std::vector<std::vector<int> > test_histogram, std::vector<double> &result)
{
	bool first_flag = false;
	std::vector<double> test_norm;
	for(size_t i = 0; i < ref_histogram.size(); i++){
		double ith_histogram_average = 0;
		for(size_t j = 0; j < ref_histogram[0].size(); j++){
			double histogram_one = 0;
			double test_sum = 0;
			for(size_t k = 0; k < ref_histogram[0][0].size(); k++){
				if(test_histogram[j][k] + ref_histogram[i][j][k] != 0){
					histogram_one += ref_histogram[i][j][k] * test_histogram[j][k];
					if(!first_flag) test_sum += pow(test_histogram[j][k], 2);
				}
			}
			if(!first_flag) test_norm.push_back(pow(test_sum, 0.5));
			if(ref_histogram_norm[i][j] && test_norm[j])
				histogram_one /= (ref_histogram_norm[i][j]*test_norm[j]);
			else if(!ref_histogram_norm[i][j] && !test_norm[j]){
				histogram_one = 1.0;
			}
			else histogram_one = 0.0;
		
			ith_histogram_average += histogram_one;
		}
		
		ith_histogram_average /= (double)ref_histogram[0].size();
		result.push_back(ith_histogram_average);
		first_flag = true;
	}

}





void histogram_operation::match_histogram(std::vector<std::vector<std::vector<int> > >  ref_histogram, std::vector<std::vector<int> > test_histogram, std::vector<double> &result)
{
	for(size_t i = 0; i < ref_histogram.size(); i++){
		double ith_histogram_average = 0;
		for(size_t j = 0; j < ref_histogram[0].size(); j++){
			double histogram_one = 0;
			for(size_t k = 0; k < ref_histogram[0][0].size(); k++){
				if(test_histogram[j][k] + ref_histogram[i][j][k] != 0){
					histogram_one += 2 * pow(test_histogram[j][k] - ref_histogram[i][j][k], 2) / (test_histogram[j][k] + ref_histogram[i][j][k]);
				}
			}
			ith_histogram_average += histogram_one;
		}
		ith_histogram_average /= (double)ref_histogram[0].size();
		result.push_back(ith_histogram_average);
	}

}



void histogram_operation::match_histogram(std::vector<std::vector<std::vector<int> > >  ref_histogram, std::vector<std::vector<int> > test_histogram, std_msgs::Float64MultiArray &result, int &best_pr_num)
{
	bool flag = false;
	
	for(size_t i = 0; i < ref_histogram.size(); i++){
		double ith_histogram_average = 0;
		for(size_t j = 0; j < ref_histogram[0].size(); j++){
			double histogram_one = 0;
			for(size_t k = 0; k < ref_histogram[0][0].size(); k++){
				if(test_histogram[j][k] + ref_histogram[i][j][k] != 0){
					histogram_one += 2 * pow(test_histogram[j][k] - ref_histogram[i][j][k], 2) / (test_histogram[j][k] + ref_histogram[i][j][k]);
				}
			}
			ith_histogram_average += histogram_one;
		}
		ith_histogram_average /= (double)ref_histogram[0].size();
		
		double score_;
		if(ith_histogram_average==0.0) score_ = 10000.0;
		
		else score_ = 10000.0 / ith_histogram_average;

		result.data.push_back(score_);
	
		if(!flag){
			final_ans.score = ith_histogram_average;
			final_ans.num= i;
			flag = true;
		}
		else{
			if(ith_histogram_average < final_ans.score){
				final_ans.score = ith_histogram_average;
				final_ans.num = i; 
			}	
		}
	}
	
	std::cout<<"number:"<<final_ans.num<<",score:"<<final_ans.score<<std::endl;

	best_pr_num = final_ans.num;
}



void histogram_operation::match_histogram_one(std::string forb, std::string test_forb)
{
	histogram_result.clear();
	int ref_hist_vol = 0;
	if(forb == "f"){
		std::cout << "Matching histograms \n" << "TEST      -> FRONT" << std::endl;
		ref_hist_vol = file_ope->file_count("f");
	}else if(forb == "b"){
		std::cout << "Matching histograms \n" << "TEST      -> BACK" << std::endl;
		ref_hist_vol = file_ope->file_count("b");
	}else{
		std::cout << "Can't match histograms -> " << forb << std::endl;
		exit(0);
	}

	if(test_forb == "f"){
		std::cout << "REFERENCE -> FRONT" << std::endl;
		for(int i = 0; i < ref_hist_vol; i++){
			std::vector<std::vector<int> > test_histogram;
			std::vector<double> histogram_result_one;
			file_ope->input_histogram(test_histogram, i, forb);
			match_histogram(ref_histogram_f, test_histogram, histogram_result_one);
			histogram_result.push_back(histogram_result_one);
		}
	}else{
		std::cout << "REFERENCE -> BACK" << std::endl;
		for(int i = 0; i < ref_hist_vol; i++){
			std::vector<std::vector<int> > test_histogram;
			std::vector<double> histogram_result_one;
			file_ope->input_histogram(test_histogram, i, forb);
			match_histogram(ref_histogram_f, test_histogram, histogram_result_one);
			histogram_result.push_back(histogram_result_one);
		}
	}
}

void histogram_operation::match_histogram_one_output(std::string forb, std::string test_forb)
{
	match_histogram_one(forb, test_forb);
	file_ope->output_hist_result(histogram_result);
}

void histogram_operation::joint_histogram(void)
{
	ref_histogram_all.clear();
	int hist_vol = file_ope->file_count("f");
	for(int i = 0; i < hist_vol; i++){
		ref_histogram_all.push_back(ref_histogram_f[i]);
	}

	hist_vol = file_ope->file_count("b");
	for(int i = 0; i < hist_vol; i++){
		ref_histogram_all.push_back(ref_histogram_b[i]);
	}
}

void histogram_operation::match_histogram_all(std::string forb)
{
	if(forb == "f"){
		std::cout << "Matching histograms ALL\n" << "TEST   -> FRONT" << std::endl;
	}else if(forb == "b"){
		std::cout << "Matching histograms ALL\n" << "TEST   -> BACK" << std::endl;
	}else{
		std::cout << "Can't match histograms -> " << forb << std::endl;
		exit(0);
	}
	const int ref_hist_vol = file_ope->file_count(forb);
	joint_histogram();

	for(int i = 0; i < ref_hist_vol; i++){
		std::vector<std::vector<int> > test_histogram;
		std::vector<double> histogram_result_one;
		file_ope->input_histogram(test_histogram, i, forb);
		match_histogram(ref_histogram_all, test_histogram, histogram_result_one);
		histogram_result.push_back(histogram_result_one);
	}
}

void histogram_operation::match_histogram_all_output(std::string forb)
{
	match_histogram_all(forb);
	file_ope->output_hist_result(histogram_result);
}

std::string histogram_operation::rename_hist_number(size_t index, std::pair<int, std::string> &rename_hist)
{
	if((int)index < ref_hist_vol_f){
		rename_hist.first = (int)index;
		rename_hist.second = "f";
		std::string s_index = std::to_string(index);
		s_index.append("  FRONT");
		return s_index;
	}else if((int)index >= ref_hist_vol_f){
		index -= ref_hist_vol_f;
		rename_hist.first = (int)index;
		rename_hist.second = "b";
		std::string s_index = std::to_string(index);
		s_index.append("  BACK");
		return s_index;
	}
	return "ERROR -> rename_hist_number";
}

void histogram_operation::research_match_one(std::vector<std::vector<int> > histogram)
{
	read_ref_histogram_f();
	read_ref_histogram_b();
	joint_histogram();
	std::vector<double> result;
	match_histogram(ref_histogram_all, histogram, result);
	
	double min = *std::min_element(result.begin(), result.end());
	std::vector<double>::iterator minIt = std::min_element(result.begin(), result.end());
	size_t minIndex = std::distance(result.begin(), minIt);

	close_nodes.clear();
	close_nodes.resize(1);
	close_nodes[0] = std::make_pair(min, minIndex);
	std::pair<int, std::string> rename_hist;
	std::string min_hist = rename_hist_number(minIndex, rename_hist);
	std::cout << "value " << min << "  index " << min_hist << std::endl;
}




void histogram_operation::research_match_pubscore(std::vector<std::vector<int> > histogram, std_msgs::Float64MultiArray &score, std_msgs::Int32 &best_score)
{
	read_ref_histogram_f();
	read_ref_histogram_b();
	joint_histogram();

	int best_score_;
	match_histogram(ref_histogram_all, histogram, score, best_score_);
	
	best_score.data = (best_score_ < ref_hist_vol_f ? best_score_ :  best_score_ - ref_hist_vol_f);


}

void histogram_operation::research_match_pubscore_n(
		std::vector<std::vector<int> > histogram, 
		std::vector<int>& better_score)
{
	std::vector<double> result;
	match_histogram(ref_histogram_all, histogram, result);
	
	std::vector<double> cos_similarity;
	match_histogram_cos(ref_histogram_all, histogram, cos_similarity);
	
	
	std::vector<std::pair<double, size_t> > cos_similarity_id;

	close_nodes.clear();
	close_nodes.resize(result.size());
	cos_similarity_id.resize(result.size());
	for(size_t i = 0; i < result.size(); i++){
		close_nodes[i] = std::make_pair(result[i], i);
		cos_similarity_id[i] = std::make_pair(cos_similarity[i], i);
	}
	std::sort(close_nodes.begin(), close_nodes.end());
	std::sort(cos_similarity_id.begin(), cos_similarity_id.end(), std::greater<std::pair<double, size_t>>());


	int nth = better_score.size();

	//std::cout << "--------------" << std::endl;
	for(int i = 0; i < nth; i++){
		std::pair<int, std::string> rename_hist;
		std::string min_hist = rename_hist_number(close_nodes[i].second, rename_hist);
		candidate_hist.push_back(rename_hist);
		
		better_score[i] = (close_nodes[i].second < ref_hist_vol_f ? close_nodes[i].second :  close_nodes[i].second - ref_hist_vol_f);
		std::cout << i << "th -> value: " << better_score[i] << " score:" << close_nodes[i].first << std::endl;

	}
	std::cout<<std::endl;
	for(int i =0; i < nth ; i++){
		double cos_id = (cos_similarity_id[i].second < ref_hist_vol_f ? cos_similarity_id[i].second :  cos_similarity_id[i].second - ref_hist_vol_f);

		better_score[i] = (int)cos_id;
		std::cout << i << "th -> value: " << cos_id << " score:" << cos_similarity_id[i].first << std::endl;
	}
	
}



void histogram_operation::research_match_n(std::vector<std::vector<int> > histogram, int nth)
{
	read_ref_histogram_f();
	read_ref_histogram_b();
	joint_histogram();
	std::vector<double> result;
	match_histogram(ref_histogram_all, histogram, result);

	close_nodes.clear();
	close_nodes.resize(result.size());
	for(size_t i = 0; i < result.size(); i++){
		close_nodes[i] = std::make_pair(result[i], i);
	}
	std::sort(close_nodes.begin(), close_nodes.end());

	//std::cout << "--------------" << std::endl;
	for(int i = 0; i < nth; i++){
		std::pair<int, std::string> rename_hist;
		std::string min_hist = rename_hist_number(close_nodes[i].second, rename_hist);
		candidate_hist.push_back(rename_hist);
		//std::cout << i << "th -> value: " << close_nodes[i].first << "  index: " << min_hist << std::endl;
	}
}

void histogram_operation::evaluate_match_n(nav_msgs::Odometry odometry, std::vector<std::vector<int> > histogram, int nth)
{
	candidate_hist.clear();
	research_match_n(histogram, nth);
	std::vector<double> distance_vec;
	for(int i = 0; i < nth; i++){
		nav_msgs::Odometry odometry_tmp;
		file_ope->odom_input(odometry_tmp, candidate_hist[i]);
		double distance = sqrt(pow(odometry.pose.pose.position.x - odometry_tmp.pose.pose.position.x, 2) + pow(odometry.pose.pose.position.y - odometry_tmp.pose.pose.position.y, 2));
		distance_vec.push_back(distance);
		//std::cout << i << "th odom_x :" << odometry_tmp.pose.pose.position.x << std::endl;
		//std::cout << i << "th odom_y :" << odometry_tmp.pose.pose.position.y << std::endl;
		std::cout << i << "th distance :" << distance << std::endl;
	}
	file_ope->odom_dist_chk(distance_vec);
}

void histogram_operation::loop_close(void)
{
	ref_hist_vol_f = file_ope->file_count("f");
	int research_n = 3;
	for(int i = 0; i < ref_hist_vol_f; i++){
		std::vector<std::vector<int> > test_histogram;
		file_ope->input_histogram(test_histogram, i, "f");
		research_match_n(test_histogram, research_n);
		std::cout << "-----------------" << std::endl;
		for(int j = 0; j < research_n; j++){
			std::pair<int, std::string> rename_hist;
			std::string min_hist = rename_hist_number(close_nodes[j].second, rename_hist);
			if(rename_hist.first != i){
				std::cout << i << "th  value: " << close_nodes[j].first << "  node: " << min_hist << std::endl;
				if(i - rename_hist.first >= 10 && close_nodes[j].first <= loop_close_threshold){
					std::cout << "loop close" << std::endl;
				}
				break;
			}
		} 
	}
}


