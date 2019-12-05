#ifndef _HISTOGRAM_OPERATION_H_
#define _HISTOGRAM_OPERATION_H_

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <vector>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/Int32.h>
// #include "fileope.h"

class file_operation;

class histogram_operation
{
	public:
		histogram_operation(ros::NodeHandle n,ros::NodeHandle private_nh_);	
		~histogram_operation();
		//保存されたhistogramの入力
		void read_ref_histogram_f(); 
		void read_ref_histogram_b();
		//histogramのマッチング

		void match_histogram_cos(std::vector<std::vector<std::vector<int>>>, std::vector<std::vector<int>>, std::vector<double>&);
		void match_histogram(std::vector<std::vector<std::vector<int> > >  ref_histogram_f,
				std::vector<std::vector<int> > test_histogram, std::vector<double> &result);
		void match_histogram(std::vector<std::vector<std::vector<int> > >  ref_histogram, std::vector<std::vector<int> > test_histogram, std_msgs::Float64MultiArray &result, int &best_pr_num);
		//単一のヒストグラム比較 forb->x軸 test_forb->y軸
		void match_histogram_one_output(std::string forb, std::string test_forb);
		void match_histogram_all_output(std::string forb);
		//最小誤差のノードを検索
		void research_match_one(std::vector<std::vector<int> > histogram);
		
		
		//最小誤差のノードを検索
		void research_match_pubscore(std::vector<std::vector<int> > histogram, std_msgs::Float64MultiArray &score, std_msgs::Int32 &best_score);
		
		//第nth番までのノードを検索
		void research_match_pubscore_n(std::vector<std::vector<int> > histogram, std::vector<int>& better_score, std_msgs::Float64MultiArray& eval_score);
		
		
		//第nth番までのノードを検索
		void research_match_n(std::vector<std::vector<int> > histogram, int nth);
		//ノード検索とodomでの評価を出力
		void evaluate_match_n(nav_msgs::Odometry odometry, std::vector<std::vector<int> > histogram, int nth);

		void loop_close(void);


	private:
		file_operation* file_ope;
		void match_histogram_one(std::string forb, std::string test_forb);
		void match_histogram_all(std::string forb);
		void joint_histogram(void);
		void cal_norm(void);
		std::string rename_hist_number(size_t index, std::pair<int, std::string> &rename_hist);

		std::vector<std::vector<std::vector<int> > >  ref_histogram_f;
		std::vector<std::vector<std::vector<int> > >  ref_histogram_b;
		std::vector<std::vector<std::vector<int> > >  ref_histogram_all;
		std::vector<std::vector<double> >  ref_histogram_norm;	//cos_similarity 



		std::vector<std::vector<double> > histogram_result;
		int ref_hist_vol_f;
		int ref_hist_vol_b;
		std::vector<std::pair<int, std::string> > candidate_hist;  //number forb
		std::vector<std::pair<double, size_t> > close_nodes;
		const double loop_close_threshold = 600.0;


		struct FINAL_ANS{	
			float score;
			int num;
		};
		FINAL_ANS final_ans;

};

#endif
