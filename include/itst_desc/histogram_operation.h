#ifndef _HISTOGRAM_OPERATION_H_
#define _HISTOGRAM_OPERATION_H_

#include <vector>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/Int32.h>
#include "fileope.h"

#define MIN_(x,y) ((x) < (y) ? (x) : (y))

class file_operation;

class histogram_operation
{
  public:
    histogram_operation(ros::NodeHandle n,ros::NodeHandle private_nh_);
    ~histogram_operation();
    void read_ref_histogram(); 
    void match_histogram(int test_number, std::vector<double> &result);
    void match_histogram_all();

	void match_histogram_pc(const std::vector<std::vector<int> > test_histogram, std_msgs::Float64MultiArray &score, std_msgs::Int32 &best_pr_num);
	
	void match_histogram_pc_pf(const std::vector<std::vector<int> > test_histogram, double &score_);
	


  private:
    file_operation* file_ope;
    std::vector < std::vector < std::vector <int> > >  ref_histogram;	//参照データ

	struct FINAL_ANS{	
		float score;
		int num;
	};
	FINAL_ANS final_ans;

};

#endif
