#ifndef _HISTOGRAM_OPERATION_H_
#define _HISTOGRAM_OPERATION_H_

#include <vector>
#include <fileope.h>
#include <nav_msgs/Odometry.h>

class file_operation;

class histogram_operation
{
  public:
    histogram_operation();
    ~histogram_operation();
    void read_ref_histogram_f(); 
    void read_ref_histogram_b(); 
    void match_histogram(std::vector<std::vector<std::vector<int> > >  ref_histogram_f,
                         std::vector<std::vector<int> > test_histogram, std::vector<double> &result);
    //単一のヒストグラム比較 forb->x軸　test_forb->y軸
    void match_histogram_one_output(std::string forb, std::string test_forb);
    void match_histogram_all_output(std::string forb);
    //最小誤差のノードを検索
    void research_match_one(std::vector<std::vector<int> > histogram);
    //第nth番までのノードを検索
    void research_match_n(std::vector<std::vector<int> > histogram, int nth);
    void evaluate_match_n(nav_msgs::Odometry odometry, std::vector<std::vector<int> > histogram, int nth);

  private:
    void match_histogram_one(std::string forb, std::string test_forb);
    void match_histogram_all(std::string forb);
    void joint_histogram(void);
    std::string rename_hist_number(size_t index, std::pair<int, std::string> &rename_hist);
 
    file_operation file_ope;
    std::vector<std::vector<std::vector<int> > >  ref_histogram_f;
    std::vector<std::vector<std::vector<int> > >  ref_histogram_b;
    std::vector<std::vector<std::vector<int> > >  ref_histogram_all;
    std::vector<std::vector<double> > histogram_result;
    int ref_hist_vol_f;
    int ref_hist_vol_b;
    //std::vector<std::string> candidate_hist;
    std::vector<std::pair<int, std::string> > candidate_hist;

};

#endif
