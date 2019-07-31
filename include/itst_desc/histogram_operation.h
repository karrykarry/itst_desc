#ifndef _HISTOGRAM_OPERATION_H_
#define _HISTOGRAM_OPERATION_H_

#include <vector>
#include <fileope.h>

class file_operation;

class histogram_operation
{
  public:
    histogram_operation();
    ~histogram_operation();
    void read_ref_histogram_f(); 
    void read_ref_histogram_b(); 
    void match_histogram(std::vector<std::vector<std::vector<int> > >  ref_histogram_f,
                         int test_number, std::string forb, std::vector<double> &result);
    //単一のヒストグラム比較 forb->x軸　test_forb->y軸
    void match_histogram_one(std::string forb, std::string test_forb);
    void match_histogram_all();

  private:
    file_operation file_ope;
    std::vector<std::vector<std::vector<int> > >  ref_histogram_f;
    std::vector<std::vector<std::vector<int> > >  ref_histogram_b;
    std::vector<std::vector<double> > histogram_result;

};

#endif
