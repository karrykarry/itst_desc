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
    void read_ref_histogram(); 
    void match_histogram(int test_number, std::vector<double> &result);
    void match_histogram_all();

  private:
    file_operation* file_ope;
    std::vector<std::vector<std::vector<int> > >  ref_histogram;

};

#endif
