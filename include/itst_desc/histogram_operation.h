#ifndef _HISTOGRAM_OPERATION_H_
#define _HISTOGRAM_OPERATION_H_

#include <vector>
#include <fileope.h>

#define MIN_(x,y) ((x) < (y) ? (x) : (y))

class file_operation;

class histogram_operation
{
  public:
    histogram_operation();
    ~histogram_operation();
    void read_ref_histogram(); 
    void match_histogram(int test_number, std::vector<double> &result);
    void match_histogram_all();

	void match_histogram_pc(const std::vector<std::vector<int> > test_histogram);


  private:
    file_operation* file_ope;
    std::vector < std::vector < std::vector <int> > >  ref_histogram;	//$B;2>H%G!<%?(B

	struct FINAL_ANS{	
		float score;
		int num;
	};
	FINAL_ANS final_ans;

};

#endif
