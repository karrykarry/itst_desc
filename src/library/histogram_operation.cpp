//histogram_operation.cpp

#include <histogram_operation.h>

histogram_operation::histogram_operation()
{
	file_ope = new file_operation();
}

histogram_operation::~histogram_operation()
{
  std::vector<std::vector<std::vector<int> > >().swap(ref_histogram);
}

void histogram_operation::read_ref_histogram(void)
{
  ref_histogram.clear();
  const int ref_hist_vol = file_ope->file_count();
  for(int i = 0; i < ref_hist_vol; i++){
    std::vector<std::vector<int> > ref_histogram_tmp;
    file_ope->input_histogram(ref_histogram_tmp, i);
    ref_histogram.push_back(ref_histogram_tmp);
  }
}

void histogram_operation::match_histogram(int test_number, std::vector<double> &result)
{
  std::vector<std::vector<int> > test_histogram;
  file_ope->input_histogram(test_histogram, test_number);

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

void histogram_operation::match_histogram_all(void)
{
  std::cout << "matching all histograms" << std::endl;
  std::vector<std::vector<double> > histogram_result;
  const int ref_hist_vol = file_ope->file_count();
  for(int i = 0; i < ref_hist_vol; i++){
    std::vector<double> histogram_result_one;
    match_histogram(i, histogram_result_one);
    histogram_result.push_back(histogram_result_one);
  }
  file_ope->output_hist_result(histogram_result);
} 