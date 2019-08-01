//histogram_operation.cpp

#include <histogram_operation.h>

histogram_operation::histogram_operation()
{
  ref_hist_vol_f = 0;
  ref_hist_vol_b = 0;
}

histogram_operation::~histogram_operation()
{
  std::vector<std::vector<std::vector<int> > >().swap(ref_histogram_f);
  std::vector<std::vector<std::vector<int> > >().swap(ref_histogram_b);
  std::vector<std::vector<std::vector<int> > >().swap(ref_histogram_all);
  std::vector<std::vector<double> >().swap(histogram_result);
}

void histogram_operation::read_ref_histogram_f(void)
{
  ref_histogram_f.clear();
  ref_hist_vol_f = file_ope.file_count("f");
  for(int i = 0; i < ref_hist_vol_f; i++){
    std::vector<std::vector<int> > ref_histogram_tmp;
    file_ope.input_histogram(ref_histogram_tmp, i, "f");
    ref_histogram_f.push_back(ref_histogram_tmp);
  }
}

void histogram_operation::read_ref_histogram_b(void)
{
  ref_histogram_b.clear();
  const int ref_hist_vol_b = file_ope.file_count("b");
  for(int i = 0; i < ref_hist_vol_b; i++){
    std::vector<std::vector<int> > ref_histogram_tmp;
    file_ope.input_histogram(ref_histogram_tmp, i, "b");
    ref_histogram_b.push_back(ref_histogram_tmp);
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

void histogram_operation::match_histogram_one(std::string forb, std::string test_forb)
{
  histogram_result.clear();
  int ref_hist_vol = 0;
  if(forb == "f"){
    std::cout << "Matching histograms \n" << "TEST      -> FRONT" << std::endl;
    ref_hist_vol = file_ope.file_count("f");
  }else if(forb == "b"){
    std::cout << "Matching histograms \n" << "TEST      -> BACK" << std::endl;
    ref_hist_vol = file_ope.file_count("b");
  }else{
    std::cout << "Can't match histograms -> " << forb << std::endl;
    exit(0);
  }

  if(test_forb == "f"){
    std::cout << "REFERENCE -> FRONT" << std::endl;
    for(int i = 0; i < ref_hist_vol; i++){
      std::vector<std::vector<int> > test_histogram;
      std::vector<double> histogram_result_one;
      file_ope.input_histogram(test_histogram, i, forb);
      match_histogram(ref_histogram_f, test_histogram, histogram_result_one);
      histogram_result.push_back(histogram_result_one);
    }
  }else{
    std::cout << "REFERENCE -> BACK" << std::endl;
    for(int i = 0; i < ref_hist_vol; i++){
      std::vector<std::vector<int> > test_histogram;
      std::vector<double> histogram_result_one;
      file_ope.input_histogram(test_histogram, i, forb);
      match_histogram(ref_histogram_f, test_histogram, histogram_result_one);
      histogram_result.push_back(histogram_result_one);
    }
  }
}

void histogram_operation::match_histogram_one_output(std::string forb, std::string test_forb)
{
  match_histogram_one(forb, test_forb);
  file_ope.output_hist_result(histogram_result);
}

void histogram_operation::joint_histogram(void)
{
  ref_histogram_all.clear();
  int hist_vol = file_ope.file_count("f");
  for(int i = 0; i < hist_vol; i++){
    ref_histogram_all.push_back(ref_histogram_f[i]);
  }
 
  hist_vol = file_ope.file_count("b");
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
  const int ref_hist_vol = file_ope.file_count(forb);
  joint_histogram();
  
  for(int i = 0; i < ref_hist_vol; i++){
    std::vector<std::vector<int> > test_histogram;
    std::vector<double> histogram_result_one;
    file_ope.input_histogram(test_histogram, i, forb);
    match_histogram(ref_histogram_all, test_histogram, histogram_result_one);
    histogram_result.push_back(histogram_result_one);
  }
}
 
void histogram_operation::match_histogram_all_output(std::string forb)
{
  match_histogram_all(forb);
  file_ope.output_hist_result(histogram_result);
}

std::string histogram_operation::rename_hist_number(size_t index)
{
  if((int)index < ref_hist_vol_f){
    std::string s_index = std::to_string(index);
    s_index.append("  FRONT");
    return s_index;
  }else if((int)index >= ref_hist_vol_f){
    index -= (ref_hist_vol_f - 1);
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
  std::string min_hist = rename_hist_number(minIndex);
  std::cout << "value " << min << "  index " << min_hist << std::endl;
}

void histogram_operation::research_match_n(std::vector<std::vector<int> > histogram, int nth)
{
  read_ref_histogram_f();
  read_ref_histogram_b();
  joint_histogram();
  std::vector<double> result;
  match_histogram(ref_histogram_all, histogram, result);

  std::vector<std::pair<double, size_t> > i_result(result.size());
  for(size_t i = 0; i < result.size(); i++){
    i_result[i] = std::make_pair(result[i], i);
  }
  std::sort(i_result.begin(), i_result.end());
  
  for(int i = 0; i < nth; i++){
    std::string min_hist = rename_hist_number(i_result[i].second);
    std::cout << i << "th -> value: " << i_result[i].first << "  index: " << min_hist << std::endl;
  }
  std::cout << "--------------" << std::endl;
}
