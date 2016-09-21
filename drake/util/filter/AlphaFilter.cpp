#include "AlphaFilter.hpp"
#include <stdio.h>

using namespace Eigen;
using namespace std;

namespace FilterTools {

AlphaFilter::AlphaFilter(const double & breakFrequencyInHz, bool verbose):
     breakFrequencyInHz_(breakFrequencyInHz), t_prev_(0), init_(false), verbose_(verbose),
     x_filtered_prev_(0){}


double AlphaFilter::processSample(const double & t, const double & x){
  if (!init_) {
     x_filtered_prev_ = x;
     init_ = true;
     t_prev_ = t;
     return x;
  }

  double dt = t - t_prev_;
  double alpha = computeAlphaGivenBreakFrequencyInHz(breakFrequencyInHz_, dt);
  double x_filtered =  alpha * x_filtered_prev_  + (1 - alpha) * x;

  x_filtered_prev_ = x_filtered;
  t_prev_ = t;
  return x_filtered;
}


}
