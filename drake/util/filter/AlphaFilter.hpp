#ifndef __ALPHA_FILTER_HPP__
#define __ALPHA_FILTER_HPP__

#include <iostream>
#include <inttypes.h>
#include <Eigen/Dense>
#include <Eigen/StdVector>
#include <Eigen/Core>
#include <cmath>

#include "SimpleFilter.h"

namespace FilterTools {

inline double computeAlphaGivenBreakFrequencyInHz(double breakFrequencyInHz, double dt){

    if (isinf(breakFrequencyInHz)){
        return 0;
    }

    double omega = 2.0 * M_PI * breakFrequencyInHz;
    double alpha = (1 - omega * dt / 2.0) / (1.0 + omega * dt / 2.0);
    alpha = std::min(1.0, alpha);
    alpha = std::max(0.0, alpha);

    return alpha;
}

class AlphaFilter: public SimpleFilter{
  public:
    AlphaFilter() { 
        breakFrequencyInHz_= 1e9;
        init_ = false;
        verbose_ = false;
    }

    AlphaFilter(const double & breakFrequencyInHz, bool verbose = false);
    virtual ~AlphaFilter() {}

    double processSample(const double & x, const double & t);

  private:
    double breakFrequencyInHz_;
    double t_prev_;
    bool init_;
    bool verbose_;
    double x_filtered_prev_;
};

}

#endif
