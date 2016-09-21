//
// Created by manuelli on 9/20/16.
//

#ifndef DRAKE_SUPERBUILD_SLEWFILTER_H
#define DRAKE_SUPERBUILD_SLEWFILTER_H

#include "SimpleFilter.h"

namespace FilterTools{
  class SlewFilter: public SimpleFilter {
  public:
    SlewFilter(const double & maxDeltaPerSecond);
    ~SlewFilter(){}

    double processSample(const double& t, const double & x);


  private:
    const double & maxDeltaPerSecond_;
    double tPrev_;
    double xPrev_;
    bool init_;

  };

}


#endif //DRAKE_SUPERBUILD_SLEWFILTER_H
