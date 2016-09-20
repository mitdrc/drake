//
// Created by manuelli on 9/20/16.
//

#ifndef DRAKE_SUPERBUILD_SIMPLEFILTER_H
#define DRAKE_SUPERBUILD_SIMPLEFILTER_H

#endif //DRAKE_SUPERBUILD_SIMPLEFILTER_H

namespace FilterTools{
  class SimpleFilter{
  public:
    virtual double processSample(const double & t, const double & x) = 0;
    virtual ~SimpleFilter(){};
  };
}
