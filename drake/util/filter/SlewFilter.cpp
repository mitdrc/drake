//
// Created by manuelli on 9/20/16.
//

#include "SlewFilter.h"
#include <algorithm>
namespace FilterTools{

  SlewFilter::SlewFilter(const double &maxDeltaPerSecond):maxDeltaPerSecond_(maxDeltaPerSecond){};

  double SlewFilter::processSample(const double &t, const double &x) {
    if (!this->init_){
      init_ = true;
      tPrev_ = t;
      xPrev_ = x;
      return x;
    }

    double dt = t - tPrev_;
    double xMax = xPrev_ + maxDeltaPerSecond_*dt;
    double xMin = xPrev_ - maxDeltaPerSecond_*dt;

    double xClip = std::min(x, xMax);
    xClip = std::max(xClip,xMin);

    //bookkeeping
    tPrev_ = t;
    xPrev_ = xClip;

    return xClip;
  }

}