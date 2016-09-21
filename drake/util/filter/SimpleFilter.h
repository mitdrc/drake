//
// Created by manuelli on 9/20/16.
//

#pragma once

namespace FilterTools{
  class SimpleFilter{
  public:
    virtual double processSample(const double & t, const double & x) = 0;
    virtual ~SimpleFilter(){};
  };
}
