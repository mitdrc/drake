//
// Created by manuelli on 11/15/16.
//

#ifndef DRAKE_SUPERBUILD_SIMPLEACTUATORDYNAMICS_H
#define DRAKE_SUPERBUILD_SIMPLEACTUATORDYNAMICS_H

#endif //DRAKE_SUPERBUILD_SIMPLEACTUATORDYNAMICS_H

namespace ActuatorDynamicsTools{
  class SimpleActuatorDynamics{
  public:
    void processSample(const double& t, const double& tau) = 0;
    virtual ~SimpleActuatorDynamics(){};
  };
}