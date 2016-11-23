//
// Created by manuelli on 11/15/16.
//

#ifndef DRAKE_SUPERBUILD_ACTUATORDYNAMICS_H
#define DRAKE_SUPERBUILD_ACTUATORDYNAMICS_H

namespace ActuatorDynamicsTools {

  class ActuatorDynamics: public SimpleActuatorDynamics {
  private:
    Eigen::VectorXd x_; // contains tau and required derivatives
    double t_prev_;
    bool init_;
    double u_max_;


  public:
    ActuatorDynamics(int order, double u_max){
      x_ = Eigen::VectorXd::Zero(order);
      u_max_ = u_max;
    }

    void processSample(const double& t, const double& tau){
      if (!init_){
        t_prev = t;
        x(0) = tau;
        return;
      }
      
    }
  };


}


#endif //DRAKE_SUPERBUILD_ACTUATORDYNAMICS_H
