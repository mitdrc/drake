//
// Created by manuelli on 11/15/16.
//

#ifndef DRAKE_SUPERBUILD_ACTUATORDYNAMICS_H
#define DRAKE_SUPERBUILD_ACTUATORDYNAMICS_H

#include "SimpleActuatorDynamics.h"
#include <Eigen/Dense>
#include <Eigen/StdVector>


namespace ActuatorDynamicsTools {

  class ActuatorDynamics: public SimpleActuatorDynamics {
  private:

    double t_prev_;
    bool init_;
    double u_max_;
    int order_;
    Eigen::MatrixXd A_;
    Eigen::VectorXd B_;
    std::vector<Eigen::MatrixXd> exp_A_vec_; // stores the relevant terms of exp(A) with the factorials
    Eigen::VectorXd factorial_;

    Eigen::VectorXd dt_power_vector_;

    Eigen::VectorXd constant_term_;
    Eigen::VectorXd linear_term_;


    void compute_dt_power_vector(const double& dt);
    void computeLinearTerm(const Eigen::VectorXd & dt_power_vec);
    void computeConstantTerm(const Eigen::VectorXd & dt_power_vec);


  public:
    // HACK for now so that we can inspect it

    Eigen::VectorXd x_; // contains tau and required derivatives
    ActuatorDynamics(){} // default constructor, does nothing
    ActuatorDynamics(int order, double u_max);

    void processSample(const double& t, const double& tau);

    std::vector<double> getBounds(const double& t);
  };


}


#endif //DRAKE_SUPERBUILD_ACTUATORDYNAMICS_H
