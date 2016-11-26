//
// Created by manuelli on 11/15/16.
//

#include "ActuatorDynamics.h"
#include <cmath>
#include <limits>
#include <iostream>

double LARGE_DOUBLE_VAL = 1e6;
using namespace Eigen;
namespace ActuatorDynamicsTools {

  ActuatorDynamics::ActuatorDynamics(int order, double u_max){
    // initialize some variables
    x_ = VectorXd::Zero(order);
    dt_power_vector_ = VectorXd::Zero(order);
    constant_term_ = VectorXd::Zero(order);
    linear_term_ = VectorXd::Zero(order);



    u_max_ = u_max;
    order_ = order;

    A_ = MatrixXd::Zero(order_, order_);
    for (int i = 0; i < order - 1; i++){
      A_(i,i+1) = 1;
    }

    Eigen::MatrixXd A_temp = A_;

    exp_A_vec_.resize(order);

    for (int i = 0; i < order_; i++){
      double factorial = std::tgamma((i+1)+1); // should be (i+1)!;
      exp_A_vec_[i] = 1/factorial*A_temp; // should be 1/(i+1)!*A^(i+1)
      A_temp = A_*A_temp; // raises A to a power
    }

    b_ = VectorXd::Zero(order_);
    for (int i = 0; i < order; i++){
      double factorial = std::tgamma(i + 1); // (order - i)!
      b_(i) = 1/factorial;
    }
  }

  void ActuatorDynamics::processSample(const double &t, const double &tau) {

    // this means it is being called for the first time
    if(!init_){
      x_(0) = tau;
      t_prev_ = t;
      return;
    }

    double dt = t - t_prev_;
    this->compute_dt_power_vector(dt);
    this->computeLinearTerm(dt_power_vector_);
    this->computeConstantTerm(dt_power_vector_);

    // then x(t + dt) = constant_term_*x(t) + linear_term*u

    Eigen::MatrixXd const_term_temp = constant_term_*x_;
    double u = (tau - const_term_temp(0))/(linear_term_(0));

    // this is the new x_, achieved using a constant control input u during one tick
    x_ = constant_term_*x_ + linear_term_*u;
    t_prev_ = dt;

  }


  // computes the matrix exp(A*dt)
  void ActuatorDynamics::computeConstantTerm(const Eigen::VectorXd &dt_power_vec){
    linear_term_.setZero();

    for (int i = 0; i < order_; i++){
      linear_term_ = linear_term_ + exp_A_vec_[i]*dt_power_vector_(i);
    }
  }

  void ActuatorDynamics::computeLinearTerm(const Eigen::VectorXd &dt_power_vec) {
    linear_term_ = b_*dt_power_vector_; // this is pointwise multiplication
  }

  std::vector<double> ActuatorDynamics::getBounds(const double& t){

    std::vector<double> bounds(2);

    // if not initialized just return some large values
    if(!init_){
      bounds[0] = -LARGE_DOUBLE_VAL;
      bounds[1] = LARGE_DOUBLE_VAL;
      return bounds;
    }

    double dt = t - t_prev_;
    std::cout << dt << std::endl;
    this->compute_dt_power_vector(dt);
    this->computeLinearTerm(dt_power_vector_);
    this->computeConstantTerm(dt_power_vector_);

    double lower_bound = (constant_term_*x_ - linear_term_*u_max_)(0); // just look at first entry;
    double upper_bound = (constant_term_*x_ + linear_term_*u_max_)(0);

    bounds[0] = lower_bound;
    bounds[1] = upper_bound;
    return bounds;
  }

  // populates dt_vector_ such that dt_vector_(i) = (dt)^(i+1)
  void ActuatorDynamics::compute_dt_power_vector(const double& dt){
    double temp = dt;
    for (int i = 0; i < order_; i++){
      dt_power_vector_(i) = temp; // dt^(i+1)
      temp = dt*temp;
    }
  }


}