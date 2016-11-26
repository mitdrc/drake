//
// Created by manuelli on 11/15/16.
//

#include "ActuatorDynamics.h"
#include <cmath>
#include <limits>
#include <iostream>

double LARGE_DOUBLE_VAL = 1e6;
using namespace Eigen;


// void printVector(Eigen::VectorXd& vec){
//   for (int i = 0 ; i < vec.length(); i++){
//     std::cout << vec[i] << " "; 
//   }

//   std::cout << std::endl;
// }


namespace ActuatorDynamicsTools {

  ActuatorDynamics::ActuatorDynamics(int order, double u_max){
    // initialize some variables
    x_ = VectorXd::Zero(order);
    dt_power_vector_ = VectorXd::Zero(order+1);
    constant_term_ = VectorXd::Zero(order);
    linear_term_ = VectorXd::Zero(order);
    init_ = false;



    u_max_ = u_max;
    order_ = order;

    A_ = MatrixXd::Zero(order_, order_);
    B_ = VectorXd::Zero(order_);
    for (int i = 0; i < order; i++){
      A_(i,i+1) = 1;
    }

    B_(order_ - 1) = 1;

    MatrixXd A_temp = MatrixXd::Identity(order_ , order_);

    // needs to be this size since A^order = 0 so need to keep track of A^(order-1)
    exp_A_vec_.resize(order);
    for (int i = 0; i < order_; i++){
      exp_A_vec_[i] = A_temp; // should be 1/(i)!*A^(i)
      A_temp = A_*A_temp; // raises A to a power
    }


    factorial_ = VectorXd::Zero(order_ + 1);
    for (int i = 0; i < order_ + 1; i++){
      double factorial = std::tgamma(i + 1); // ((order+1) - i)!
      factorial_[i] = 1/factorial;
    }
  }

  void ActuatorDynamics::processSample(const double &t, const double &tau) {

    // this means it is being called for the first time
    if(!init_){
      x_(0) = tau;
      t_prev_ = t;
      init_ = true;
      std::cout << "state vector is " << x_ << std::endl;
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
    t_prev_ = t;



  }


  // computes the matrix exp(A*dt)
  void ActuatorDynamics::computeConstantTerm(const Eigen::VectorXd &dt_power_vec){
    constant_term_.setZero();

    // we know that A
    for (int i = 0; i < order_; i++){
      constant_term_ = constant_term_ + 1/factorial_[i]*exp_A_vec_[i]*dt_power_vector_(i);
    }
  }

  void ActuatorDynamics::computeLinearTerm(const Eigen::VectorXd &dt_power_vec) {
    linear_term_.setZero();

    for (int i = 0; i < order_; i++){
      linear_term_ = linear_term_ + 1/factorial_[i+1]*exp_A_vec_[i]*dt_power_vector_(i+1);
    }

    linear_term_ = linear_term_*B_;

  }

  std::vector<double> ActuatorDynamics::getBounds(const double& t){

    std::vector<double> bounds(2);

    // if not initialized just return some large values
    if(!init_){
      std::cout << "actuator not initialized, returning just some large bounds \n";
      bounds[0] = -LARGE_DOUBLE_VAL;
      bounds[1] = LARGE_DOUBLE_VAL;
      return bounds;
    }

    double dt = t - t_prev_;
    std::cout << "dt is " << dt << std::endl;
    this->compute_dt_power_vector(dt);
    this->computeLinearTerm(dt_power_vector_);
    this->computeConstantTerm(dt_power_vector_);


    if (true){
      std::cout << "dt_power_vector_ " << dt_power_vector_ << std::endl;
      std::cout << "constant term " << constant_term_ << std::endl;
      std::cout << "linear term " << linear_term_ << std::endl;
    }

    double lower_bound = (constant_term_*x_ - linear_term_*u_max_)(0); // just look at first entry;
    double upper_bound = (constant_term_*x_ + linear_term_*u_max_)(0);

    bounds[0] = lower_bound;
    bounds[1] = upper_bound;
    return bounds;
  }

  // populates dt_vector_ such that dt_vector_(i) = (dt)^(i+1)
  void ActuatorDynamics::compute_dt_power_vector(const double& dt){
    double temp = 1;  
    for (int i = 0; i < order_ + 1; i++){
      dt_power_vector_(i) = temp; // dt^(i+1)
      temp = dt*temp;
    }
  }


}