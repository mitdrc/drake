#include "ExponentialPlusPiecewisePolynomial.h"
#include <Eigen/Core>
#include <random>
#include <iostream>
#include <cmath>
#include "testUtil.h"
#include "trajectoryTestUtil.h"

using namespace std;
using namespace Eigen;

default_random_engine generator;

template <typename CoefficientType>
void testSimpleCase() {
  typedef ExponentialPlusPiecewisePolynomial<CoefficientType> ExponentialPlusPiecewisePolynomialType;
  typedef typename ExponentialPlusPiecewisePolynomialType::MatrixX MatrixX;
  typedef typename ExponentialPlusPiecewisePolynomialType::VectorX VectorX;
  int num_coefficients = 5;
  int num_segments = 1;

  MatrixX K = MatrixX::Random(1, 1);
  MatrixX A = MatrixX::Random(1, 1);
  vector<VectorX> alpha;
  VectorX alpha0 = VectorX::Random(1);
  alpha.push_back(alpha0);

  auto segment_times = generateRandomSegmentTimes(num_segments, generator);
  auto polynomial_part = generateRandomPiecewisePolynomial<CoefficientType>(1, 1, num_coefficients, segment_times);

  ExponentialPlusPiecewisePolynomial<CoefficientType> expPlusPp(K, A, alpha, polynomial_part);

  uniform_real_distribution<CoefficientType> uniform(expPlusPp.getStartTime(), expPlusPp.getEndTime());
  double t = uniform(generator);
  auto check = K(0) * std::exp(A(0) * (t - expPlusPp.getStartTime())) * alpha0(0) + polynomial_part.value(t);

  valuecheck(check, expPlusPp.value(t)(0), 1e-8);
}

int main(int argc, char **argv) {
  testSimpleCase<double>();
  std::cout << "test passed";
  return 0;
}