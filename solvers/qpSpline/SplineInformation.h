#ifndef DRAKE_SOLVERS_QPSPLINE_SPLINEINFORMATION_H_
#define DRAKE_SOLVERS_QPSPLINE_SPLINEINFORMATION_H_

#include <vector>
#include "PiecewisePolynomialBase.h"
#include "ValueConstraint.h"
#include "ContinuityConstraint.h"

class SplineInformation : public PiecewisePolynomialBase
{
private:
  std::vector<int> segment_polynomial_orders;
  std::vector<std::vector<ValueConstraint> > value_constraints;
  std::vector<ContinuityConstraint> continuity_constraints;

public:
  SplineInformation(std::vector<int> const & segment_polynomial_orders, std::vector<double> const & segment_times);

  virtual int getSegmentPolynomialOrder(int segment_number) const;

  std::vector<ValueConstraint> const & getValueConstraints(int segment_number) const;

  std::vector<ContinuityConstraint> const & getContinuityConstraints() const;

  void addValueConstraint(int segment_index, ValueConstraint const & constraint);

  int getNumberOfConstraints() const;

  void addContinuityConstraint(ContinuityConstraint const & constraint);

  std::vector<double> const & getSegmentTimes() const;
};

#endif /* DRAKE_SOLVERS_QPSPLINE_SPLINEINFORMATION_H_ */