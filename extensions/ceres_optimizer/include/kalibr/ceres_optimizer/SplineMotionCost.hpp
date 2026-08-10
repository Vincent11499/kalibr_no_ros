#pragma once

#include <ceres/cost_function.h>

#include <Eigen/Core>

namespace kalibr::ceres_optimizer {

// Exact Ceres representation of one Kalibr BSplineMotionError segment.
// Kalibr's BSpline::segmentIntegral() returns R such that the segment motion
// energy is c.transpose() * R.transpose() * R * c.  Each parameter block is
// one vector-valued spline control vertex and the residual is simply R * c.
class SplineSegmentMotionCost final : public ceres::CostFunction {
 public:
  SplineSegmentMotionCost(Eigen::MatrixXd square_root_information,
                          int parameter_block_size);

  bool Evaluate(double const* const* parameters, double* residuals,
                double** jacobians) const override;

  int parameterBlockSize() const { return parameter_block_size_; }
  int parameterBlockCount() const {
    return static_cast<int>(parameter_block_sizes().size());
  }

 private:
  Eigen::MatrixXd square_root_information_;
  int parameter_block_size_;
};

}  // namespace kalibr::ceres_optimizer
