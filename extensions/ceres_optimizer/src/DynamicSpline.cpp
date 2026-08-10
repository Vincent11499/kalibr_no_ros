#include "kalibr/ceres_optimizer/DynamicSpline.hpp"

namespace kalibr::ceres_optimizer {

DynamicSpline::DynamicSpline(const bsplines::BSpline& spline)
    : order_(spline.splineOrder()), knots_(spline.knots()) {
  if (order_ < 2 || spline.numValidTimeSegments() < 1)
    throw std::invalid_argument("Kalibr spline is not initialized");
  basis_matrices_.reserve(spline.numValidTimeSegments());
  for (int segment = 0; segment < spline.numValidTimeSegments(); ++segment)
    basis_matrices_.push_back(spline.basisMatrix(segment));
}

int DynamicSpline::segmentForTime(double timestamp) const {
  if (timestamp < minimumTime() || timestamp > maximumTime())
    throw std::out_of_range("Spline timestamp is outside its valid interval");
  if (std::abs(timestamp - maximumTime()) < 1.0e-12)
    return static_cast<int>(knots_.size()) - order_ - 1;
  const auto upper = std::upper_bound(knots_.begin(), knots_.end(), timestamp);
  return static_cast<int>(upper - knots_.begin()) - 1;
}

std::pair<int, int> DynamicSpline::candidateControls(
    double minimum_time, double maximum_time) const {
  if (minimum_time > maximum_time)
    throw std::invalid_argument("Spline candidate interval is reversed");
  const int first = segmentForTime(minimum_time) - order_ + 1;
  const int last = segmentForTime(maximum_time);
  return {first, last};
}

int DynamicSpline::derivativeMultiplier(int power, int derivative_order) {
  int result = 1;
  for (int derivative = 0; derivative < derivative_order; ++derivative)
    result *= power - derivative;
  return result;
}

}  // namespace kalibr::ceres_optimizer
