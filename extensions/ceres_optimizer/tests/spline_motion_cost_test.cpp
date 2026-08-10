#include "kalibr/ceres_optimizer/SplineMotionCost.hpp"

#include <bsplines/BSpline.hpp>

#include <Eigen/Core>

#include <cmath>
#include <iostream>
#include <vector>

int main() {
  constexpr int kOrder = 6;
  constexpr int kDimension = 3;
  bsplines::BSpline spline(kOrder);
  spline.initConstantSpline(0.0, 0.2, 14, Eigen::Vector3d::Zero());

  Eigen::MatrixXd coefficients = spline.coefficients();
  for (int column = 0; column < coefficients.cols(); ++column) {
    coefficients.col(column) << 0.02 * column,
        std::sin(0.3 * column), 0.01 * column * column;
  }
  spline.setCoefficientMatrix(coefficients);

  const Eigen::Matrix3d information =
      (Eigen::Vector3d(2.0, 3.0, 5.0)).asDiagonal();
  double native_energy = 0.0;
  double ceres_energy = 0.0;
  for (int segment = 0; segment < spline.numValidTimeSegments(); ++segment) {
    const Eigen::MatrixXd factor = spline.segmentIntegral(segment, information, 1);
    const Eigen::VectorXd local = spline.segmentCoefficientVector(segment);
    native_energy += local.dot(factor.transpose() * factor * local);

    kalibr::ceres_optimizer::SplineSegmentMotionCost cost(factor, kDimension);
    const Eigen::VectorXi indices =
        spline.segmentVvCoefficientVectorIndices(segment);
    std::vector<const double*> parameters;
    for (int index = 0; index < indices.size(); ++index) {
      parameters.push_back(coefficients.col(indices[index]).data());
    }
    Eigen::VectorXd residuals(cost.num_residuals());
    std::vector<Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic,
                              Eigen::RowMajor>>
        jacobian_storage;
    std::vector<double*> jacobians;
    jacobian_storage.reserve(cost.parameterBlockCount());
    jacobians.reserve(cost.parameterBlockCount());
    for (int block = 0; block < cost.parameterBlockCount(); ++block) {
      jacobian_storage.emplace_back(cost.num_residuals(), kDimension);
      jacobians.push_back(jacobian_storage.back().data());
    }
    if (!cost.Evaluate(parameters.data(), residuals.data(), jacobians.data())) {
      std::cerr << "Spline motion cost evaluation failed\n";
      return 1;
    }
    ceres_energy += residuals.squaredNorm();
    for (int block = 0; block < cost.parameterBlockCount(); ++block) {
      const Eigen::MatrixXd expected =
          factor.middleCols(block * kDimension, kDimension);
      if ((jacobian_storage[block] - expected).norm() > 1.0e-12) {
        std::cerr << "Spline motion Jacobian mismatch\n";
        return 2;
      }
    }
  }

  const double relative_error = std::abs(native_energy - ceres_energy) /
      std::max(1.0, std::abs(native_energy));
  if (!std::isfinite(relative_error) || relative_error > 1.0e-12) {
    std::cerr << "Spline motion energy mismatch: native=" << native_energy
              << " ceres=" << ceres_energy << '\n';
    return 3;
  }
  return 0;
}
