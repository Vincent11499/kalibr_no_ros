#include "kalibr/ceres_optimizer/SplineMotionCost.hpp"

#include <Eigen/Core>

#include <stdexcept>
#include <utility>

namespace kalibr::ceres_optimizer {

SplineSegmentMotionCost::SplineSegmentMotionCost(
    Eigen::MatrixXd square_root_information, int parameter_block_size)
    : square_root_information_(std::move(square_root_information)),
      parameter_block_size_(parameter_block_size) {
  if (parameter_block_size_ <= 0) {
    throw std::invalid_argument("Spline parameter block size must be positive");
  }
  if (square_root_information_.rows() <= 0 ||
      square_root_information_.cols() <= 0 ||
      square_root_information_.cols() % parameter_block_size_ != 0) {
    throw std::invalid_argument(
        "Spline square-root information has incompatible dimensions");
  }
  if (!square_root_information_.allFinite()) {
    throw std::invalid_argument(
        "Spline square-root information must be finite");
  }

  set_num_residuals(static_cast<int>(square_root_information_.rows()));
  const int block_count =
      static_cast<int>(square_root_information_.cols()) /
      parameter_block_size_;
  mutable_parameter_block_sizes()->assign(block_count, parameter_block_size_);
}

bool SplineSegmentMotionCost::Evaluate(double const* const* parameters,
                                       double* residuals,
                                       double** jacobians) const {
  Eigen::VectorXd coefficients(square_root_information_.cols());
  for (int block = 0; block < parameterBlockCount(); ++block) {
    coefficients.segment(block * parameter_block_size_, parameter_block_size_) =
        Eigen::Map<const Eigen::VectorXd>(parameters[block],
                                         parameter_block_size_);
  }
  Eigen::Map<Eigen::VectorXd>(residuals, num_residuals()) =
      square_root_information_ * coefficients;

  if (jacobians == nullptr) return true;
  for (int block = 0; block < parameterBlockCount(); ++block) {
    if (jacobians[block] == nullptr) continue;
    using RowMajorMatrix =
        Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>;
    Eigen::Map<RowMajorMatrix> jacobian(jacobians[block], num_residuals(),
                                        parameter_block_size_);
    jacobian = square_root_information_.middleCols(
        block * parameter_block_size_, parameter_block_size_);
  }
  return true;
}

}  // namespace kalibr::ceres_optimizer
