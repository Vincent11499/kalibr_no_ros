#include <aslam/calibration/algorithms/linalg.h>

#include <Eigen/Core>

#include <cmath>
#include <iostream>
#include <stdexcept>

namespace {

void require(bool condition, const char* message) {
  if (!condition) {
    throw std::runtime_error(message);
  }
}

}  // namespace

int main() {
  try {
    const Eigen::VectorXd zero = Eigen::VectorXd::Zero(6);
    require(aslam::calibration::estimateNumericalRank(zero, 0.0) == 0,
            "an all-zero spectrum must have rank zero");
    require(std::isinf(aslam::calibration::svGap(zero, 0)),
            "the rank-zero spectral gap must use the infinity sentinel");

    Eigen::VectorXd mixed(4);
    mixed << 9.0, 2.0, 1.0e-9, 0.0;
    require(aslam::calibration::estimateNumericalRank(mixed, 1.0e-8) == 2,
            "positive singular values above tolerance must retain their rank");
    require(std::abs(aslam::calibration::svGap(mixed, 2) - 2.0e9) < 1.0,
            "the interior spectral gap changed unexpectedly");

    Eigen::VectorXd full(3);
    full << 3.0, 2.0, 1.0;
    require(aslam::calibration::estimateNumericalRank(full, 0.0) == 3,
            "a positive spectrum must remain full rank");
    require(std::isinf(aslam::calibration::svGap(full, 3)),
            "the full-rank spectral gap must use the infinity sentinel");

    std::cout << "Incremental rank-zero test passed\n";
    return 0;
  } catch (const std::exception& error) {
    std::cerr << "Incremental rank-zero test failed: " << error.what()
              << '\n';
    return 1;
  }
}
