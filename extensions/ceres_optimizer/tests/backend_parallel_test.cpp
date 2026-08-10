#include "kalibr/ceres_optimizer/Backend.hpp"

#include <ceres/ceres.h>

#include <cmath>
#include <iostream>
#include <mutex>
#include <set>
#include <thread>

namespace {

struct ThreadTracker {
  void record() {
    std::lock_guard<std::mutex> lock(mutex);
    threads.insert(std::this_thread::get_id());
  }

  std::size_t count() const {
    std::lock_guard<std::mutex> lock(mutex);
    return threads.size();
  }

  mutable std::mutex mutex;
  std::set<std::thread::id> threads;
};

struct ScalarResidual {
  ScalarResidual(double target, ThreadTracker* tracker)
      : target(target), tracker(tracker) {}

  template <typename T>
  bool operator()(const T* const value, T* residual) const {
    tracker->record();
    residual[0] = value[0] - T(target);
    return true;
  }

  double target;
  ThreadTracker* tracker;
};

}  // namespace

int main() {
  constexpr int kResidualCount = 16384;
  double value = 10.0;
  ThreadTracker tracker;
  ceres::Problem problem;
  for (int index = 0; index < kResidualCount; ++index) {
    const double target = 1.0 + 0.001 * std::sin(index * 0.01);
    problem.AddResidualBlock(
        new ceres::AutoDiffCostFunction<ScalarResidual, 1, 1>(
            new ScalarResidual(target, &tracker)),
        nullptr, &value);
  }

  kalibr::ceres_optimizer::Options options;
  options.max_iterations = 5;
  options.num_threads = 4;
  const auto summary = kalibr::ceres_optimizer::solve(&problem, options);

  std::cout << summary.brief_report << '\n'
            << "requested_threads=" << options.num_threads
            << " observed_threads=" << tracker.count() << '\n';
  if (!summary.usable || !std::isfinite(summary.final_cost) ||
      summary.final_cost >= summary.initial_cost) {
    std::cerr << "Ceres did not produce a decreasing usable solution\n";
    return 1;
  }
  if (tracker.count() < 2) {
    std::cerr << "Ceres residual evaluation did not use multiple threads\n";
    return 2;
  }
  return 0;
}
