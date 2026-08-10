#include "kalibr/ceres_optimizer/Backend.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <thread>

namespace kalibr::ceres_optimizer {

int resolveThreadCount(int requested_threads) {
  if (requested_threads < 0) {
    throw std::invalid_argument("Ceres thread count must not be negative");
  }
  if (requested_threads > 0) return requested_threads;
  return static_cast<int>(std::max(1u, std::thread::hardware_concurrency()));
}

ceres::Solver::Options makeSolverOptions(const Options& options) {
  if (options.max_iterations < 0) {
    throw std::invalid_argument("Ceres iteration count must not be negative");
  }

  ceres::Solver::Options result;
  result.max_num_iterations = options.max_iterations;
  result.num_threads = resolveThreadCount(options.num_threads);
  result.trust_region_strategy_type = ceres::LEVENBERG_MARQUARDT;
  result.linear_solver_type =
      options.structure == ProblemStructure::kBundleAdjustment
          ? ceres::SPARSE_SCHUR
          : ceres::SPARSE_NORMAL_CHOLESKY;
  result.sparse_linear_algebra_library_type = ceres::SUITE_SPARSE;
  result.minimizer_progress_to_stdout = options.verbose;
  return result;
}

Summary solve(ceres::Problem* problem, const Options& options) {
  if (problem == nullptr) {
    throw std::invalid_argument("Ceres problem must not be null");
  }

  const ceres::Solver::Options solver_options = makeSolverOptions(options);
  ceres::Solver::Summary ceres_summary;
  ceres::Solve(solver_options, problem, &ceres_summary);

  Summary result;
  result.usable = ceres_summary.IsSolutionUsable() &&
                  std::isfinite(ceres_summary.initial_cost) &&
                  std::isfinite(ceres_summary.final_cost);
  result.converged = result.usable &&
                     ceres_summary.termination_type == ceres::CONVERGENCE;
  result.iterations =
      std::max(0, static_cast<int>(ceres_summary.iterations.size()) - 1);
  result.num_threads = solver_options.num_threads;
  result.initial_cost = ceres_summary.initial_cost;
  result.final_cost = ceres_summary.final_cost;
  result.total_time_seconds = ceres_summary.total_time_in_seconds;
  result.brief_report = ceres_summary.BriefReport();
  result.full_report = ceres_summary.FullReport();
  return result;
}

}  // namespace kalibr::ceres_optimizer
