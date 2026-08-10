#pragma once

#include <ceres/ceres.h>

#include <string>

namespace kalibr::ceres_optimizer {

enum class ProblemStructure {
  kGeneralSparse,
  kBundleAdjustment,
};

struct Options {
  int max_iterations = 50;
  // Zero selects all logical CPUs, matching the standalone Kalibr policy.
  int num_threads = 0;
  bool verbose = false;
  ProblemStructure structure = ProblemStructure::kGeneralSparse;
};

struct Summary {
  bool usable = false;
  bool converged = false;
  int iterations = 0;
  int num_threads = 1;
  double initial_cost = 0.0;
  double final_cost = 0.0;
  double total_time_seconds = 0.0;
  std::string brief_report;
  std::string full_report;
};

int resolveThreadCount(int requested_threads);
ceres::Solver::Options makeSolverOptions(const Options& options);
Summary solve(ceres::Problem* problem, const Options& options = {});

}  // namespace kalibr::ceres_optimizer
