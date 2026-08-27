/******************************************************************************
 * Copyright (C) 2013 by Jerome Maye                                          *
 * jerome.maye@gmail.com                                                      *
 *                                                                            *
 * This program is free software; you can redistribute it and/or modify       *
 * it under the terms of the Lesser GNU General Public License as published by*
 * the Free Software Foundation; either version 3 of the License, or          *
 * (at your option) any later version.                                        *
 *                                                                            *
 * This program is distributed in the hope that it will be useful,            *
 * but WITHOUT ANY WARRANTY; without even the implied warranty of             *
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the              *
 * Lesser GNU General Public License for more details.                        *
 *                                                                            *
 * You should have received a copy of the Lesser GNU General Public License   *
 * along with this program. If not, see <http://www.gnu.org/licenses/>.       *
 ******************************************************************************/

/** \file LinearSolver.cpp
    \brief This file defines the python exports for the LinearSolver class.
  */

#include <boost/shared_ptr.hpp>

#include <algorithm>
#include <limits>
#include <set>
#include <stdexcept>
#include <vector>

#include <numpy_eigen/boost_python_headers.hpp>

#include <sm/PropertyTree.hpp>

#include <aslam/backend/LinearSystemSolver.hpp>
#include <aslam/backend/OptimizationProblemBase.hpp>
#include <aslam/backend/DesignVariable.hpp>
#include <aslam/backend/ErrorTerm.hpp>

#include <aslam/calibration/core/LinearSolverOptions.h>
#include <aslam/calibration/core/LinearSolver.h>

using namespace boost::python;
using namespace aslam::calibration;
using namespace aslam::backend;
using namespace sm;

namespace {

struct ProblemOrderingGuard {
  explicit ProblemOrderingGuard(OptimizationProblemBase& problem) :
      _problem(problem) {
    _designVariables.reserve(problem.numDesignVariables());
    _blockIndices.reserve(problem.numDesignVariables());
    _columnBases.reserve(problem.numDesignVariables());
    for (size_t i = 0; i < problem.numDesignVariables(); ++i) {
      DesignVariable* dv = problem.designVariable(i);
      _designVariables.push_back(dv);
      _blockIndices.push_back(dv->blockIndex());
      _columnBases.push_back(dv->columnBase());
    }
    _errorTerms.reserve(problem.numErrorTerms());
    _rowBases.reserve(problem.numErrorTerms());
    for (size_t i = 0; i < problem.numErrorTerms(); ++i) {
      ErrorTerm* error = problem.errorTerm(i);
      _errorTerms.push_back(error);
      _rowBases.push_back(error->rowBase());
    }
  }

  ~ProblemOrderingGuard() {
    for (size_t i = 0; i < _designVariables.size(); ++i) {
      _designVariables[i]->setBlockIndex(_blockIndices[i]);
      _designVariables[i]->setColumnBase(_columnBases[i]);
    }
    for (size_t i = 0; i < _errorTerms.size(); ++i)
      _errorTerms[i]->setRowBase(_rowBases[i]);
  }

  OptimizationProblemBase& _problem;
  std::vector<DesignVariable*> _designVariables;
  std::vector<int> _blockIndices;
  std::vector<int> _columnBases;
  std::vector<ErrorTerm*> _errorTerms;
  std::vector<size_t> _rowBases;
};

boost::python::dict analyzeObservability(
    const OptimizationProblemBase& constProblem,
    const boost::python::list& selectedDesignVariables,
    size_t numThreads) {
  // IncrementalEstimator intentionally exposes its assembled problem as
  // const.  Analysis temporarily changes only ordering metadata and restores
  // it with ProblemOrderingGuard; parameter values are never updated.
  OptimizationProblemBase& problem =
    const_cast<OptimizationProblemBase&>(constProblem);
  ProblemOrderingGuard guard(problem);
  std::set<DesignVariable*> selected;
  const Py_ssize_t requestedCount = boost::python::len(selectedDesignVariables);
  for (Py_ssize_t i = 0; i < requestedCount; ++i) {
    DesignVariable* dv = boost::python::extract<DesignVariable*>(
      selectedDesignVariables[i]);
    if (!dv)
      throw std::invalid_argument("selected design variable is null");
    if (!selected.insert(dv).second)
      throw std::invalid_argument("selected design variable is duplicated");
  }

  std::set<DesignVariable*> problemVariables;
  std::vector<DesignVariable*> nuisance;
  std::vector<DesignVariable*> calibration;
  std::vector<int> selectedActive;
  selectedActive.reserve(static_cast<size_t>(requestedCount));
  size_t nuisanceDimensions = 0;
  size_t calibrationDimensions = 0;
  for (size_t i = 0; i < problem.numDesignVariables(); ++i) {
    DesignVariable* dv = problem.designVariable(i);
    problemVariables.insert(dv);
    if (!dv->isActive())
      continue;
    if (!selected.count(dv)) {
      nuisance.push_back(dv);
      nuisanceDimensions += dv->minimalDimensions();
    }
  }
  for (Py_ssize_t i = 0; i < requestedCount; ++i) {
    DesignVariable* dv = boost::python::extract<DesignVariable*>(
      selectedDesignVariables[i]);
    if (!problemVariables.count(dv))
      throw std::invalid_argument(
        "selected design variable is not part of the optimization problem");
    selectedActive.push_back(dv->isActive() ? 1 : 0);
    if (dv->isActive()) {
      calibration.push_back(dv);
      calibrationDimensions += dv->minimalDimensions();
    }
  }
  if (calibration.empty())
    throw std::invalid_argument("no active calibration design variables selected");
  if (nuisance.empty())
    throw std::invalid_argument(
      "observability analysis requires at least one nuisance design variable");

  std::vector<DesignVariable*> ordered;
  ordered.reserve(nuisance.size() + calibration.size());
  ordered.insert(ordered.end(), nuisance.begin(), nuisance.end());
  ordered.insert(ordered.end(), calibration.begin(), calibration.end());
  int columnBase = 0;
  for (size_t i = 0; i < ordered.size(); ++i) {
    ordered[i]->setBlockIndex(static_cast<int>(i));
    ordered[i]->setColumnBase(columnBase);
    columnBase += ordered[i]->minimalDimensions();
  }
  for (size_t i = 0; i < problem.numDesignVariables(); ++i) {
    DesignVariable* dv = problem.designVariable(i);
    if (!dv->isActive()) {
      dv->setBlockIndex(-1);
      dv->setColumnBase(-1);
    }
  }

  std::vector<ErrorTerm*> errors;
  std::vector<ErrorTerm::Ptr> equivalentErrorOwners;
  errors.reserve(problem.numErrorTerms());
  size_t expandedQuadraticTerms = 0;
  size_t rowBase = 0;
  for (size_t i = 0; i < problem.numErrorTerms(); ++i) {
    ErrorTerm* error = problem.errorTerm(i);
    std::vector<ErrorTerm::Ptr> replacements;
    if (error->getJacobianEquivalentErrorTerms(replacements)) {
      if (replacements.empty())
        throw std::runtime_error(
          "quadratic error produced no Jacobian-equivalent terms");
      ++expandedQuadraticTerms;
      for (size_t replacementIndex = 0;
           replacementIndex < replacements.size(); ++replacementIndex) {
        if (!replacements[replacementIndex])
          throw std::runtime_error(
            "quadratic error produced a null Jacobian-equivalent term");
        replacements[replacementIndex]->setRowBase(rowBase);
        rowBase += replacements[replacementIndex]->dimension();
        errors.push_back(replacements[replacementIndex].get());
        equivalentErrorOwners.push_back(replacements[replacementIndex]);
      }
    }
    else {
      error->setRowBase(rowBase);
      rowBase += error->dimension();
      errors.push_back(error);
    }
  }

  LinearSolverOptions options;
  options.columnScaling = true;
  // A hard failure must mean a numerical/structural nullspace, not merely a
  // direction that the incremental camera solver would truncate for stable
  // updates.  rankTol() already applies the standard spectrum-size factor;
  // use the native LinearSolverOptions default epsilon for the hard rank.
  options.epsSVD = std::numeric_limits<double>::epsilon();
  options.svdTol = -1.0;
  options.qrTol = -1.0;
  LinearSolver solver(options);
  solver.setMargStartIndex(static_cast<std::ptrdiff_t>(nuisanceDimensions));
  solver.initMatrixStructure(ordered, errors, false);
  solver.evaluateError(std::max<size_t>(1, numThreads), true);
  solver.buildSystem(std::max<size_t>(1, numThreads), true);
  // Run the native solve path so nuisance and calibration columns receive the
  // exact same L2 scaling as a real optimization step.  The resulting update
  // is intentionally discarded: this is a read-only observability analysis
  // and must not alter any design variable.
  Eigen::VectorXd unusedUpdate;
  if (!solver.solveSystem(unusedUpdate))
    throw std::runtime_error("observability linearization failed");

  const Eigen::VectorXd singularValues = solver.getSingularValues();
  // Consume the tolerance calculated by the original incremental solver's
  // rankTol() implementation (sigma_max * epsSVD * spectrum size).  Keeping
  // this as the single source of truth prevents diagnostics from drifting
  // away from the native solver if its rank policy changes later.
  const double tolerance = solver.getSVDTolerance();
  std::ptrdiff_t rank = 0;
  for (Eigen::Index i = 0; i < singularValues.size(); ++i)
    if (singularValues[i] > tolerance)
      ++rank;
  const std::ptrdiff_t deficiency =
    static_cast<std::ptrdiff_t>(calibrationDimensions) - rank;
  Eigen::MatrixXd nullspace(
    static_cast<Eigen::Index>(calibrationDimensions),
    static_cast<Eigen::Index>(std::max<std::ptrdiff_t>(0, deficiency)));
  if (deficiency > 0 && solver.getMatrixV().cols() >= deficiency)
    nullspace = solver.getMatrixV().rightCols(deficiency);
  else
    nullspace.resize(calibrationDimensions, 0);

  // Kalibr's final incremental camera refinement explicitly uses epsSVD=1e-6
  // to suppress weak updates.  Preserve that useful engineering diagnostic,
  // but report it separately: weak observability must not be mislabeled as an
  // exact rank deficiency or cause a seeded calibration to be discarded.
  const double operationalEpsSVD = 1.0e-6;
  const double operationalTolerance = singularValues.size() > 0
    ? singularValues[0] * operationalEpsSVD * singularValues.size()
    : 0.0;
  std::ptrdiff_t operationalRank = 0;
  for (Eigen::Index i = 0; i < singularValues.size(); ++i)
    if (singularValues[i] > operationalTolerance)
      ++operationalRank;
  const std::ptrdiff_t operationalDeficiency =
    static_cast<std::ptrdiff_t>(calibrationDimensions) - operationalRank;
  Eigen::MatrixXd operationalNullspace(
    static_cast<Eigen::Index>(calibrationDimensions),
    static_cast<Eigen::Index>(
      std::max<std::ptrdiff_t>(0, operationalDeficiency)));
  if (operationalDeficiency > 0 &&
      solver.getMatrixV().cols() >= operationalDeficiency)
    operationalNullspace =
      solver.getMatrixV().rightCols(operationalDeficiency);
  else
    operationalNullspace.resize(calibrationDimensions, 0);

  boost::python::list activeFlags;
  for (size_t i = 0; i < selectedActive.size(); ++i)
    activeFlags.append(selectedActive[i] != 0);
  boost::python::dict result;
  result["state"] = "final_relinearized";
  result["uses_m_estimator"] = true;
  result["damping_applied"] = false;
  result["column_scaling"] = "l2";
  result["nuisance_dimensions"] = nuisanceDimensions;
  result["calibration_dimensions"] = calibrationDimensions;
  result["rank"] = rank;
  result["deficiency"] = deficiency;
  result["qr_rank"] = solver.getQRRank();
  result["qr_deficiency"] = solver.getQRRankDeficiency();
  result["qr_tolerance"] = solver.getQRTolerance();
  result["svd_tolerance"] = tolerance;
  result["singular_values"] = singularValues;
  result["nullspace"] = nullspace;
  result["operational_eps_svd"] = operationalEpsSVD;
  result["operational_rank"] = operationalRank;
  result["operational_deficiency"] = operationalDeficiency;
  result["operational_svd_tolerance"] = operationalTolerance;
  result["operational_nullspace"] = operationalNullspace;
  result["selected_active"] = activeFlags;
  result["original_error_terms"] = problem.numErrorTerms();
  result["linearized_error_terms"] = errors.size();
  result["expanded_quadratic_error_terms"] = expandedQuadraticTerms;
  return result;
}

}  // namespace

void exportLinearSolver() {
  /// Export LinearSolverOptions structure
  class_<LinearSolverOptions>("LinearSolverOptions", init<>())
    .def_readwrite("columnScaling", &LinearSolverOptions::columnScaling)
    .def_readwrite("epsNorm", &LinearSolverOptions::epsNorm)
    .def_readwrite("epsSVD", &LinearSolverOptions::epsSVD)
    .def_readwrite("epsQR", &LinearSolverOptions::epsQR)
    .def_readwrite("svdTol", &LinearSolverOptions::svdTol)
    .def_readwrite("qrTol", &LinearSolverOptions::qrTol)
    .def_readwrite("verbose", &LinearSolverOptions::verbose)
    ;

  /// Function for querying the options
  LinearSolverOptions& (LinearSolver::*getOptions)() =
    &LinearSolver::getOptions;

  /// Export LinearSolver class
  class_<LinearSolver, boost::shared_ptr<LinearSolver>,
    bases<LinearSystemSolver>, boost::noncopyable>("LinearSolver",
    init<const LinearSolverOptions&>())
    .def(init<const PropertyTree&>())
    .def("getOptions", getOptions, return_internal_reference<>())
    ;

  def("analyzeObservability", &analyzeObservability,
    (arg("problem"), arg("selected_design_variables"),
      arg("num_threads") = 1));
}
