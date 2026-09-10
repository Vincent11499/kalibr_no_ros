#include <aslam/backend/BlockCholeskyLinearSystemSolver.hpp>
#include <aslam/backend/ErrorTerm.hpp>
#include <aslam/backend/MEstimatorPolicies.hpp>

#include <Eigen/Core>

#include <atomic>
#include <chrono>
#include <cmath>
#include <cstring>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

namespace {

using aslam::backend::BlockCholeskyLinearSystemSolver;
using aslam::backend::DesignVariable;
using aslam::backend::ErrorTerm;
using aslam::backend::ErrorTermFs;
using aslam::backend::JacobianContainer;
using aslam::backend::SparseBlockMatrix;

class ScalarDesignVariable : public DesignVariable {
 public:
  explicit ScalarDesignVariable(double value)
      : value_(value), previousValue_(value) {
    setActive(true);
  }

  double value() const { return value_; }

 protected:
  int minimalDimensionsImplementation() const override { return 1; }

  void updateImplementation(const double* update, int size) override {
    if (size != 1) throw std::runtime_error("invalid scalar update size");
    previousValue_ = value_;
    value_ += update[0];
  }

  void revertUpdateImplementation() override { value_ = previousValue_; }

  void getParametersImplementation(Eigen::MatrixXd& value) const override {
    value.resize(1, 1);
    value(0, 0) = value_;
  }

  void setParametersImplementation(const Eigen::MatrixXd& value) override {
    if (value.rows() != 1 || value.cols() != 1) {
      throw std::runtime_error("invalid scalar parameter shape");
    }
    previousValue_ = value_;
    value_ = value(0, 0);
  }

 private:
  double value_;
  double previousValue_;
};

class DeterministicError : public ErrorTermFs<2> {
 public:
  DeterministicError(const std::vector<ScalarDesignVariable*>& variables,
                     int errorIndex)
      : variables_(variables) {
    std::vector<DesignVariable*> baseVariables(variables.begin(),
                                               variables.end());
    setDesignVariables(baseVariables);

    jacobians_.reserve(variables_.size());
    Eigen::Vector2d prediction = Eigen::Vector2d::Zero();
    for (size_t i = 0; i < variables_.size(); ++i) {
      Eigen::Vector2d jacobian;
      jacobian << 0.031 * (errorIndex + 1) * (i + 1),
                  -0.047 * (errorIndex + 2) / (i + 1);
      jacobians_.push_back(jacobian);
      prediction += jacobian * variables_[i]->value();
    }
    measurement_ = prediction;
    measurement_[0] += 0.003 * ((errorIndex % 7) - 3);
    measurement_[1] -= 0.004 * ((errorIndex % 5) - 2);

    Eigen::Matrix2d inverseCovariance;
    inverseCovariance << 2.0, 0.15, 0.15, 1.25;
    setInvR(inverseCovariance);
  }

  static void resetConcurrencyObservation() {
    activeEvaluations_.store(0);
    maxActiveEvaluations_.store(0);
  }

  static int maxActiveEvaluations() {
    return maxActiveEvaluations_.load();
  }

  static void setArtificialDelay(bool enabled) {
    artificialDelay_.store(enabled);
  }

 protected:
  double evaluateErrorImplementation() override {
    Eigen::Vector2d prediction = Eigen::Vector2d::Zero();
    for (size_t i = 0; i < variables_.size(); ++i) {
      prediction += jacobians_[i] * variables_[i]->value();
    }
    setError(measurement_ - prediction);
    return evaluateChiSquaredError();
  }

  void evaluateJacobiansImplementation(JacobianContainer& out) const override {
    const int active = activeEvaluations_.fetch_add(1) + 1;
    int observed = maxActiveEvaluations_.load();
    while (active > observed &&
           !maxActiveEvaluations_.compare_exchange_weak(observed, active)) {
    }

    // Make overlap observable in the correctness test without coupling
    // numerical results to scheduling. The larger accumulation test disables it.
    if (artificialDelay_.load()) {
      std::this_thread::sleep_for(std::chrono::microseconds(200));
    }
    for (size_t i = 0; i < variables_.size(); ++i) {
      out.add(variables_[i], -jacobians_[i]);
    }
    activeEvaluations_.fetch_sub(1);
  }

 private:
  std::vector<ScalarDesignVariable*> variables_;
  std::vector<Eigen::Vector2d> jacobians_;
  Eigen::Vector2d measurement_;

  static std::atomic<int> activeEvaluations_;
  static std::atomic<int> maxActiveEvaluations_;
  static std::atomic<bool> artificialDelay_;
};

std::atomic<int> DeterministicError::activeEvaluations_(0);
std::atomic<int> DeterministicError::maxActiveEvaluations_(0);
std::atomic<bool> DeterministicError::artificialDelay_(true);

// Models Kalibr's BSplineMotionError contract: there is no Jacobian form, but
// the error provides a direct quadratic contribution through buildHessian().
class QuadraticDirectError : public ErrorTermFs<1> {
 public:
  QuadraticDirectError(ScalarDesignVariable* variable, double hessian,
                       double rhs)
      : variable_(variable), hessian_(hessian), rhs_(rhs) {
    setDesignVariables(variable_);
  }

  static int directBuildCalls() { return directBuildCalls_.load(); }

 protected:
  double evaluateErrorImplementation() override {
    Eigen::Matrix<double, 1, 1> error;
    error << variable_->value();
    setError(error);
    return evaluateChiSquaredError();
  }

  void evaluateJacobiansImplementation(JacobianContainer&) const override {
    throw aslam::Exception(
        "quadratic direct term must not evaluate Jacobians");
  }

  void buildHessianImplementation(SparseBlockMatrix& hessian,
                                  Eigen::VectorXd& rhs,
                                  bool) override {
    directBuildCalls_.fetch_add(1);
    Eigen::MatrixXd* block =
        hessian.block(variable_->blockIndex(), variable_->blockIndex(), true);
    (*block)(0, 0) += hessian_;
    rhs[hessian.rowBaseOfBlock(variable_->blockIndex())] += rhs_;
  }

 private:
  ScalarDesignVariable* variable_;
  double hessian_;
  double rhs_;
  static std::atomic<int> directBuildCalls_;
};

std::atomic<int> QuadraticDirectError::directBuildCalls_(0);

class ThrowingDirectError : public ErrorTermFs<1> {
 public:
  explicit ThrowingDirectError(ScalarDesignVariable* variable) {
    setDesignVariables(variable);
  }

 protected:
  double evaluateErrorImplementation() override {
    Eigen::Matrix<double, 1, 1> error;
    error.setZero();
    setError(error);
    return evaluateChiSquaredError();
  }

  void evaluateJacobiansImplementation(JacobianContainer&) const override {
    throw aslam::Exception("throwing term must not evaluate Jacobians");
  }

  void buildHessianImplementation(SparseBlockMatrix&, Eigen::VectorXd&,
                                  bool) override {
    throw std::runtime_error("intentional parallel Hessian failure");
  }
};

struct SystemStorage {
  std::vector<std::unique_ptr<ScalarDesignVariable> > ownedVariables;
  std::vector<std::unique_ptr<ErrorTerm> > ownedErrors;
  std::vector<DesignVariable*> variables;
  std::vector<ErrorTerm*> errors;
};

SystemStorage makeSystem(int numberOfErrors = 120,
                         int numberOfVariables = 24,
                         bool includeDirectErrors = true) {
  SystemStorage system;
  for (int i = 0; i < numberOfVariables; ++i) {
    system.ownedVariables.push_back(
        std::unique_ptr<ScalarDesignVariable>(
            new ScalarDesignVariable(0.02 * (i - numberOfVariables / 2))));
    // Exercise the exact native DV-scaled normal-equation path. Deliberately
    // use no unit scaling values.
    system.ownedVariables.back()->setScaling(0.2 + 0.07 * (i % 9));
    system.variables.push_back(system.ownedVariables.back().get());
  }

  size_t rowBase = 0;
  for (int errorIndex = 0; errorIndex < numberOfErrors; ++errorIndex) {
    if (includeDirectErrors && (errorIndex == 17 || errorIndex == 73)) {
      system.ownedErrors.push_back(std::unique_ptr<ErrorTerm>(
          new QuadraticDirectError(
              system.ownedVariables[errorIndex % numberOfVariables].get(),
              0.75 + 0.01 * errorIndex, -0.2 + 0.002 * errorIndex)));
    } else {
      const int variableCount = 1 + errorIndex % 4;
      std::vector<ScalarDesignVariable*> connected;
      for (int i = 0; i < variableCount; ++i) {
        connected.push_back(
            system.ownedVariables[
                (errorIndex + 5 * i) % numberOfVariables].get());
      }
      std::unique_ptr<DeterministicError> error(
          new DeterministicError(connected, errorIndex));
      error->setMEstimatorPolicy(
          boost::shared_ptr<aslam::backend::MEstimator>(
              new aslam::backend::HuberMEstimator(1.7)));
      system.ownedErrors.push_back(std::move(error));
    }
    system.ownedErrors.back()->setRowBase(rowBase);
    rowBase += system.ownedErrors.back()->dimension();
    system.errors.push_back(system.ownedErrors.back().get());
  }
  return system;
}

struct NormalEquations {
  Eigen::MatrixXd hessian;
  Eigen::VectorXd rhs;
};

NormalEquations build(BlockCholeskyLinearSystemSolver& solver,
                      size_t threadCount, bool useMEstimator) {
  solver.buildSystem(threadCount, useMEstimator);
  BlockCholeskyLinearSystemSolver::SparseBlockMatrix sparseHessian;
  solver.copyHessian(sparseHessian);
  NormalEquations result;
  result.hessian = sparseHessian.toDense();
  result.rhs = solver.rhs();
  return result;
}

bool bitwiseEqual(const Eigen::MatrixXd& lhs, const Eigen::MatrixXd& rhs) {
  return lhs.rows() == rhs.rows() && lhs.cols() == rhs.cols() &&
         std::memcmp(lhs.data(), rhs.data(),
                     static_cast<size_t>(lhs.size()) * sizeof(double)) == 0;
}

bool bitwiseEqual(const Eigen::VectorXd& lhs, const Eigen::VectorXd& rhs) {
  return lhs.size() == rhs.size() &&
         std::memcmp(lhs.data(), rhs.data(),
                     static_cast<size_t>(lhs.size()) * sizeof(double)) == 0;
}

void require(bool condition, const char* message) {
  if (!condition) throw std::runtime_error(message);
}

void checkClose(const NormalEquations& reference,
                const NormalEquations& candidate) {
  const double hessianTolerance =
      2e-12 * (1.0 + reference.hessian.cwiseAbs().maxCoeff());
  const double rhsTolerance =
      2e-12 * (1.0 + reference.rhs.cwiseAbs().maxCoeff());
  require((reference.hessian - candidate.hessian).cwiseAbs().maxCoeff() <=
              hessianTolerance,
          "parallel Hessian differs from the native serial Hessian");
  require((reference.rhs - candidate.rhs).cwiseAbs().maxCoeff() <=
              rhsTolerance,
          "parallel RHS differs from the native serial RHS");
}

void runCase(bool useMEstimator) {
  SystemStorage system = makeSystem();
  BlockCholeskyLinearSystemSolver solver;
  solver.initMatrixStructure(system.variables, system.errors, false);
  solver.evaluateError(1, useMEstimator);

  const NormalEquations nativeZero = build(solver, 0, useMEstimator);
  const NormalEquations nativeOne = build(solver, 1, useMEstimator);
  require(bitwiseEqual(nativeZero.hessian, nativeOne.hessian),
          "nThreads=0 and nThreads=1 Hessians are not bitwise identical");
  require(bitwiseEqual(nativeZero.rhs, nativeOne.rhs),
          "nThreads=0 and nThreads=1 RHS vectors are not bitwise identical");

  DeterministicError::resetConcurrencyObservation();
  for (size_t threadCount : std::vector<size_t>{2, 4}) {
    const NormalEquations first = build(solver, threadCount, useMEstimator);
    checkClose(nativeOne, first);
    for (int repetition = 0; repetition < 2; ++repetition) {
      const NormalEquations repeated =
          build(solver, threadCount, useMEstimator);
      require(bitwiseEqual(first.hessian, repeated.hessian),
              "parallel Hessian reduction is not deterministic");
      require(bitwiseEqual(first.rhs, repeated.rhs),
              "parallel RHS reduction is not deterministic");
    }
  }

  require(DeterministicError::maxActiveEvaluations() > 1,
          "Jacobian contributions did not execute concurrently");
  require(QuadraticDirectError::directBuildCalls() > 0,
          "quadratic error did not use its native direct-Hessian path");
}

void runExceptionCase() {
  SystemStorage system;
  for (int i = 0; i < 4; ++i) {
    system.ownedVariables.push_back(std::unique_ptr<ScalarDesignVariable>(
        new ScalarDesignVariable(0.1 * i)));
    system.variables.push_back(system.ownedVariables.back().get());
  }
  size_t rowBase = 0;
  for (int i = 0; i < 4; ++i) {
    if (i == 2) {
      system.ownedErrors.push_back(std::unique_ptr<ErrorTerm>(
          new ThrowingDirectError(system.ownedVariables[i].get())));
    } else {
      system.ownedErrors.push_back(std::unique_ptr<ErrorTerm>(
          new QuadraticDirectError(system.ownedVariables[i].get(),
                                   1.0 + i, -0.25 * i)));
    }
    system.ownedErrors.back()->setRowBase(rowBase);
    rowBase += system.ownedErrors.back()->dimension();
    system.errors.push_back(system.ownedErrors.back().get());
  }

  BlockCholeskyLinearSystemSolver solver;
  solver.initMatrixStructure(system.variables, system.errors, false);
  solver.evaluateError(1, false);
  bool caughtExpectedFailure = false;
  try {
    solver.buildSystem(4, false);
  } catch (const std::runtime_error& error) {
    caughtExpectedFailure =
        std::string(error.what()).find("intentional parallel Hessian failure") !=
        std::string::npos;
  }
  require(caughtExpectedFailure,
          "parallel worker exception was not propagated to the caller");
}

void runAccumulationOrderStressCase() {
  DeterministicError::setArtificialDelay(false);
  SystemStorage system = makeSystem(10000, 128, false);
  BlockCholeskyLinearSystemSolver solver;
  solver.initMatrixStructure(system.variables, system.errors, false);
  solver.evaluateError(4, true);
  const NormalEquations serial = build(solver, 1, true);
  for (size_t threadCount : std::vector<size_t>{2, 4}) {
    const NormalEquations first = build(solver, threadCount, true);
    checkClose(serial, first);
    const NormalEquations repeated = build(solver, threadCount, true);
    require(bitwiseEqual(first.hessian, repeated.hessian),
            "large fixed-chunk Hessian build is not deterministic");
    require(bitwiseEqual(first.rhs, repeated.rhs),
            "large fixed-chunk RHS build is not deterministic");
  }
  DeterministicError::setArtificialDelay(true);
}

}  // namespace

int main() {
  try {
    runCase(false);
    runCase(true);
    runExceptionCase();
    runAccumulationOrderStressCase();
    std::cout << "BlockCholesky deterministic parallel test passed\n";
    return 0;
  } catch (const std::exception& exception) {
    std::cerr << "BlockCholesky deterministic parallel test failed: "
              << exception.what() << '\n';
    return 1;
  }
}
