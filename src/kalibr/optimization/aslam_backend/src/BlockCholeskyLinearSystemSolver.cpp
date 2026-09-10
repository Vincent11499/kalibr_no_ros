#include <aslam/backend/BlockCholeskyLinearSystemSolver.hpp>
#include <sparse_block_matrix/linear_solver_cholmod.h>
#include <sparse_block_matrix/linear_solver_spqr.h>
#include <aslam/backend/ErrorTerm.hpp>
#include <sm/PropertyTree.hpp>
#include <aslam/backend/Profiling.hpp>
#include <boost/thread.hpp>
#include <algorithm>
#include <exception>
#include <memory>
#include <vector>

namespace aslam {
  namespace backend {
  namespace {

    struct NormalEquationChunk {
      typedef sparse_block_matrix::SparseBlockMatrix<Eigen::MatrixXd>
          SparseBlockMatrix;

      NormalEquationChunk(const std::vector<int>& blockIndices,
                          Eigen::Index rhsSize)
          : hessian(blockIndices, blockIndices),
            rhs(Eigen::VectorXd::Zero(rhsSize)) {}

      SparseBlockMatrix hessian;
      Eigen::VectorXd rhs;
    };

    void mergeNormalEquationChunk(
        const NormalEquationChunk& chunk,
        sparse_block_matrix::SparseBlockMatrix<Eigen::MatrixXd>& hessian,
        Eigen::VectorXd& rhs) {
      rhs += chunk.rhs;
      const std::vector<NormalEquationChunk::SparseBlockMatrix::IntBlockMap>&
          blockColumns = chunk.hessian.blockCols();
      for (size_t columnBlock = 0; columnBlock < blockColumns.size();
           ++columnBlock) {
        for (NormalEquationChunk::SparseBlockMatrix::IntBlockMap::const_iterator
                 blockIt = blockColumns[columnBlock].begin();
             blockIt != blockColumns[columnBlock].end(); ++blockIt) {
          Eigen::MatrixXd* destination = hessian.block(
              blockIt->first, static_cast<int>(columnBlock), true);
          SM_ASSERT_TRUE(aslam::Exception, destination != NULL,
                         "The Hessian block is NULL");
          SM_ASSERT_EQ(aslam::Exception, destination->rows(),
                       blockIt->second->rows(),
                       "Unexpected Hessian block row count");
          SM_ASSERT_EQ(aslam::Exception, destination->cols(),
                       blockIt->second->cols(),
                       "Unexpected Hessian block column count");
          *destination += *blockIt->second;
        }
      }
    }

  }  // namespace

  BlockCholeskyLinearSystemSolver::BlockCholeskyLinearSystemSolver(const std::string & solver, const BlockCholeskyLinearSolverOptions& options) :
      _options(options),
      _solverType(solver) {
    initSolver();
  }

    BlockCholeskyLinearSystemSolver::BlockCholeskyLinearSystemSolver(const sm::PropertyTree& config) {
      _solverType = config.getString("solverType", "cholesky");
      // NO OPTIONS CURRENTLY IMPLEMENTED
      // USING C++11 would allow to do constructor delegation and more elegant code
      if(_solverType == "cholesky") {
        _solver.reset(new sparse_block_matrix::LinearSolverCholmod<Eigen::MatrixXd>());
      } else if(_solverType == "spqr") {
        _solver.reset(new sparse_block_matrix::LinearSolverQr<Eigen::MatrixXd>());
      } else {
        std::cout << "Unknown block solver type " << _solverType << ". Try \"cholesky\" or \"spqr\"\nDefaulting to cholesky.\n";
        _solver.reset(new sparse_block_matrix::LinearSolverCholmod<Eigen::MatrixXd>());
      }
    }

    BlockCholeskyLinearSystemSolver::~BlockCholeskyLinearSystemSolver()
    {
    }


    void BlockCholeskyLinearSystemSolver::initMatrixStructureImplementation(const std::vector<DesignVariable*>& dvs, const std::vector<ErrorTerm*>& errors, bool useDiagonalConditioner)
    {
      if(_solverType == "cholesky") {
        _solver.reset(new sparse_block_matrix::LinearSolverCholmod<Eigen::MatrixXd>());
      } else if(_solverType == "spqr") {
        _solver.reset(new sparse_block_matrix::LinearSolverQr<Eigen::MatrixXd>());
      } else {
        std::cout << "Unknown block solver type " << _solverType << ". Try \"cholesky\" or \"spqr\"\nDefaulting to cholesky.\n";
        _solver.reset(new sparse_block_matrix::LinearSolverCholmod<Eigen::MatrixXd>());
      }
      _solver->init();
      _useDiagonalConditioner = useDiagonalConditioner;
      _errorTerms = errors;
      std::vector<int> blocks;
      for (size_t i = 0; i < dvs.size(); ++i) {
        dvs[i]->setBlockIndex(i);
        blocks.push_back(dvs[i]->minimalDimensions());
      }
      std::partial_sum(blocks.begin(), blocks.end(), blocks.begin());
      // Now we can initialized the sparse Hessian matrix.
      _H._M = SparseBlockMatrix(blocks, blocks);
    }


  void BlockCholeskyLinearSystemSolver::buildSystem(size_t nThreads, bool useMEstimator)
    {
      ProfilingTimer timer("BlockCholesky: build Hessian");
      _H._M.clear(false);
      _rhs.setZero();

      const size_t workerCount = std::min(nThreads, _errorTerms.size());
      if (workerCount <= 1) {
        // Preserve the native implementation verbatim for nThreads == 0/1
        // (and for a single error term), including its arithmetic path.
        std::vector<ErrorTerm*>::iterator it, it_end;
        it = _errorTerms.begin();
        it_end = _errorTerms.end();
        for (; it != it_end; ++it) {
          (*it)->buildHessian(_H._M, _rhs, useMEstimator);
        }
        return;
      }

      // Split the original error sequence into fixed contiguous chunks. Each
      // worker calls the unmodified ErrorTerm::buildHessian() path in original
      // order inside its chunk. This amortizes sparse-block allocation and
      // full-length RHS traversal over a whole chunk instead of repeating
      // both operations for every error term.
      const std::vector<int>& blockIndices =
          _H._M.rowBlockIndices();
      const Eigen::Index rhsSize = _rhs.size();
      std::vector<std::unique_ptr<NormalEquationChunk> > chunks;
      chunks.reserve(workerCount);
      for (size_t i = 0; i < workerCount; ++i) {
        chunks.push_back(std::unique_ptr<NormalEquationChunk>(
            new NormalEquationChunk(blockIndices, rhsSize)));
      }
      std::vector<std::exception_ptr> failures(workerCount);

      boost::thread_group workers;
      try {
        for (size_t workerIndex = 0; workerIndex < workerCount;
             ++workerIndex) {
          NormalEquationChunk* chunk = chunks[workerIndex].get();
          const std::vector<ErrorTerm*>* errorTerms = &_errorTerms;
          std::exception_ptr* failure = &failures[workerIndex];
          workers.create_thread(
              [chunk, errorTerms, failure, workerIndex, workerCount,
               useMEstimator]() {
                try {
                  const size_t begin =
                      errorTerms->size() * workerIndex / workerCount;
                  const size_t end =
                      errorTerms->size() * (workerIndex + 1) / workerCount;
                  for (size_t errorIndex = begin; errorIndex < end;
                       ++errorIndex) {
                    (*errorTerms)[errorIndex]->buildHessian(
                        chunk->hessian, chunk->rhs, useMEstimator);
                  }
                } catch (...) {
                  *failure = std::current_exception();
                }
              });
        }
      } catch (...) {
        workers.join_all();
        throw;
      }
      workers.join_all();
      for (size_t chunkIndex = 0; chunkIndex < workerCount;
           ++chunkIndex) {
        if (failures[chunkIndex]) {
          std::rethrow_exception(failures[chunkIndex]);
        }
        mergeNormalEquationChunk(*chunks[chunkIndex], _H._M, _rhs);
      }
    }

    bool BlockCholeskyLinearSystemSolver::solveSystem(Eigen::VectorXd& outDx)
    {
      ProfilingTimer timer("BlockCholesky: factorize and solve");
      if (_useDiagonalConditioner) {
        Eigen::VectorXd d = _diagonalConditioner.cwiseProduct(_diagonalConditioner);
        // Augment the diagonal
        int rowBase = 0;
        for (int i = 0; i < _H._M.bRows(); ++i) {
          Eigen::MatrixXd& block = *_H._M.block(i, i, true);
          SM_ASSERT_EQ_DBG(Exception, block.rows(), block.cols(), "Diagonal blocks are square...right?");
          block.diagonal() += d.segment(rowBase, block.rows());
          rowBase += block.rows();
        }
      }
      // Solve the system
      outDx.resize(_H._M.rows());
      bool solutionSuccess = _solver->solve(_H._M, &outDx[0], &_rhs[0]);
      if (_useDiagonalConditioner) {
        // Un-augment the diagonal
        int rowBase = 0;
        for (int i = 0; i < _H._M.bRows(); ++i) {
          Eigen::MatrixXd& block = *_H._M.block(i, i, true);
          block.diagonal() -= _diagonalConditioner.segment(rowBase, block.rows());
          rowBase += block.rows();
        }
      }
      if( ! solutionSuccess ) {
        //std::cout << "Solution failed...creating a new solver\n";
        // This seems to help when the CHOLMOD stuff gets into a bad state
        initSolver();
      }
      
      return solutionSuccess;
    }


  void BlockCholeskyLinearSystemSolver::initSolver() {
      if(_solverType == "cholesky") {
        _solver.reset(new sparse_block_matrix::LinearSolverCholmod<Eigen::MatrixXd>());
      } else if(_solverType == "spqr") {
        _solver.reset(new sparse_block_matrix::LinearSolverQr<Eigen::MatrixXd>());
      } else {
        std::cout << "Unknown block solver type " << _solverType << ". Try \"cholesky\" or \"spqr\"\nDefaulting to cholesky.\n";
        _solver.reset(new sparse_block_matrix::LinearSolverCholmod<Eigen::MatrixXd>());
      }

  }

    /// \brief compute only the covariance blocks associated with the block indices passed as an argument
    void BlockCholeskyLinearSystemSolver::computeCovarianceBlocks(const std::vector<std::pair<int, int> >& blockIndices, SparseBlockMatrix& outP)
    {
      // Not sure why I have to do this.
      //_solver->init();
      if (_useDiagonalConditioner) {
        Eigen::VectorXd d = _diagonalConditioner.cwiseProduct(_diagonalConditioner);
        // Augment the diagonal
        int rowBase = 0;
        for (int i = 0; i < _H._M.bRows(); ++i) {
          Eigen::MatrixXd& block = *_H._M.block(i, i, true);
          SM_ASSERT_EQ_DBG(Exception, block.rows(), block.cols(), "Diagonal blocks are square...right?");
          block.diagonal() += d.segment(rowBase, block.rows());
          rowBase += block.rows();
        }
      }
      bool success = _solver->solvePattern(outP, blockIndices, _H._M);
      SM_ASSERT_TRUE(Exception, success, "Unable to retrieve covariance");
      if (_useDiagonalConditioner) {
        // Un-augment the diagonal
        int rowBase = 0;
        for (int i = 0; i < _H._M.bRows(); ++i) {
          Eigen::MatrixXd& block = *_H._M.block(i, i, true);
          block.diagonal() -= _diagonalConditioner.segment(rowBase, block.rows());
          rowBase += block.rows();
        }
      }
    }

    void BlockCholeskyLinearSystemSolver::copyHessian(SparseBlockMatrix& H)
    {
      _H._M.cloneInto(H);
    }

    const BlockCholeskyLinearSolverOptions&
    BlockCholeskyLinearSystemSolver::getOptions() const {
      return _options;
    }

    BlockCholeskyLinearSolverOptions&
    BlockCholeskyLinearSystemSolver::getOptions() {
      return _options;
    }

    void BlockCholeskyLinearSystemSolver::setOptions(
        const BlockCholeskyLinearSolverOptions& options) {
      _options = options;
    }
      
      
      
    double BlockCholeskyLinearSystemSolver::rhsJtJrhs() {
        Eigen::VectorXd JtJrhs;
        _H.rightMultiply(_rhs, JtJrhs);
        return _rhs.dot(JtJrhs);
    }
      

  } // namespace backend
} // namespace aslam
