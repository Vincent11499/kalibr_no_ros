#include <aslam/backend/JacobianContainer.hpp>
#include <sm/assert_macros.hpp>
#include <vector>

namespace aslam {
  namespace backend {



    JacobianContainer::JacobianContainer(int rows) : _rows(rows)
    {
    }

    JacobianContainer::~JacobianContainer()
    {
    }



    /// \brief Add the rhs container to this one.
    void JacobianContainer::add(const JacobianContainer& rhs)
    {
      SM_ASSERT_EQ(Exception, _rows, rhs._rows, "The JacobianContainers cannot be added. They don't have the same number of rows.");
      // Merge the two maps.
      // They are sorted by block it->first->blockIndex() so we can be smart about this.
      map_t::iterator lt = _jacobianMap.begin();
      const map_t::iterator lt_end = _jacobianMap.end();
      map_t::const_iterator rt = rhs._jacobianMap.begin();
      map_t::const_iterator rt_end = rhs._jacobianMap.end();
      bool done = rt == rt_end;
      for (; lt != lt_end && !done; ++lt) {
        // The maps are sorted by block index. FFWD the rhs map
        // inserting elements until we find the next possible
        // equal element.
        while (!done && rt->first->blockIndex() < lt->first->blockIndex()) {
          _jacobianMap.insert(lt, *rt);
          ++rt;
          done = (rt == rt_end);
        }
        // If these keys match the Jacobians add
        if (!done && rt->first->blockIndex() == lt->first->blockIndex()) {
          SM_ASSERT_TRUE_DBG(Exception, rt->first == lt->first, "Two design variables had the same block index but different pointer values");
          // add the Jacobians.
          lt->second += rt->second;
        }
      }
      // Now the lhs list is done...add the remaining elements of the rhs list.
      for (; rt != rt_end; rt++) {
        _jacobianMap.insert(lt, *rt);
      }
    }

    /// \brief how many design variables does this jacobian container represent.
    size_t JacobianContainer::numDesignVariables() const
    {
      return _jacobianMap.size();
    }



    /// \brief How many rows does this set of Jacobians have?
    int JacobianContainer::rows() const
    {
      return _rows;
    }

    JacobianContainer::map_t::const_iterator JacobianContainer::begin() const
    {
      return _jacobianMap.begin();
    }

    JacobianContainer::map_t::const_iterator JacobianContainer::end() const
    {
      return _jacobianMap.end();
    }

    JacobianContainer::map_t::iterator JacobianContainer::begin()
    {
      return _jacobianMap.begin();
    }

    JacobianContainer::map_t::iterator JacobianContainer::end()
    {
      return _jacobianMap.end();
    }


    /// \brief Apply the chain rule to the set of Jacobians.
    /// This may change the number of rows of this set of Jacobians
    /// by multiplying through by df_dx on the left.
    void JacobianContainer::applyChainRule(const Eigen::MatrixXd& df_dx)
    {
      SM_ASSERT_EQ(Exception, df_dx.cols(), _rows, "Invalid matrix multiplication");
      map_t::iterator it = _jacobianMap.begin(),
                      it_end = _jacobianMap.end();
      for (; it != it_end; ++it)
        it->second = df_dx * it->second;
      _rows = df_dx.rows();
    }




    void JacobianContainer::buildCorrelatedHessianBlock(const Eigen::VectorXd& e,
                                                        const Eigen::MatrixXd& J1, int j1_block,
                                                        SparseBlockMatrix& outHessian,
                                                        map_t::const_iterator it, map_t::const_iterator it_end) const
    {
      // Recursion base case.
      if (it == it_end)
        return;
      int j2_block = it->first->blockIndex();
      SM_ASSERT_LE_DBG(Exception, j1_block, j2_block, "The recursion should proceed in ordered blocks in order to only populate the upper diagonal of the Hessian. This violates the ordering.");
      const bool allocateIfMissing = true;
      Eigen::MatrixXd* J1t_invR_J2 = outHessian.block(j1_block, j2_block, allocateIfMissing);
      SM_ASSERT_TRUE_DBG(Exception, J1t_invR_J2 != NULL, "The Hessian block is NULL");
      SM_ASSERT_EQ_DBG(Exception, J1t_invR_J2->rows(), J1.cols(),
                       "The Hessian block has an unexpected number of rows. Block J1^T invR J2: (" <<
                       j1_block << ", " << j2_block << "). J1 is: " <<
                       J1.cols() << "x" << J1.rows() <<  ", J2 is: " <<
                       it->second.rows() << "x" << it->second.cols());
      SM_ASSERT_EQ_DBG(Exception, J1t_invR_J2->cols(), it->second.cols(),
                       "The Hessian block has an unexpected number of rows. Block J1^T invR J2: (" <<
                       j1_block << ", " << j2_block << "). J1 is: " <<
                       J1.cols() << "x" << J1.rows() <<  ", J2 is: " <<
                       it->second.rows() << "x" << it->second.cols());
      *J1t_invR_J2 += J1.transpose() * it->second;
      // Tail recursion
      buildCorrelatedHessianBlock(e, J1, j1_block, outHessian, ++it, it_end);
    }


    void JacobianContainer::buildHessianBlock(const Eigen::VectorXd& e,
                                              SparseBlockMatrix& outHessian,
                                              Eigen::VectorXd& outRhs,
                                              map_t::const_iterator it, map_t::const_iterator it_end) const
    {
      // Recursion base case.
      if (it == it_end)
        return;
      const int j1_block = it->first->blockIndex();
      // \todo Make this a debug assert.
      SM_ASSERT_NE_DBG(Exception, j1_block, -1, "Negative blocks shouldn't make it in here");
      // templated version: outRhs.segment< J1::ColsAtCompileTime >.(j1_block);
      outRhs.segment(outHessian.rowBaseOfBlock(j1_block), it->second.cols()) -= it->second.transpose() * e;
      // Recurse in to the list to build correlated blocks
      buildCorrelatedHessianBlock(e, it->second, j1_block, outHessian, it, it_end);
      // Tail recursion to traverse the list
      buildHessianBlock(e, outHessian, outRhs, ++it, it_end);
    }

    void JacobianContainer::evaluateHessian(const Eigen::VectorXd& e, const Eigen::MatrixXd& sqrtInvR, SparseBlockMatrix& outHessian, Eigen::VectorXd& outRhs) const
    {
      SM_ASSERT_EQ_DBG(Exception, e.size(), _rows, "The error and this Jacobian container should have the same size");
      SM_ASSERT_EQ_DBG(Exception, e.size(), sqrtInvR.rows(), "The error and the covariance matrix don't have compatible sizes");

      // Preserve Kalibr's block ordering and arithmetic order while avoiding a
      // deep std::map copy (and one map-node allocation per design variable)
      // for every error term on every iteration.
      std::vector<DesignVariable*> designVariables;
      std::vector<Eigen::MatrixXd> weightedJacobians;
      designVariables.reserve(_jacobianMap.size());
      weightedJacobians.reserve(_jacobianMap.size());

      const Eigen::MatrixXd sqrtInvRTranspose = sqrtInvR.transpose();
      for (map_t::const_iterator it = _jacobianMap.begin();
           it != _jacobianMap.end(); ++it) {
        designVariables.push_back(it->first);
        weightedJacobians.push_back(
            (it->first->scaling() * sqrtInvRTranspose * it->second).eval());
      }
      const Eigen::VectorXd weightedError = sqrtInvRTranspose * e;

      for (size_t i = 0; i < weightedJacobians.size(); ++i) {
        const int rowBlock = designVariables[i]->blockIndex();
        const Eigen::MatrixXd& Ji = weightedJacobians[i];
        SM_ASSERT_NE_DBG(Exception, rowBlock, -1,
                         "Negative blocks shouldn't make it in here");
        outRhs.segment(outHessian.rowBaseOfBlock(rowBlock), Ji.cols()) -=
            Ji.transpose() * weightedError;

        for (size_t j = i; j < weightedJacobians.size(); ++j) {
          const int colBlock = designVariables[j]->blockIndex();
          SM_ASSERT_LE_DBG(Exception, rowBlock, colBlock,
                           "Jacobian blocks must remain ordered");
          Eigen::MatrixXd* hessianBlock =
              outHessian.block(rowBlock, colBlock, true);
          SM_ASSERT_TRUE_DBG(Exception, hessianBlock != NULL,
                             "The Hessian block is NULL");
          SM_ASSERT_EQ_DBG(Exception, hessianBlock->rows(), Ji.cols(),
                           "Unexpected Hessian block row count");
          SM_ASSERT_EQ_DBG(Exception, hessianBlock->cols(),
                           weightedJacobians[j].cols(),
                           "Unexpected Hessian block column count");
          *hessianBlock += Ji.transpose() * weightedJacobians[j];
        }
      }
    }

    const Eigen::MatrixXd& JacobianContainer::Jacobian(const DesignVariable* dv) const
    {
      map_t::const_iterator it = _jacobianMap.find(const_cast<DesignVariable* >(dv));
      SM_ASSERT_TRUE(Exception, it != _jacobianMap.end(), "The design variable does not exist in the container");
      return it->second;
    }

    /// \brief Get design variable i.
    DesignVariable* JacobianContainer::designVariable(size_t i)
    {
      SM_ASSERT_LT(Exception, i, numDesignVariables(), "Index out of range");
      map_t::iterator it = _jacobianMap.begin();
      for (size_t ii = 0; ii < i; ++ii)
        it++;
      return it->first;
    }

    /// \brief Get design variable i.
    const DesignVariable* JacobianContainer::designVariable(size_t i) const
    {
      SM_ASSERT_LT(Exception, i, numDesignVariables(), "Index out of range");
      map_t::const_iterator it = _jacobianMap.begin();
      for (size_t ii = 0; ii < i; ++ii)
        it++;
      return it->first;
    }

  void JacobianContainer::reset(int rows) {
    clear();
    _rows = rows;
  }
  
    /// \brief Clear the contents of this container
    void JacobianContainer::clear()
    {
      _jacobianMap.clear();
    }

    Eigen::MatrixXd JacobianContainer::asDenseMatrix() const
    {
      // \todo make efficient
      return asSparseMatrix().toDense();
    }

    Eigen::MatrixXd JacobianContainer::asDenseMatrix(const std::vector<int>& colBlockIndices) const
    {
      // \todo make efficient...however, these are only for debugging and unit testing.
      return asSparseMatrix(colBlockIndices).toDense();
    }

    SparseBlockMatrix JacobianContainer::asSparseMatrix(const std::vector<int>& colBlockIndices) const
    {
      // \todo error checking.
      std::vector<int> rows(1);
      rows[0] = _rows;
      /// Step 2: fill the Jacobian
      SparseBlockMatrix J(rows, colBlockIndices, true);
      map_t::const_iterator it = _jacobianMap.begin();
      for (int col = 0; it != _jacobianMap.end(); ++it, col++) {
        const bool allocateBlock = true;
        SM_ASSERT_GE_LT_DBG(aslam::IndexOutOfBoundsException, (size_t)it->first->blockIndex(), 0, colBlockIndices.size(), "Block index is out of bounds");
        Eigen::MatrixXd& Ji = *J.block(0, it->first->blockIndex(), allocateBlock);
        Ji = it->second;
      }
      return J;
    }

    SparseBlockMatrix JacobianContainer::asSparseMatrix() const
    {
      /// Step 1: Determine the block structure of the Jacobian.
      std::vector<int> rows(1);
      rows[0] = _rows;
      std::vector<int> cols(numDesignVariables());
      int sum = 0;
      map_t::const_iterator it = _jacobianMap.begin();
      for (int i = 0 ; it != _jacobianMap.end(); ++it, ++i) {
        sum += it->first->minimalDimensions();
        cols[i] = sum;
      }
      /// Step 2: fill the Jacobian
      SparseBlockMatrix J(rows, cols, true);
      it = _jacobianMap.begin();
      for (int col = 0; it != _jacobianMap.end(); ++it, col++) {
        const bool allocateBlock = true;
        Eigen::MatrixXd& Ji = *J.block(0, col, allocateBlock);
        Ji = it->second;
      }
      return J;
    }


    int JacobianContainer::cols() const
    {
      int sum = 0;
      map_t::const_iterator it = _jacobianMap.begin();
      for (int i = 0 ; it != _jacobianMap.end(); ++it, ++i) {
        sum += it->first->minimalDimensions();
      }
      return sum;
    }
  } // namespace backend
} // namespace aslam
