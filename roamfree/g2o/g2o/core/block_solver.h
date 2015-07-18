// g2o - General Graph Optimization
// Copyright (C) 2011 R. Kuemmerle, G. Grisetti, W. Burgard
// 
// g2o is free software: you can redistribute it and/or modify
// it under the terms of the GNU Lesser General Public License as published
// by the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
// 
// g2o is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU Lesser General Public License for more details.
// 
// You should have received a copy of the GNU Lesser General Public License
// along with this program.  If not, see <http://www.gnu.org/licenses/>.

#ifndef G2O_BLOCK_SOLVER_H
#define G2O_BLOCK_SOLVER_H
#include <Eigen/Core>
#include "solver.h"
#include "linear_solver.h"
#include "sparse_block_matrix.h"
#include "g2o/config.h"

namespace g2o {
  using namespace Eigen;

  /**
   * \brief traits to summarize the properties of the fixed size optimization problem
   */
  template <int _PoseDim, int _LandmarkDim>
  struct BlockSolverTraits
  {
    static const int PoseDim = _PoseDim;
    static const int LandmarkDim = _LandmarkDim;
    typedef Matrix<double, PoseDim, PoseDim> PoseMatrixType;
    typedef Matrix<double, LandmarkDim, LandmarkDim> LandmarkMatrixType;
    typedef Matrix<double, PoseDim, LandmarkDim> PoseLandmarkMatrixType;
    typedef Matrix<double, PoseDim, 1> PoseVectorType;
    typedef Matrix<double, LandmarkDim, 1> LandmarkVectorType;

    typedef SparseBlockMatrix<PoseMatrixType> PoseHessianType;
    typedef SparseBlockMatrix<LandmarkMatrixType> LandmarkHessianType;
    typedef SparseBlockMatrix<PoseLandmarkMatrixType> PoseLandmarkHessianType;
    typedef LinearSolver<PoseMatrixType> LinearSolverType;
  };

  /**
   * \brief traits to summarize the properties of the dynamic size optimization problem
   */
  template <>
  struct BlockSolverTraits<-1, -1>
  {
    static const int PoseDim = -1;
    static const int LandmarkDim = -1;
    typedef MatrixXd PoseMatrixType;
    typedef MatrixXd LandmarkMatrixType;
    typedef MatrixXd PoseLandmarkMatrixType;
    typedef VectorXd PoseVectorType;
    typedef VectorXd LandmarkVectorType;


    typedef SparseBlockMatrix<PoseMatrixType> PoseHessianType;
    typedef SparseBlockMatrix<LandmarkMatrixType> LandmarkHessianType;
    typedef SparseBlockMatrix<PoseLandmarkMatrixType> PoseLandmarkHessianType;
    typedef LinearSolver<PoseMatrixType> LinearSolverType;
  };

  /**
   * \brief Implementation of a solver operating on the blocks of the Hessian
   */
  template <typename Traits>
  class BlockSolver: public Solver {
    public:

      static const int PoseDim = Traits::PoseDim;
      static const int LandmarkDim = Traits::LandmarkDim;
      typedef typename Traits::PoseMatrixType PoseMatrixType;
      typedef typename Traits::LandmarkMatrixType LandmarkMatrixType; 
      typedef typename Traits::PoseLandmarkMatrixType PoseLandmarkMatrixType;
      typedef typename Traits::PoseVectorType PoseVectorType;
      typedef typename Traits::LandmarkVectorType LandmarkVectorType;

      typedef typename Traits::PoseHessianType PoseHessianType;
      typedef typename Traits::LandmarkHessianType LandmarkHessianType;
      typedef typename Traits::PoseLandmarkHessianType PoseLandmarkHessianType;
      typedef typename Traits::LinearSolverType LinearSolverType;

    public:

      /**
       * allocate a block solver ontop of the underlying linear solver.
       * NOTE: The BlockSolver assumes exclusive access to the linear solver and will therefore free the pointer
       * in its destructor.
       */
      BlockSolver(SparseOptimizer* optimizer, LinearSolverType* linearSolver);
      ~BlockSolver();

      virtual bool init(bool online = false);
      virtual bool buildStructure(bool zeroBlocks = false);
      virtual bool updateStructure(const std::vector<HyperGraph::Vertex*>& vset, const HyperGraph::EdgeSet& edges);
      virtual bool buildSystem();
      virtual bool solve();
      virtual bool computeMarginals();
      virtual bool computeMarginals(SparseBlockMatrix<MatrixXd>& spinv, const std::vector<std::pair<int, int> >& blockIndices);
      virtual bool setLambda(double lambda);
      virtual bool supportsSchur() {return true;}
      virtual bool schur() { return _doSchur;}
      virtual void setSchur(bool s) { _doSchur = s;}

      LinearSolver<PoseMatrixType>* linearSolver() const { return _linearSolver;}

    protected:
      void resize(int* blockPoseIndices, int numPoseBlocks, 
          int* blockLandmarkIndices, int numLandmarkBlocks, int totalDim);

      SparseBlockMatrix<PoseMatrixType>* _Hpp;
      SparseBlockMatrix<LandmarkMatrixType>* _Hll;
      SparseBlockMatrix<PoseLandmarkMatrixType>* _Hpl;

      SparseBlockMatrix<PoseMatrixType>* _Hschur;
      SparseBlockMatrix<LandmarkMatrixType>* _DInvSchur;

      LinearSolver<PoseMatrixType>* _linearSolver;

      bool _doSchur;

      double* _coefficients;
      double* _bschur;

      int _numPoses, _numLandmarks;
      int _sizePoses, _sizeLandmarks;
  };


  //variable size solver
  typedef BlockSolver< BlockSolverTraits<-1, -1> > BlockSolverX;
  // solver for BA/3D SLAM
  typedef BlockSolver< BlockSolverTraits<6, 3> > BlockSolver_6_3;  
  // solver fo BA with scale
  typedef BlockSolver< BlockSolverTraits<7, 3> > BlockSolver_7_3;  
  // 2Dof landmarks 3Dof poses
  typedef BlockSolver< BlockSolverTraits<3, 2> > BlockSolver_3_2;

} // end namespace

#include "block_solver.hpp"


#endif
