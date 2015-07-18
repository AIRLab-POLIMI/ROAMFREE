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

#ifndef G2O_BASE_VERTEX_H
#define G2O_BASE_VERTEX_H

#include "optimizable_graph.h"
#include "creators.h"
#include "g2o/stuff/macros.h"

#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Cholesky>
#include <stack>

namespace g2o {

  using namespace Eigen;


/**
 * \brief Templatized BaseVertex
 *
 * Templatized BaseVertex
 * D  : minimal dimension of the vertex, e.g., 3 for rotation in 3D
 * T  : internal type to represent the estimate, e.g., Quaternion for rotation in 3D
 * ID : dimension of the internal representaion, defaults to D, e.g., 4 for the Quaternion in 3D
 */
  template <int D, typename T>
  class BaseVertex : public OptimizableGraph::Vertex {
    public:
    typedef T EstimateType;
    typedef std::stack<EstimateType, 
                       std::deque <EstimateType,  Eigen::aligned_allocator<EstimateType> > >
    BackupStackType;

    static const int Dimension = D;           ///< dimension of the estimate (minimal) in the manifold space

    typedef Map<Matrix<double, D, D>, Matrix<double,D,D>::Flags & AlignedBit ? Aligned : Unaligned >  HessianBlockType;

  public:
    BaseVertex();

    virtual const double& hessian(int i, int j) const { assert(i<D && j<D); return _hessian(i,j);}
    virtual double& hessian(int i, int j)  { assert(i<D && j<D); return _hessian(i,j);}
    virtual double hessianDeterminant() const {return _hessian.determinant();}
    virtual double* hessianData() { return const_cast<double*>(_hessian.data());}

    virtual void mapHessianMemory(double* d);

    virtual int copyB(double* b_) const {
      memcpy(b_, _b.data(), Dimension * sizeof(double));
      return Dimension; 
    }

    virtual const double& b(int i) const { assert(i < D); return _b(i);}
    virtual double& b(int i) { assert(i < D); return _b(i);}
    virtual double* bData() { return _b.data();}

    virtual void clearQuadraticForm();

    //! updates the current vertex with the direct solution x += H_ii\b_ii
    //! @returns the determinant of the inverted hessian
    virtual double solveDirect(double lambda=0);

    //! get the i_th element of the estimate
    //virtual double estimate(int i) const {assert(i<InternalDimension); return _estimate[i]; }

    //! return right hand side b of the constructed linear system
    Matrix<double, D, 1>& b() { return _b;}
    const Matrix<double, D, 1>& b() const { return _b;}
    //! return the hessian block associated with the vertex
    HessianBlockType& A() { return _hessian;}
    const HessianBlockType& A() const { return _hessian;}

    virtual void push() { _backup.push(_estimate);}
    virtual void pop() { assert(!_backup.empty()); _estimate = _backup.top(); _backup.pop();}
    virtual void discardTop() { assert(!_backup.empty()); _backup.pop();}
    virtual int stackSize() const {return _backup.size();}

    //! return the current estimate of the vertex
    const EstimateType& estimate() const { return _estimate;}
    EstimateType& estimate() { return _estimate;}
    //! set the estimate for the vertex
    void setEstimate(const EstimateType& et) { _estimate = et;}

    virtual void setUncertainty(double* c);
    virtual double* uncertaintyData() { return _uncertainty.data();}
    //! return the uncertainty, i.e., marginal covariance of the node
    const Matrix<double, D, D>& uncertainty() const { return _uncertainty;}
    //! set the uncertainty of the vertex, i.e., marginal covariance
    void setUncertainty(const Matrix<double, D, D>& uncertainty) { _uncertainty = uncertainty;}

  protected:
    HessianBlockType _hessian;
    Matrix<double, D, 1> _b;
    EstimateType _estimate;
    BackupStackType _backup;
    Matrix<double, D, D> _uncertainty;
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

#include "base_vertex.hpp"

} // end namespace g2o


#endif
