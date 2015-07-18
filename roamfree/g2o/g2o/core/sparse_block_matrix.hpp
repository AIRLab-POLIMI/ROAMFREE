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

#include <assert.h>
#include "sparse_block_matrix.h"

namespace g2o {
using namespace Eigen;

namespace {
struct TripletEntry {
  int r, c;
  double x;
  TripletEntry(int r_, int c_, double x_) :
      r(r_), c(c_), x(x_) {
  }
};
struct TripletColSort {
  bool operator()(const TripletEntry& e1, const TripletEntry& e2) const {
    return e1.c < e2.c || (e1.c == e2.c && e1.r < e2.r);
  }
};
}

template<class MatrixType>
SparseBlockMatrix<MatrixType>::SparseBlockMatrix(const int * rbi,
    const int* cbi, int rb, int cb, bool hasStorage) :
    _rowBlockIndices(rbi, rbi + rb), _colBlockIndices(cbi, cbi + cb), _blockCols(
        cb), _hasStorage(hasStorage) {
}

template<class MatrixType>
SparseBlockMatrix<MatrixType>::SparseBlockMatrix() :
    _blockCols(0), _hasStorage(true) {
}

template<class MatrixType>
void SparseBlockMatrix<MatrixType>::clear(bool dealloc) {
#   ifdef G2O_OPENMP
#   pragma omp parallel for default (shared) if (_blockCols.size() > 100)
#   endif
  for (int i = 0; i < static_cast<int>(_blockCols.size()); ++i) {
    for (typename SparseBlockMatrix<MatrixType>::IntBlockMap::const_iterator it =
        _blockCols[i].begin(); it != _blockCols[i].end(); it++) {
      typename SparseBlockMatrix<MatrixType>::SparseMatrixBlock* b = it->second;
      if (_hasStorage && dealloc)
        delete b;
      else
        b->setZero();
    }
    if (_hasStorage && dealloc)
      _blockCols[i].clear();
  }
}

template<class MatrixType>
SparseBlockMatrix<MatrixType>::~SparseBlockMatrix() {
  if (_hasStorage)
    clear(true);
}

template<class MatrixType>
typename SparseBlockMatrix<MatrixType>::SparseMatrixBlock* SparseBlockMatrix<
    MatrixType>::block(int r, int c, bool alloc) {
  typename SparseBlockMatrix<MatrixType>::IntBlockMap::iterator it =
      _blockCols[c].find(r);
  typename SparseBlockMatrix<MatrixType>::SparseMatrixBlock* _block = 0;
  if (it == _blockCols[c].end()) {
    if (!_hasStorage && !alloc)
      return 0;
    else {
      int rb = rowsOfBlock(r);
      int cb = colsOfBlock(c);
      _block = new typename SparseBlockMatrix<MatrixType>::SparseMatrixBlock(rb,
          cb);
      _block->setZero();
      std::pair<typename SparseBlockMatrix<MatrixType>::IntBlockMap::iterator,
          bool> result = _blockCols[c].insert(std::make_pair(r, _block));
      (void) result;
      assert(result.second);
    }
  } else {
    _block = it->second;
  }
  return _block;
}

template<class MatrixType>
const typename SparseBlockMatrix<MatrixType>::SparseMatrixBlock* SparseBlockMatrix<
    MatrixType>::block(int r, int c) const {
  typename SparseBlockMatrix<MatrixType>::IntBlockMap::const_iterator it =
      _blockCols[c].find(r);
  if (it == _blockCols[c].end())
    return 0;
  return it->second;
}

template<class MatrixType>
SparseBlockMatrix<MatrixType>* SparseBlockMatrix<MatrixType>::clone() const {
  SparseBlockMatrix* ret = new SparseBlockMatrix(&_rowBlockIndices[0],
      &_colBlockIndices[0], _rowBlockIndices.size(), _colBlockIndices.size());
  for (size_t i = 0; i < _blockCols.size(); i++) {
    for (typename SparseBlockMatrix<MatrixType>::IntBlockMap::const_iterator it =
        _blockCols[i].begin(); it != _blockCols[i].end(); it++) {
      typename SparseBlockMatrix<MatrixType>::SparseMatrixBlock* b =
          new typename SparseBlockMatrix<MatrixType>::SparseMatrixBlock(
              *it->second);
      ret->_blockCols[i].insert(std::make_pair(it->first, b));
    }
  }
  ret->_hasStorage = true;
  return ret;
}

template<class MatrixType>
template<class MatrixTransposedType>
bool SparseBlockMatrix<MatrixType>::transpose(
    SparseBlockMatrix<MatrixTransposedType>*& dest) const {
  if (!dest) {
    dest = new SparseBlockMatrix<MatrixTransposedType>(&_colBlockIndices[0],
        &_rowBlockIndices[0], _colBlockIndices.size(), _rowBlockIndices.size());
  } else {
    if (!dest->_hasStorage)
      return false;
    if (_rowBlockIndices.size() != dest->_colBlockIndices.size())
      return false;
    if (_colBlockIndices.size() != dest->_rowBlockIndices.size())
      return false;
    for (size_t i = 0; i < _rowBlockIndices.size(); i++) {
      if (_rowBlockIndices[i] != dest->_colBlockIndices[i])
        return false;
    }
    for (size_t i = 0; i < _colBlockIndices.size(); i++) {
      if (_colBlockIndices[i] != dest->_rowBlockIndices[i])
        return false;
    }
  }

  for (size_t i = 0; i < _blockCols.size(); i++) {
    for (typename SparseBlockMatrix<MatrixType>::IntBlockMap::const_iterator it =
        _blockCols[i].begin(); it != _blockCols[i].end(); it++) {
      typename SparseBlockMatrix<MatrixType>::SparseMatrixBlock* s = it->second;
      typename SparseBlockMatrix<MatrixTransposedType>::SparseMatrixBlock* d =
          dest->block(i, it->first, true);
      *d = s->transpose();
    }
  }
  return true;
}

template<class MatrixType>
bool SparseBlockMatrix<MatrixType>::add(SparseBlockMatrix*& dest) const {
  if (!dest) {
    dest = new SparseBlockMatrix(&_rowBlockIndices[0], &_colBlockIndices[0],
        _rowBlockIndices.size(), _colBlockIndices.size());
  } else {
    if (!dest->_hasStorage)
      return false;
    if (_rowBlockIndices.size() != dest->_rowBlockIndices.size())
      return false;
    if (_colBlockIndices.size() != dest->_colBlockIndices.size())
      return false;
    for (size_t i = 0; i < _rowBlockIndices.size(); i++) {
      if (_rowBlockIndices[i] != dest->_rowBlockIndices[i])
        return false;
    }
    for (size_t i = 0; i < _colBlockIndices.size(); i++) {
      if (_colBlockIndices[i] != dest->_colBlockIndices[i])
        return false;
    }
  }
  for (size_t i = 0; i < _blockCols.size(); i++) {
    for (typename SparseBlockMatrix<MatrixType>::IntBlockMap::const_iterator it =
        _blockCols[i].begin(); it != _blockCols[i].end(); it++) {
      typename SparseBlockMatrix<MatrixType>::SparseMatrixBlock* s = it->second;
      typename SparseBlockMatrix<MatrixType>::SparseMatrixBlock* d =
          dest->block(it->first, i, true);
      (*d) += *s;
    }
  }
  return true;
}

template<class MatrixType>
template<class MatrixResultType, class MatrixFactorType>
bool SparseBlockMatrix<MatrixType>::multiply(
    SparseBlockMatrix<MatrixResultType>*& dest,
    const SparseBlockMatrix<MatrixFactorType> * M) const {
  // sanity check
  if (_colBlockIndices.size() != M->_rowBlockIndices.size())
    return false;
  for (size_t i = 0; i < _colBlockIndices.size(); i++) {
    if (_colBlockIndices[i] != M->_rowBlockIndices[i])
      return false;
  }
  if (!dest) {
    dest = new SparseBlockMatrix<MatrixResultType>(&_rowBlockIndices[0],
        &(M->_colBlockIndices[0]), _rowBlockIndices.size(),
        M->_colBlockIndices.size());
  }
  if (!dest->_hasStorage)
    return false;
  for (size_t i = 0; i < M->_blockCols.size(); i++) {
    for (typename SparseBlockMatrix<MatrixFactorType>::IntBlockMap::const_iterator it =
        M->_blockCols[i].begin(); it != M->_blockCols[i].end(); it++) {
      // look for a non-zero block in a row of column it
      int colM = i;
      const typename SparseBlockMatrix<MatrixFactorType>::SparseMatrixBlock *b =
          it->second;
      typename SparseBlockMatrix<MatrixType>::IntBlockMap::const_iterator rbt =
          _blockCols[it->first].begin();
      while (rbt != _blockCols[it->first].end()) {
        //int colA=it->first;
        int rowA = rbt->first;
        typename SparseBlockMatrix<MatrixType>::SparseMatrixBlock *a =
            rbt->second;
        typename SparseBlockMatrix<MatrixResultType>::SparseMatrixBlock *c =
            dest->block(rowA, colM, true);
        assert(c->rows() == a->rows());
        assert(c->cols() == b->cols());
        rbt++;
        (*c) += (*a) * (*b);
      }
    }
  }
  return false;
}

template<typename MatrixType>
inline void axpy(const MatrixType& A, const Map<VectorXd>& x, int xoff,
    Map<VectorXd>& y, int yoff) {
  y.segment < MatrixType::RowsAtCompileTime > (yoff) += A * x.segment
      < MatrixType::ColsAtCompileTime > (xoff);
}

template<>
inline void axpy(const MatrixXd& A, const Map<VectorXd>& x, int xoff,
    Map<VectorXd>& y, int yoff) {
  y.segment(yoff, A.rows()) += A * x.segment(xoff, A.cols());
}

template<typename MatrixType>
inline void atxpy(const MatrixType& A, Map<const VectorXd>& x, int xoff,
    Map<VectorXd>& y, int yoff) {
  y.segment < MatrixType::ColsAtCompileTime > (yoff) += A.transpose()
      * x.segment < MatrixType::RowsAtCompileTime > (xoff);
}

template<>
inline void atxpy(const MatrixXd& A, Map<const VectorXd>& x, int xoff,
    Map<VectorXd>& y, int yoff) {
  y.segment(yoff, A.cols()) += A.transpose() * x.segment(xoff, A.rows());
}

template<class MatrixType>
void SparseBlockMatrix<MatrixType>::multiply(double*& dest,
    const double* src) const {
  if (!dest) {
    dest = new double[_rowBlockIndices[_rowBlockIndices.size() - 1]];
    memset(dest, 0,
        _rowBlockIndices[_rowBlockIndices.size() - 1] * sizeof(double));
  }

  // map the memory by Eigen
  Map<VectorXd> destVec(dest, rows());
  const Map<VectorXd> srcVec(src, cols());

  for (size_t i = 0; i < _blockCols.size(); i++) {
    int srcOffset = i ? _colBlockIndices[i - 1] : 0;

    for (typename SparseBlockMatrix<MatrixType>::IntBlockMap::const_iterator it =
        _blockCols[i].begin(); it != _blockCols[i].end(); ++it) {
      const typename SparseBlockMatrix<MatrixType>::SparseMatrixBlock* a =
          it->second;
      int destOffset = it->first ? _rowBlockIndices[it->first - 1] : 0;
      // destVec += *a * srcVec (according to the sub-vector parts)
      axpy(*a, srcVec, srcOffset, destVec, destOffset);
    }
  }
}

template<class MatrixType>
void SparseBlockMatrix<MatrixType>::rightMultiply(double*& dest,
    const double* src) const {
  int destSize = cols();

  if (!dest) {
    dest = new double[destSize];
    memset(dest, 0, destSize * sizeof(double));
  }

  // map the memory by Eigen
  Map<VectorXd> destVec(dest, destSize);
  Map<const VectorXd> srcVec(src, rows());

#   ifdef G2O_OPENMP
#   pragma omp parallel for default (shared)
#   endif
  for (int i = 0; i < static_cast<int>(_blockCols.size()); i++) {
    int destOffset = colBaseOfBlock(i);
    for (typename SparseBlockMatrix<MatrixType>::IntBlockMap::const_iterator it =
        _blockCols[i].begin(); it != _blockCols[i].end(); ++it) {
      const typename SparseBlockMatrix<MatrixType>::SparseMatrixBlock* a =
          it->second;
      int srcOffset = rowBaseOfBlock(it->first);
      // destVec += *a.transpose() * srcVec (according to the sub-vector parts)
      atxpy(*a, srcVec, srcOffset, destVec, destOffset);
    }
  }

}

template<class MatrixType>
void SparseBlockMatrix<MatrixType>::scale(double a_) {
  for (size_t i = 0; i < _blockCols.size(); i++) {
    for (typename SparseBlockMatrix<MatrixType>::IntBlockMap::const_iterator it =
        _blockCols[i].begin(); it != _blockCols[i].end(); ++it) {
      typename SparseBlockMatrix<MatrixType>::SparseMatrixBlock* a = it->second;
      *a *= a_;
    }
  }
}

template<class MatrixType>
SparseBlockMatrix<MatrixType>* SparseBlockMatrix<MatrixType>::slice(int rmin,
    int rmax, int cmin, int cmax, bool alloc) const {
  int m = rmax - rmin;
  int n = cmax - cmin;
  int rowIdx[m];
  rowIdx[0] = rowsOfBlock(rmin);
  for (int i = 1; i < m; i++) {
    rowIdx[i] = rowIdx[i - 1] + rowsOfBlock(rmin + i);
  }

  int colIdx[n];
  colIdx[0] = colsOfBlock(cmin);
  for (int i = 1; i < n; i++) {
    colIdx[i] = colIdx[i - 1] + colsOfBlock(cmin + i);
  }
  typename SparseBlockMatrix<MatrixType>::SparseBlockMatrix* s =
      new SparseBlockMatrix(rowIdx, colIdx, m, n, true);
  for (int i = 0; i < n; i++) {
    int mc = cmin + i;
    for (typename SparseBlockMatrix<MatrixType>::IntBlockMap::const_iterator it =
        _blockCols[mc].begin(); it != _blockCols[mc].end(); it++) {
      if (it->first >= rmin && it->first < rmax) {
        typename SparseBlockMatrix<MatrixType>::SparseMatrixBlock* b =
            alloc ?
                new typename SparseBlockMatrix<MatrixType>::SparseMatrixBlock(
                    *(it->second)) :
                it->second;
        s->_blockCols[i].insert(std::make_pair(it->first - rmin, b));
      }
    }
  }
  s->_hasStorage = alloc;
  return s;
}

template<class MatrixType>
size_t SparseBlockMatrix<MatrixType>::nonZeroBlocks() const {
  size_t count = 0;
  for (size_t i = 0; i < _blockCols.size(); i++)
    count += _blockCols[i].size();
  return count;
}

template<class MatrixType>
size_t SparseBlockMatrix<MatrixType>::nonZeros() const {
  if (MatrixType::SizeAtCompileTime != Eigen::Dynamic) {
    size_t nnz = nonZeroBlocks() * MatrixType::SizeAtCompileTime;
    return nnz;
  } else {
    size_t count = 0;
    for (size_t i = 0; i < _blockCols.size(); i++) {
      for (typename SparseBlockMatrix<MatrixType>::IntBlockMap::const_iterator it =
          _blockCols[i].begin(); it != _blockCols[i].end(); it++) {
        const typename SparseBlockMatrix<MatrixType>::SparseMatrixBlock* a =
            it->second;
        count += a->cols() * a->rows();
      }
    }
    return count;
  }
}

template<class MatrixType>
std::ostream& operator <<(std::ostream& os,
    const SparseBlockMatrix<MatrixType>& m) {
  os << "RBI: " << m.rowBlockIndices().size();
  for (size_t i = 0; i < m.rowBlockIndices().size(); i++)
    os << " " << m.rowBlockIndices()[i];
  os << std::endl;
  os << "CBI: " << m.colBlockIndices().size();
  for (size_t i = 0; i < m.colBlockIndices().size(); i++)
    os << " " << m.colBlockIndices()[i];
  os << std::endl;

  for (size_t i = 0; i < m.blockCols().size(); i++) {
    for (typename SparseBlockMatrix<MatrixType>::IntBlockMap::const_iterator it =
        m.blockCols()[i].begin(); it != m.blockCols()[i].end(); it++) {
      const typename SparseBlockMatrix<MatrixType>::SparseMatrixBlock* b =
          it->second;
      os << "BLOCK: " << it->first << " " << i << std::endl;
      os << *b << std::endl;
    }
  }
  return os;
}

template<class MatrixType>
bool SparseBlockMatrix<MatrixType>::symmPermutation(
    SparseBlockMatrix<MatrixType>*& dest, const int* pinv,
    bool upperTriangle) const {
  // compute the permuted version of the new row/column layout
  size_t n = _rowBlockIndices.size();
  // computed the block sizes
  int blockSizes[_rowBlockIndices.size()];
  blockSizes[0] = _rowBlockIndices[0];
  for (size_t i = 1; i < n; i++) {
    blockSizes[i] = _rowBlockIndices[i] - _rowBlockIndices[i - 1];
  }
  // permute them
  int pBlockIndices[_rowBlockIndices.size()];
  for (size_t i = 0; i < n; i++) {
    pBlockIndices[pinv[i]] = blockSizes[i];
  }
  for (size_t i = 1; i < n; i++) {
    pBlockIndices[i] += pBlockIndices[i - 1];
  }
  // allocate C, or check the structure;
  if (!dest) {
    dest = new SparseBlockMatrix(pBlockIndices, pBlockIndices, n, n);
  } else {
    if (dest->_rowBlockIndices.size() != n)
      return false;
    if (dest->_colBlockIndices.size() != n)
      return false;
    for (size_t i = 0; i < n; i++) {
      if (dest->_rowBlockIndices[i] != pBlockIndices[i])
        return false;
      if (dest->_colBlockIndices[i] != pBlockIndices[i])
        return false;
    }
    dest->clear();
  }
  // now ready to permute the columns
  for (size_t i = 0; i < n; i++) {
    //cerr << PVAR(i) <<  " ";
    int pi = pinv[i];
    for (typename SparseBlockMatrix<MatrixType>::IntBlockMap::const_iterator it =
        _blockCols[i].begin(); it != _blockCols[i].end(); it++) {
      int pj = pinv[it->first];

      const typename SparseBlockMatrix<MatrixType>::SparseMatrixBlock* s =
          it->second;

      typename SparseBlockMatrix<MatrixType>::SparseMatrixBlock* b = 0;
      if (!upperTriangle || pj <= pi) {
        b = dest->block(pj, pi, true);
        assert(b->cols() == s->cols());
        assert(b->rows() == s->rows());
        *b = *s;
      } else {
        b = dest->block(pi, pj, true);
        assert(b);
        assert(b->rows() == s->cols());
        assert(b->cols() == s->rows());
        *b = s->transpose();
      }
    }
    //cerr << endl;
    // within each row,
  }
  return true;

}

template<class MatrixType>
int SparseBlockMatrix<MatrixType>::fillCCS(double* Cx,
    bool upperTriangle) const {
  double* CxStart = Cx;
  for (size_t i = 0; i < _blockCols.size(); ++i) {
    int cstart = i ? _colBlockIndices[i - 1] : 0;
    int csize = colsOfBlock(i);
    for (int c = 0; c < csize; c++) {
      for (typename SparseBlockMatrix<MatrixType>::IntBlockMap::const_iterator it =
          _blockCols[i].begin(); it != _blockCols[i].end(); ++it) {
        const typename SparseBlockMatrix<MatrixType>::SparseMatrixBlock* b =
            it->second;
        int rstart = it->first ? _rowBlockIndices[it->first - 1] : 0;

        int elemsToCopy = b->rows();
        if (upperTriangle && rstart == cstart)
          elemsToCopy = c + 1;
        memcpy(Cx, b->data() + c * b->rows(), elemsToCopy * sizeof(double));
        Cx += elemsToCopy;

      }
    }
  }
  return Cx - CxStart;
}

template<class MatrixType>
int SparseBlockMatrix<MatrixType>::fillCCS(int* Cp, int* Ci, double* Cx,
    bool upperTriangle) const {
  int nz = 0;
  for (size_t i = 0; i < _blockCols.size(); ++i) {
    int cstart = i ? _colBlockIndices[i - 1] : 0;
    int csize = colsOfBlock(i);
    for (int c = 0; c < csize; c++) {
      *Cp = nz;
      for (typename SparseBlockMatrix<MatrixType>::IntBlockMap::const_iterator it =
          _blockCols[i].begin(); it != _blockCols[i].end(); ++it) {
        const typename SparseBlockMatrix<MatrixType>::SparseMatrixBlock* b =
            it->second;
        int rstart = it->first ? _rowBlockIndices[it->first - 1] : 0;

        int elemsToCopy = b->rows();
        if (upperTriangle && rstart == cstart)
          elemsToCopy = c + 1;
        for (int r = 0; r < elemsToCopy; ++r) {
          *Cx++ = (*b)(r, c);
          *Ci++ = rstart++;
          ++nz;
        }
      }
      ++Cp;
    }
  }
  *Cp = nz;
  return nz;
}

template<class MatrixType>
void SparseBlockMatrix<MatrixType>::fillBlockStructure(
    MatrixStructure& ms) const {
  int n = _colBlockIndices.size();
  int nzMax = (int) nonZeroBlocks();

  ms.alloc(n, nzMax);
  ms.m = _rowBlockIndices.size();

  int nz = 0;
  int* Cp = ms.Ap;
  int* Ci = ms.Aii;
  for (size_t i = 0; i < _blockCols.size(); ++i) {
    *Cp = nz;
    const int& c = i;
    for (typename SparseBlockMatrix<MatrixType>::IntBlockMap::const_iterator it =
        _blockCols[i].begin(); it != _blockCols[i].end(); ++it) {
      const int& r = it->first;
      if (r <= c) {
        *Ci++ = r;
        ++nz;
      }
    }
    Cp++;
  }
  *Cp = nz;
  assert(nz <= nzMax);
}

template<class MatrixType>
template<class MatrixTargetType>
void SparseBlockMatrix<MatrixType>::toDense(MatrixTargetType &dest,
    bool upperTriangle) const {

  for (size_t r = 0; r < rowBlockIndices().size(); ++r) {
    for (size_t c = (upperTriangle ? r : 0); c < colBlockIndices().size();
        ++c) {
      const SparseMatrixBlock *m = block(r, c);

      int r_base = rowBaseOfBlock(r);
      int r_size = rowsOfBlock(r);

      int c_base = colBaseOfBlock(c);
      int c_size = colsOfBlock(c);

      if (m == NULL) {
        dest.block(r_base, c_base, r_size, c_size).setZero();
      } else {
        dest.block(r_base, c_base, r_size, c_size) = *m;
      }

      if (upperTriangle) {
        if (m == NULL) {
          dest.block(c_base, r_base, c_size, r_size).setZero();
        } else {
          dest.block(c_base, r_base, c_size, r_size) = m->transpose();
        }
      }
    }
  }
}

template<class MatrixType>
bool SparseBlockMatrix<MatrixType>::writeOctave(const char* filename,
    bool upperTriangle) const {
  std::string name = filename;
  std::string::size_type lastDot = name.find_last_of('.');
  if (lastDot != std::string::npos)
    name = name.substr(0, lastDot);

  std::vector<TripletEntry> entries;
  for (size_t i = 0; i < _blockCols.size(); ++i) {
    const int& c = i;
    for (typename SparseBlockMatrix<MatrixType>::IntBlockMap::const_iterator it =
        _blockCols[i].begin(); it != _blockCols[i].end(); ++it) {
      const int& r = it->first;
      const MatrixType& m = *(it->second);
      for (int cc = 0; cc < m.cols(); ++cc)
        for (int rr = 0; rr < m.rows(); ++rr) {
          int aux_r = rowBaseOfBlock(r) + rr;
          int aux_c = colBaseOfBlock(c) + cc;
          entries.push_back(TripletEntry(aux_r, aux_c, m(rr, cc)));
          if (upperTriangle && r != c) {
            entries.push_back(TripletEntry(aux_c, aux_r, m(rr, cc)));
          }
        }
    }
  }

  int nz = entries.size();
  std::sort(entries.begin(), entries.end(), TripletColSort());

  std::ofstream fout(filename);
  fout << "# name: " << name << std::endl;
  fout << "# type: sparse matrix" << std::endl;
  fout << "# nnz: " << nz << std::endl;
  fout << "# rows: " << rows() << std::endl;
  fout << "# columns: " << cols() << std::endl;
  fout << std::setprecision(9) << std::endl;

  for (std::vector<TripletEntry>::const_iterator it = entries.begin();
      it != entries.end(); ++it) {
    const TripletEntry& entry = *it;
    fout << entry.r + 1 << " " << entry.c + 1 << " " << entry.x << std::endl;
  }
  return fout.good();
}

}      // end namespace
