/*
Copyright (c) 2013-2016 Politecnico di Milano.
All rights reserved. This program and the accompanying materials
are made available under the terms of the GNU Lesser Public License v3
which accompanies this distribution, and is available at
https://www.gnu.org/licenses/lgpl.html

Contributors:
    Davide A. Cucci (davide.cucci@epfl.ch)    
*/
    
/*
 * GenericLinearConstraintFactory.cpp
 *
 *  Created on: Sep 26, 2013
 *      Author: davide
 */

#include "GenericLinearConstraintFactory.h"

#include <Eigen/Dense>

#include "g2o/core/optimizable_graph.h"
#include "g2o/core/sparse_block_matrix.h"

#include "g2o/solvers/csparse/linear_solver_csparse.h"

#include "PriorEdges/BasePriorEdgeInterface.h"
#include "GenericEdgeInterface.h"

#include "ROAMutils/StringUtils.h"

namespace ROAMestimation {

GenericLinearConstraint* GenericLinearConstraintFactory::buildGLC(
    const std::set<g2o::HyperGraph::Vertex *> &toBeRemoved) {

  // 1 - collect relevant nodes and edges

  std::set<g2o::HyperGraph::Vertex *> markovBlanket;
  std::set<g2o::HyperGraph::Edge *> fallingEdges;

  getMarkovBlanket(toBeRemoved, markovBlanket, fallingEdges);

  return buildGLC(toBeRemoved, markovBlanket, fallingEdges);
}

GenericLinearConstraint *GenericLinearConstraintFactory::buildGLC(
    const std::set<g2o::HyperGraph::Vertex *> &toBeRemoved,
    std::set<g2o::HyperGraph::Vertex *> &markovBlanket,
    std::set<g2o::HyperGraph::Edge *> &fallingEdges) {

  if (markovBlanket.size() == 0) {
    std::cerr << "[GLCfactory] Error: empty markov blanket!! " << std::endl;
    return NULL;
  }

  // 2 - produce a mapping between nodes and blocks in the available information matrix
  //     and also determine the dimension of each block
  //     FIRST we put the toBeRemoved ones, SECOND the markovBlanket
  //     N.B. WE CARE ONLY ABOUT THE FREE NODES (i.e. not fixed)

  std::map<g2o::HyperGraph::Vertex *, int> vmap;
  int k = 0;

  // as required in SparseBlockMatrix constructor:
  // the component i of the array should contain the index of the first row of the block i+1.
  int structure[toBeRemoved.size() + markovBlanket.size()];

  std::set<g2o::HyperGraph::Vertex *>::const_iterator vit;
  for (vit = toBeRemoved.begin(); vit != toBeRemoved.end(); ++vit) {
    g2o::OptimizableGraph::Vertex * ov =
        static_cast<g2o::OptimizableGraph::Vertex *>(*vit);

    if (!ov->fixed()) {
      if (k > 0) {
        structure[k] = structure[k - 1] + ov->dimension();
      } else {
        structure[k] = ov->dimension();
      }

      vmap[*vit] = k++;
    }
  }

  int tbr_notfixed = vmap.size(); // the actual number of free nodes to be removed
  int startOfBlanket = structure[k - 1]; // row at which the markov blanket starts

  /* ---------- TODO: WORKAROUND to remove information regarding certain nodes
   int BaPosition = -1; // the first row in the hessian for the Ba vertex
   // ---------- */

  std::set<g2o::HyperGraph::Vertex *>::const_iterator vit2;
  for (vit2 = markovBlanket.begin(); vit2 != markovBlanket.end(); ++vit2) {
    g2o::OptimizableGraph::Vertex * ov =
        static_cast<g2o::OptimizableGraph::Vertex *>(*vit2);

    if (!ov->fixed()) {
      if (k > 0) {
        structure[k] = structure[k - 1] + ov->dimension();
      } else {
        structure[k] = ov->dimension();
      }

      vmap[*vit2] = k++;
    }

    /* ---------- TODO: WORKAROUND to remove information regarding certain nodes
     // get the position in the matrix for the Ba parameter vertex
     GenericVertexInterface *gv = dynamic_cast<GenericVertexInterface *>(ov);
     assert(gv != NULL);

     // check for suffix _Ba

     if (gv->getCategory().compare(gv->getCategory().length() - 3, 3, "_Ba")
     == 0) {
     BaPosition = structure[k - 1] - ov->dimension();
     }

     // ---------- */

  }

  int mb_notfixed = vmap.size() - tbr_notfixed; //the actual number of free nodes in the markov blanket of toBeRemoved
  int blanketSize = structure[k - 1] - startOfBlanket; // size in rows of the markov blanket area

  int h_size = tbr_notfixed + mb_notfixed;

# ifdef DEBUG_BUILD
  {
    std::ofstream hstruct("debug/GLCh_struct.txt");

    int k = 0;
    for (auto vit = toBeRemoved.begin(); vit != toBeRemoved.end(); ++vit) {
      GenericVertexInterface *gvi = dynamic_cast<GenericVertexInterface *>(*vit);
      assert(gvi!= NULL);

      if (!gvi->getg2oOptGraphPointer()->fixed()) {
        hstruct << gvi->getCategory() << "_" << gvi->getTimestamp() << "["
        << ROAMutils::StringUtils::writeNiceTimestamp(gvi->getTimestamp())
        << "]" << structure[k] << std::endl;
        k++;
      }
    }

    for (auto vit = markovBlanket.begin(); vit != markovBlanket.end();
        ++vit) {
      GenericVertexInterface *gvi = dynamic_cast<GenericVertexInterface *>(*vit);
      assert(gvi!= NULL);

      if (!gvi->getg2oOptGraphPointer()->fixed()) {
        hstruct << gvi->getCategory() << "_" << gvi->getTimestamp() << "["
        << ROAMutils::StringUtils::writeNiceTimestamp(gvi->getTimestamp())
        << "] " << structure[k] << std::endl;
        k++;
      }

    }

    hstruct.close();
  }
# endif

# ifdef DEBUG_PRINT_INFO_MESSAGES
  std::cerr << "[GLCfactory] Info: falling edges:" << std::endl;

  std::set<g2o::HyperGraph::Edge *>::const_iterator deit;
  for (deit = fallingEdges.begin(); deit != fallingEdges.end(); ++deit) {
    GenericEdgeInterface *e = dynamic_cast<GenericEdgeInterface *>(*deit);
    if (e != NULL) {
      std::cerr << e->writeDebugInfo();
    }

    BasePriorEdgeInterface *p = dynamic_cast<BasePriorEdgeInterface *>(*deit);
    if (p != NULL) {
      std::cerr << p->writeDebugInfo();
    }

    GenericLinearConstraint *glc =
    dynamic_cast<GenericLinearConstraint *>(*deit);

    if (glc != NULL) {
      std::cerr << glc->writeDebugInfo();
    }

    std::cerr << "(ptr: " << *deit << ")" << std::endl;
  }

  std::cerr << "[GLCfactory] --- " << std::endl;
# endif

# ifdef DEBUG_PRINT_INFO_MESSAGES
  std::cerr << "[GLCfactory] Info: falling nodes:" << std::endl;

  for (auto vit = toBeRemoved.begin(); vit != toBeRemoved.end(); ++vit) {

    GenericVertexInterface *gvi = dynamic_cast<GenericVertexInterface *>(*vit);
    g2o::OptimizableGraph::Vertex *ov = static_cast<g2o::OptimizableGraph::Vertex *>(*vit);

    assert(gvi!= NULL);

    std::cerr << gvi->getCategory() << "(" << ov->id() << ")" << "["
    << ROAMutils::StringUtils::writeNiceTimestamp(gvi->getTimestamp())
    << "]" << (ov->fixed() ? " -- Fixed --:" : ":") << std::endl;

  }
  std::cerr << "[GLCfactory] --- " << std::endl;
# endif

# ifdef DEBUG_PRINT_INFO_MESSAGES
  std::cerr << "[GLCfactory] Info: markov blanket:" << std::endl;

  for (auto vit = markovBlanket.begin(); vit != markovBlanket.end(); ++vit) {

    GenericVertexInterface *gvi = dynamic_cast<GenericVertexInterface *>(*vit);
    g2o::OptimizableGraph::Vertex *ov = static_cast<g2o::OptimizableGraph::Vertex *>(*vit);

    assert(gvi!= NULL);

    std::cerr << gvi->getCategory() << "(" << ov->id() << ")" << "["
    << ROAMutils::StringUtils::writeNiceTimestamp(gvi->getTimestamp())
    << "]" << (ov->fixed() ? " -- Fixed --:" : ":") << std::endl;

  }
  std::cerr << "[GLCfactory] --- " << std::endl;
# endif

  // debug
# ifdef DEBUG_PRINT_INFO_MESSAGES
  std::cerr << "[GLCfactory] Info: toBeRemoved: " << tbr_notfixed
  << ", markovBlanket: " << mb_notfixed << ", involved edges:"
  << fallingEdges.size() << std::endl;
# endif

  if (tbr_notfixed == 0) {
# 	ifdef DEBUG_PRINT_INFO_MESSAGES
    std::cerr << "[GLCfactory] Warning: buildGLC called with empty toBeRemoved set." << std::endl;
#		endif

    return NULL;
  }

  // 3 - allocate a sparse block matrix to contain the available information

  g2o::SparseBlockMatrix<Eigen::MatrixXd> h(structure, structure, h_size,
      h_size, true);

  // 4 - fill this big matrix. We need to recompute the hessian since
  //     TODO: for the Lamba_bb part, it is the current system Hessian
  //           values could be taken from there

  std::set<g2o::HyperGraph::Edge *>::const_iterator eit;

  //for each of the affected edges
  for (eit = fallingEdges.begin(); eit != fallingEdges.end(); ++eit) {

    g2o::OptimizableGraph::Edge *oe =
        static_cast<g2o::OptimizableGraph::Edge *>(*eit);

    // for each couple of vertices in that edge (upper triangle, including diagonal)
    for (int i = 0; i < oe->vertices().size(); i++) {

      g2o::OptimizableGraph::Vertex *vi =
          static_cast<g2o::OptimizableGraph::Vertex *>(oe->vertices()[i]);

      // assure this vertex is to be considered
      if (vmap.count(vi) == 0) {
        assert(!isAllowedInGLC(vi) || vi->fixed());
        continue;
      }

      assert(vi->fixed() == false);

      const Eigen::Map<const Eigen::MatrixXd> Ji(oe->jacobianData(i),
          oe->dimension(), vi->dimension());
      const Eigen::Map<const Eigen::MatrixXd> I(oe->informationData(),
          oe->dimension(), oe->dimension());

      for (int j = i; j < oe->vertices().size(); j++) {

        g2o::OptimizableGraph::Vertex *vj =
            static_cast<g2o::OptimizableGraph::Vertex *>(oe->vertices()[j]);

        // assure this vertex is to be considered
        if (vmap.count(vj) == 0) {
          assert(!isAllowedInGLC(vj) || vj->fixed());
          continue;
        }

        assert(vj->fixed() == false);

        const Eigen::Map<const Eigen::MatrixXd> Jj(oe->jacobianData(j),
            oe->dimension(), vj->dimension());

        // debug

        // find index in my information matrix

        int row = vmap[vi];
        int col = vmap[vj];

        if (row > col) { // we are in the lower triangle, transpose!!
          std::swap(row, col);

          Eigen::MatrixXd &cb = *(h.block(row, col, true));

          cb.noalias() += Jj.transpose() * I * Ji;
        } else {
          Eigen::MatrixXd &cb = *(h.block(row, col, true));

          cb.noalias() += Ji.transpose() * I * Jj;
        }

#       ifdef DEBUG_ARE_ALL_JACOBIANS_INITIALIZED
        // try to discover if there is some jacobian which is not initialized.
        // this could be point out by very high coefficients.

        if (Ji.maxCoeff() > 1e64 || Jj.maxCoeff() > 1e64) {
          std::cerr << "[GLCfactory] Error: bad Jacobian in ";

          GenericEdgeInterface *ei = dynamic_cast<GenericEdgeInterface *>(oe);
          if (ei != NULL) {
            std::cerr << ei->writeDebugInfo();
          } else {
            std::cerr << " GLC or Prior ";
          }

          std::cerr << " edge." << std::endl;

          std::cerr << "J(" << vj->id() << ")" << i << "_" << j << std::endl << Jj
          << std::endl;

          std::cerr << "Ji " << vmap[vi] << ": " << std::endl << Ji << std::endl;
          std::cerr << "Jj " << vmap[vj] << ": " << std::endl << Jj << std::endl;
          std::cerr << "I: " << std::endl << I << std::endl;

          assert(false);
        }
#       endif
      }
    }
  }

# ifdef DEBUG_BUILD
  h.writeOctave("debug/GLCh.txt");
# endif

  // 5 -  Now that we have computed the overall hessian of the vertex in the markov
  //      blanket with respect to ONLY the edges which have to fall
  //      we can move in the magic world of SCHUR complement

  // we slice the computed hessian to have all the blocks needed

  /* debug
   // h.writeOctave("Lambda_big.dat", true);
   std::cerr << "[GLCfactory] Info: H" << std::endl << h << std::endl;
   //*/

  g2o::SparseBlockMatrix<Eigen::MatrixXd> *h_aa = h.slice(tbr_notfixed, h_size,
      tbr_notfixed, h_size, false);
  g2o::SparseBlockMatrix<Eigen::MatrixXd> *h_ba = h.slice(0, tbr_notfixed,
      tbr_notfixed, h_size, false);
  g2o::SparseBlockMatrix<Eigen::MatrixXd> *h_bb = h.slice(0, tbr_notfixed, 0,
      tbr_notfixed, false);

  // fill the blockIndices vector so that we can invert the h_bb slice

  std::vector<std::pair<int, int> > blockIndices;

  for (int i = 0; i < tbr_notfixed; i++) {
    for (int j = 0; j < tbr_notfixed; j++) {
      blockIndices.emplace_back(i, j);
    }
  }

  // invert h_bb

  /* debug
   std::cerr << h << std::endl;
   std::cerr << "non zero blocks: " << h.nonZeroBlocks() << std::endl;
   //*/

  g2o::LinearSolverCSparse<Eigen::MatrixXd> solver;

  /* debug
   std::cerr << "[GLCfactory] Info: Lambda_bb" << std::endl << *h_bb << std::endl;
   //*/

  //
  // ------------------------ OLD CODE --------------------------------
  //
  // works but it is very slow since the solvePattern method is very slow
  // when the patter is the whole matrix
  g2o::SparseBlockMatrix<Eigen::MatrixXd> h_bb_inv;

  // in solvePattern h_bb is assumed to be symmetric (only upper triangular block is stored) and positive-semi-definit
  bool inv_ok = solver.solvePattern(h_bb_inv, blockIndices, *h_bb);

  if (inv_ok == false) {
    std::cerr << "[GLCfactory] Error: inversion of Lambda_bb Failed!!"
        << std::endl << *h_bb << std::endl;
    assert(false);
  }

  // do the shur complement, omega_t = omega_aa - omega_ba^T * omega_bb^-1 * omega_ba
  // NOTE THAT the structure of the big matrix is [bb ba; ab aa], and we want the schur complement of bb

  // TODO: extend SparseBlockMatrix so that these operation require less temporaries
  // in omega_aa I have only the upper triangular block. I need also the lower one (H is symmetric)
  for (int r = 0; r < mb_notfixed - 1; r++) {
    for (int c = r + 1; c < mb_notfixed; c++) {

      Eigen::MatrixXd *src = h_aa->block(r, c);

      if (src != NULL) {
        Eigen::MatrixXd *dest = h_aa->block(c, r, true);
        dest->noalias() = src->transpose();
      }
    }
  }

  g2o::SparseBlockMatrix<Eigen::MatrixXd> *tmp1 = NULL;
  g2o::SparseBlockMatrix<Eigen::MatrixXd> *tmp2 = NULL;
  g2o::SparseBlockMatrix<Eigen::MatrixXd> *tmp3 = NULL;

  h_bb_inv.multiply(tmp1, h_ba);  // tmp1 = h_bb^-1 * h_ba
  h_ba->transpose(tmp2);          // tmp2 = h_ba^T
  tmp2->multiply(tmp3, tmp1);     // tmp3 = tmp2 * tmp1
  tmp3->scale(-1.0);
  tmp3->add(h_aa);

  delete tmp1;
  delete tmp2;
  delete tmp3;

  // very nice now h_aa contains the target information
  // 6 - The resulting information matrix may be singular. This is no good
  //     So we perform Principal Component Analysis
  // first we copy the SparseBlockMatrix content into a full fledged, dense, Eigen<MatrixXd>
  Eigen::MatrixXd Lambda_t(h_aa->rows(), h_aa->cols());

  h_aa->toDense(Lambda_t);

  //------------------end of OLD CODE --------------------------------
  /*/

   /*
   // ------------------------ NEW CODE --------------------------------
   //
   // TODO: this can probably made much faster
   //

   // 1 - convert h_ba into a dense EigenXd, column-major, matrix, then transpose it and stuff it info h_ab
   Eigen::MatrixXd h_ba_dense(h_ba->rows(), h_ba->cols());
   Eigen::MatrixXd tmp(h_ba->rows(), h_ba->cols());

   h_ba->toDense(tmp);
   h_ba->toDense(h_ba_dense);

   // 2 - allocate the dense Lambda_t return matrix and stuff it with h_aa;

   Eigen::MatrixXd Lambda_t(h_aa->rows(), h_aa->cols());
   h_aa->toDense(Lambda_t, true); // it is upper triangular

   // 3 - have tmp (which is h_ba) to be replpaced with h_bb^-1 * h_ba

   solver.invAmultB(*h_bb, tmp);

   // 4 - compute the schur complement

   Lambda_t.noalias() -= h_ba_dense.transpose() * tmp;

   //
   // ------------------end of NEW CODE --------------------------------
   //*/

  /* write some debug matrices

   {
   Eigen::IOFormat CSVFmt(16, 0, ", ");

   Eigen::MatrixXd h_bb_dense(h_bb->rows(), h_bb->cols());
   h_bb->toDense(h_bb_dense, true);

   Eigen::MatrixXd h_aa_dense(h_aa->rows(), h_aa->cols());
   h_aa->toDense(h_aa_dense, true);

   std::ofstream hbaf("debug/h_ba.txt");
   hbaf << h_ba_dense.format(CSVFmt) << std::endl;
   hbaf.close();

   std::ofstream haaf("debug/h_aa.txt");
   haaf << h_aa_dense.format(CSVFmt) << std::endl;
   haaf.close();

   std::ofstream hbbf("debug/h_bb.txt");
   hbbf << h_bb_dense.format(CSVFmt) << std::endl;
   hbbf.close();
   }

   //*/

  /*
   //   ------------------------ DUNB TESTING CODE -----------------------
   //      compute shur complement with a naive, Eigen based, approach
   Eigen::MatrixXd Lambda_t(h_aa->rows(), h_aa->cols());
   h_aa->toDense(Lambda_t, true); // it is upper triangular

   Eigen::MatrixXd h_ba_dense(h_ba->rows(), h_ba->cols());
   h_ba->toDense(h_ba_dense);

   Eigen::MatrixXd h_bb_dense(h_bb->rows(), h_bb->cols());
   h_bb->toDense(h_bb_dense, true);

   Lambda_t.noalias() -= h_ba_dense.transpose() * h_bb_dense.inverse()
   * h_ba_dense;
   //*/

# ifdef DEBUG_BUILD
  std::ofstream shur("debug/shur.txt");

  Eigen::IOFormat CSVFmt(9, 0, ", ");
  shur << Lambda_t.format(CSVFmt) << std::endl;

  shur.close();
# endif

  /* ---------- TODO: WORKAROUND to remove information regarding certain nodes
   // add some variance to Ba, tweak with the diagonal of the Ba hessian block
   if (BaPosition >= 0) { // if it was found in the current active vertices..

   Eigen::Matrix3d Ba_I = Lambda_t.block(BaPosition - startOfBlanket,
   BaPosition - startOfBlanket, 3, 3);
   Eigen::Matrix3d Ba_cov = Ba_I.inverse();
   Ba_cov += Eigen::Matrix3d::Identity();
   Lambda_t.block(BaPosition - startOfBlanket, BaPosition - startOfBlanket, 3,
   3) = Ba_cov.inverse();

   }

   // ---------- */

  // compute eigenvalues and eigenvectors for It
  Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> es(Lambda_t);

  // eigenvalues below the given threshold are discarded
  // i will contain the last eigenvalue greater than the threshold

# ifdef DEBUG_PRINT_INFO_MESSAGES
  std::cerr << "[GLCfactory] Info: constraint eigenvalues: " << es.eigenvalues().transpose() << std::endl;
# endif

  int i = 0;

  static const double _TH = 0.0;

  while (i < Lambda_t.rows() && es.eigenvalues()(i) <= _TH) {
    i++;
  }

  i = h_aa->rows() - i;

# ifdef DEBUG_PRINT_INFO_MESSAGES
  std::cerr << "[GLCfactory] Info: dimension of the constraint " << i
  << std::endl;
# endif

  // - 7 Finally we can construct the GenericLinearConstraint to be populated with G and the
  //     vertices in the markov blanket, properly ordered.

  GenericLinearConstraint *e = new GenericLinearConstraint;
  e->resize(mb_notfixed);

  // fill the vertices array

  std::set<g2o::HyperGraph::Vertex *>::const_iterator mbit;

  int vcnt = 0;
  for (mbit = markovBlanket.begin(); mbit != markovBlanket.end(); ++mbit) {
    g2o::OptimizableGraph::Vertex *ov =
        static_cast<g2o::OptimizableGraph::Vertex *>(*mbit);

    if (!ov->fixed()) {
      e->vertices()[vcnt++] = *mbit;
    }
  }

  // we compute the G matrix, sqrt(D) * U^T, now and put it directly into the edge
  Eigen::VectorXd tmpEigs = es.eigenvalues().tail(i).array().sqrt();

  Eigen::DiagonalMatrix<double, -1, -1> D(tmpEigs); //a diagonal matrix from the sqrt of eigenvalues

  e->accessGain() = D * es.eigenvectors().rightCols(i).transpose();

  /* debug
   std::cerr << e->accessGain() << std::endl;
   //*/

  e->resizeStructures();

  // fill the measurement vector with the current values of the vertices

  int startZ = 0;
  for (mbit = markovBlanket.begin(); mbit != markovBlanket.end(); ++mbit) {
    g2o::OptimizableGraph::Vertex *mv =
        static_cast<g2o::OptimizableGraph::Vertex *>(*mbit);

    if (!mv->fixed()) {
      Eigen::Map<Eigen::VectorXd> mest(mv->accessEstimateData(),
          mv->estimateDimension());

      e->measurement().segment(startZ, mv->estimateDimension()) = mest;

      startZ += mv->estimateDimension();
    }
  }

  // pffff we're done ;)

  return e;
}

void GenericLinearConstraintFactory::getMarkovBlanket(
    const std::set<g2o::HyperGraph::Vertex *> &n,
    std::set<g2o::HyperGraph::Vertex *> &markovBlanket,
    std::set<g2o::HyperGraph::Edge *> &fallingEdges) {

  // for each vertex
  for (std::set<g2o::HyperGraph::Vertex *>::const_iterator tbr = n.begin();
      tbr != n.end(); ++tbr) {

    // for each edge he is in
    for (g2o::HyperGraph::EdgeSet::const_iterator eit = (*tbr)->edges().begin();
        eit != (*tbr)->edges().end(); ++eit) {

      /* if the current edge is not a prior TODO: I don't want only the prior on the last pose in the window
       if (dynamic_cast<ROAMestimation::BasePriorEdgeInterface *>(*eit) != NULL) {
       continue;
       }
       //*/

      // add the edge to the falling ones
      fallingEdges.insert(*eit);

      // for each vertex in that edge
      for (g2o::HyperGraph::VertexVector::const_iterator vit =
          (*eit)->vertices().begin(); vit != (*eit)->vertices().end(); ++vit) {

        if (n.count(*vit) == 0) { // otherwise we have a non-null intersection btw n and markovBlanket

          if (isAllowedInGLC(*vit)) {
            markovBlanket.insert(*vit);
          }
        }
      }
    }
  }
}

bool GenericLinearConstraintFactory::isAllowedInGLC(
    const g2o::HyperGraph::Vertex* v) {

  /* avoid IMUIntegral biases in the GLC
   const int N = 2;
   const std::string forbidden_suffixes[N] = { "_Ba", "_Bw" };
   //*/

  // everything is allowed
  const int N = 0;
  const std::string forbidden_suffixes[N] = { };
  //*/

  const GenericVertexInterface *gv =
      dynamic_cast<const GenericVertexInterface *>(v);

  for (int i = 0; i < N; i++) {
    const std::string & name = gv->getCategory();

    if (name.length() >= forbidden_suffixes[i].length()) {
      if (name.compare(name.length() - forbidden_suffixes[i].length(),
          forbidden_suffixes[i].length(), forbidden_suffixes[i]) == 0) {
        return false;
      }
    }
  }
  return true;
}

} /* namespace ROAMestimation */

