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

#include "types_slam2d_online.h"
#include "types_slam3d_online.h"

#include "graph_optimizer_sparse_online.h"

#include "g2o/stuff/macros.h"
#include "g2o/core/block_solver.h"
#include "g2o/core/solver_factory.h"

#include "g2o/solvers/pcg/linear_solver_pcg.h"

#include <iostream>
#include <iomanip>
#include <fstream>
using namespace std;

#define DIM_TO_SOLVER(p, l) BlockSolver< BlockSolverTraits<p, l> >

#define ALLOC_PCG(s, p, l) \
  if (1) { \
    std::cerr << "# Using PCG online poseDim " << p << " landMarkDim " << l << " blockordering 1" << std::endl; \
    LinearSolverPCG< DIM_TO_SOLVER(p, l)::PoseMatrixType >* linearSolver = new LinearSolverPCG<DIM_TO_SOLVER(p, l)::PoseMatrixType>(); \
    linearSolver->setMaxIterations(6); \
    s = new DIM_TO_SOLVER(p, l)(opt, linearSolver); \
  } else (void)0

namespace g2o {

SparseOptimizerOnline::SparseOptimizerOnline(bool pcg) :
  SparseOptimizer(),
  slamDimension(3), newEdges(0), batchStep(true), vizWithGnuplot(false),
  _gnuplot(0), _usePcg(pcg)
{
}

SparseOptimizerOnline::~SparseOptimizerOnline()
{
  if (_gnuplot) {
    pclose(_gnuplot);
  }
}

int SparseOptimizerOnline::optimize(int iterations, bool online)
{
  //return SparseOptimizer::optimize(iterations, online);

  (void) iterations; // we only do one iteration anyhow
  Solver* solver = _solver;
  solver->init(online);

  int cjIterations=0;
  bool ok=true;

  if (!online) {
    ok = solver->buildStructure();
    if (! ok) {
      cerr << __PRETTY_FUNCTION__ << ": Failure while building CCS structure" << endl;
      return 0;
    }
  }

  if (_usePcg)
    batchStep = true;

  if (! online || batchStep) {
    //cerr << "BATCH" << endl;
    // copy over the updated estimate as new linearization point
    if (slamDimension == 3) {
      for (size_t i = 0; i < indexMapping().size(); ++i) {
        OnlineVertexSE2* v = static_cast<OnlineVertexSE2*>(indexMapping()[i]);
        v->estimate() = v->updatedEstimate;
      }
    }
    else if (slamDimension == 6) {
      for (size_t i=0; i < indexMapping().size(); ++i) {
        OnlineVertexSE3* v = static_cast<OnlineVertexSE3*>(indexMapping()[i]);
        v->estimate() = v->updatedEstimate;
      }
    }

    SparseOptimizer::computeActiveErrors();
    SparseOptimizer::linearizeSystem();
    solver->buildSystem();
  }
  else {
    //cerr << "UPDATE" << endl;
    // compute the active errors for the required edges
    for (HyperGraph::EdgeSet::iterator it = newEdges->begin(); it != newEdges->end(); ++it) {
      OptimizableGraph::Edge * e = static_cast<OptimizableGraph::Edge*>(*it);
      e->computeError();
    }
    // linearize the constraints
    for (HyperGraph::EdgeSet::iterator it = newEdges->begin(); it != newEdges->end(); ++it) {
      OptimizableGraph::Edge* e = static_cast<OptimizableGraph::Edge*>(*it);
      e->linearizeOplus();
    }
    // add the updated constraints again to the Hessian
    for (HyperGraph::EdgeSet::iterator it = newEdges->begin(); it != newEdges->end(); ++it) {
      OptimizableGraph::Edge* e = static_cast<OptimizableGraph::Edge*>(*it);
      e->constructQuadraticForm();
    }
    // update the b vector
    for (int i = 0; i < static_cast<int>(indexMapping().size()); ++i) {
      OptimizableGraph::Vertex* v = indexMapping()[i];
      int iBase = v->colInHessian();
      v->copyB(solver->b() + iBase);
    }
  }

  ok = solver->solve();
  update(solver->x());
  ++cjIterations; 

  if (verbose()){
    computeActiveErrors();
    cerr
      << "nodes = " << vertices().size()
      << "\t edges= " << _activeEdges.size()
      << "\t chi2= " << FIXED(activeChi2())
      << endl << endl;
  }

  if (vizWithGnuplot)
    gnuplotVisualization();

  if (! ok)
    return 0;
  return 1;
}

void SparseOptimizerOnline::update(double* update)
{
  if (slamDimension == 3) {
    for (size_t i=0; i < _ivMap.size(); ++i) {
      OnlineVertexSE2* v= static_cast<OnlineVertexSE2*>(_ivMap[i]);
      v->oplusUpdatedEstimate(update);
      update += 3;
    }
  }
  else if (slamDimension == 6) {
    for (size_t i=0; i < _ivMap.size(); ++i) {
      OnlineVertexSE3* v= static_cast<OnlineVertexSE3*>(_ivMap[i]);
      v->oplusUpdatedEstimate(update);
      update += 6;
    }
  }
}

bool SparseOptimizerOnline::updateInitialization(HyperGraph::VertexSet& vset, HyperGraph::EdgeSet& eset)
{
  newEdges = &eset;
  bool result = SparseOptimizer::updateInitialization(vset, eset);
  for (HyperGraph::VertexSet::iterator it = vset.begin(); it != vset.end(); ++it) {
    OptimizableGraph::Vertex* v = static_cast<OptimizableGraph::Vertex*>(*it);
    v->clearQuadraticForm(); // be sure that b is zero for this vertex
  }
  return result;
}

static Solver* createSolver(SparseOptimizer* opt, const std::string& solverName)
{
  g2o::Solver* s = 0;
  if (solverName == "pcg3_2_cholmod") {
    ALLOC_PCG(s, 3, 2);
  }
  else if (solverName == "pcg6_3_cholmod") {
    ALLOC_PCG(s, 6, 3);
  }
  return s;
}

bool SparseOptimizerOnline::initSolver(int dimension, int /*batchEveryN*/)
{
  slamDimension = dimension;
  SolverFactory* solverFactory = SolverFactory::instance();
  SolverProperty solverProperty;
  if (_usePcg) {
    if (dimension == 3) {
      setSolver(createSolver(this, "pcg3_2_cholmod"));
    } else {
      setSolver(createSolver(this, "pcg6_3_cholmod"));
    }
  }
  else {
    if (dimension == 3) {
      setSolver(solverFactory->construct("fix3_2_cholmod", this, solverProperty));
    } else {
      setSolver(solverFactory->construct("fix6_3_cholmod", this, solverProperty));
    }
  }
  solver()->setSchur(false);
  if (! solver()) {
    cerr << "Error allocating solver. Allocating CHOLMOD solver failed!" << endl;
    return false;
  }
  return true;
}

void SparseOptimizerOnline::gnuplotVisualization()
{
  if (slamDimension == 3) {
    if (! _gnuplot) {
      _gnuplot = popen("gnuplot -persistent", "w");
      fprintf(_gnuplot, "set terminal X11 noraise\n");
      fprintf(_gnuplot, "set size ratio -1\n");
    }
    fprintf(_gnuplot, "plot \"-\" w l\n");
    for (EdgeSet::iterator it = edges().begin(); it != edges().end(); ++it) {
      OnlineEdgeSE2* e = (OnlineEdgeSE2*) *it;
      OnlineVertexSE2* v1 = (OnlineVertexSE2*) e->vertices()[0];
      OnlineVertexSE2* v2 = (OnlineVertexSE2*) e->vertices()[1];
      fprintf(_gnuplot, "%f %f\n", v1->updatedEstimate.translation().x(), v1->updatedEstimate.translation().y());
      fprintf(_gnuplot, "%f %f\n\n", v2->updatedEstimate.translation().x(), v2->updatedEstimate.translation().y());
    }
    fprintf(_gnuplot, "e\n");
  }
  if (slamDimension == 6) {
    if (! _gnuplot) {
      _gnuplot = popen("gnuplot -persistent", "w");
      fprintf(_gnuplot, "set terminal X11 noraise\n");
    }
    fprintf(_gnuplot, "splot \"-\" w l\n");
    for (EdgeSet::iterator it = edges().begin(); it != edges().end(); ++it) {
      OnlineEdgeSE3* e = (OnlineEdgeSE3*) *it;
      OnlineVertexSE3* v1 = (OnlineVertexSE3*) e->vertices()[0];
      OnlineVertexSE3* v2 = (OnlineVertexSE3*) e->vertices()[1];
      fprintf(_gnuplot, "%f %f %f\n", v1->updatedEstimate.translation().x(), v1->updatedEstimate.translation().y(), v1->updatedEstimate.translation().z());
      fprintf(_gnuplot, "%f %f %f \n\n\n", v2->updatedEstimate.translation().x(), v2->updatedEstimate.translation().y(), v2->updatedEstimate.translation().z());
    }
    fprintf(_gnuplot, "e\n");
  }
}

} // end namespace
