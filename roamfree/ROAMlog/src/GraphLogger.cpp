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
 * GraphLogger.cpp
 *
 *  Created on: Jun 4, 2013
 *      Author: davide
 */

#include <sstream>

#include "GraphLogger.h"

#include "g2o/core/optimizable_graph.h"
#include "g2o/core/graph_optimizer_sparse.h"

namespace ROAMlog {

GraphLogger::GraphLogger(const std::string& basepath, g2o::SparseOptimizer* opt) :
    _basepath(basepath), _optimizer(opt) {
}

GraphLogger::~GraphLogger() {
  std::map<std::string, SlotRandomLogger *>::const_iterator it =
      _loggers.begin();

  while (it != _loggers.end()) {
    delete it++->second;
  }
}

void GraphLogger::sync() {
  const g2o::OptimizableGraph::VertexContainer &v_cont =
      _optimizer->activeVertices();

  for (auto it = v_cont.begin(); it != v_cont.end(); ++it) {
    g2o::OptimizableGraph::Vertex *v =
        static_cast<g2o::OptimizableGraph::Vertex *>(*it);

    ROAMestimation::GenericVertexInterface *vi;

    if ((vi = dynamic_cast<ROAMestimation::GenericVertexInterface *>(v)) != NULL) {
      // TODO: add some filter at this level, we may not want to log EVERY kind of vertex..
      std::map<std::string, SlotRandomLogger *>::iterator l_it = _loggers.find(
          vi->getCategory());

      if (l_it == _loggers.end()) {
        std::stringstream s;
        s << _basepath << "/" << vi->getCategory() << ".log";

        SlotRandomLogger *l = new SlotRandomLogger(s.str(), 16);

        std::pair<std::map<std::string, SlotRandomLogger *>::iterator, bool> ret =
            _loggers.insert(
                std::pair<std::string, SlotRandomLogger *>(vi->getCategory(),
                    l));

        l_it = ret.first;
      }

      l_it->second->logObject(*vi);
    } else {
      std::cerr << "[GraphLogger] Cannot log vertex id " << v->id()
          << std::endl;
    }
  }

  g2o::OptimizableGraph::EdgeContainer e_cont = _optimizer->activeEdges();

  for (auto it = e_cont.begin(); it != e_cont.end(); ++it) {
    g2o::OptimizableGraph::Edge *e =
        static_cast<g2o::OptimizableGraph::Edge *>(*it);

    ROAMestimation::GenericEdgeInterface *ei;
    ROAMestimation::BasePriorEdgeInterface *pi;

    if ((ei = dynamic_cast<ROAMestimation::GenericEdgeInterface *>(e)) != NULL) {
      std::map<std::string, SlotRandomLogger *>::iterator l_it = _loggers.find(
          ei->getCategory());

      if (l_it == _loggers.end()) {
        std::stringstream s;
        s << _basepath << "/" << ei->getCategory() << ".log";

        SlotRandomLogger *l = new SlotRandomLogger(s.str(), 16);

        std::pair<std::map<std::string, SlotRandomLogger *>::iterator, bool> ret =
            _loggers.insert(
                std::pair<std::string, SlotRandomLogger *>(ei->getCategory(),
                    l));

        l_it = ret.first;
      }

      l_it->second->logObject(*ei);
    } else if ((pi = dynamic_cast<ROAMestimation::BasePriorEdgeInterface *>(e))
        != NULL) {
      std::map<std::string, SlotRandomLogger *>::iterator l_it = _loggers.find(
          pi->getCategory());

      if (l_it == _loggers.end()) {
        std::stringstream s;
        s << _basepath << "/" << pi->getCategory() << ".log";

        SlotRandomLogger *l = new SlotRandomLogger(s.str(), 16);

        std::pair<std::map<std::string, SlotRandomLogger *>::iterator, bool> ret =
            _loggers.insert(
                std::pair<std::string, SlotRandomLogger *>(pi->getCategory(),
                    l));

        l_it = ret.first;
      }

      l_it->second->logObject(*pi);
    } else {
      // std::cerr << "[GraphLogger] Cannot log edge" << std::endl;
    }

  }

// After I have written all the vertices, I flush the files.

  std::map<std::string, SlotRandomLogger *>::const_iterator it =
      _loggers.begin();

  while (it != _loggers.end()) {
    it->second->flush();
    ++it;
  }

}

} /* namespace ROAMlog */
