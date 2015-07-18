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
 * GenericLinearConstraintFactory.h
 *
 *  Created on: Sep 26, 2013
 *      Author: davide
 */

#ifndef GENERICLINEARCONSTRAINTFACTORY_H_
#define GENERICLINEARCONSTRAINTFACTORY_H_

#include "g2o/core/hyper_graph.h"

#include "GenericLinearConstraint.h"

namespace ROAMestimation {

/*
 * \biref makes a Generic Linear Constraint to handle a set of node removal
 *
 */

class GenericLinearConstraintFactory {

public:

	/**
	 * \brief Builds a Generic Linear Constraint
	 *
	 * builds a generic linear constraint over the Markov blanket of the provided nodes
	 * which is equivalent in the current linearization point to all the one incident
	 * into the provided nodes.
	 */

	static GenericLinearConstraint *buildGLC(
			const std::set<g2o::HyperGraph::Vertex *> &toBeRemoved);

	static GenericLinearConstraint *buildGLC(
			const std::set<g2o::HyperGraph::Vertex *> &toBeRemoved,
			std::set<g2o::HyperGraph::Vertex *> &markovBlanket,
			std::set<g2o::HyperGraph::Edge *> &fallingEdges);

	/**
	 * finds out relevant nodes and edges regarding the node set n and
	 * fills the arrays passed as parameters
	 *
	 * @param n the vector of input nodes
	 * @param markovBlanket the markov blanket (only the free nodes, i.e. not fixed) of the nodes in n
	 * @param fallingEdges the edges which have to be removed because of removal of nodes in n
	 *
	 */
	static void getMarkovBlanket(const std::set<g2o::HyperGraph::Vertex *> &n,
			std::set<g2o::HyperGraph::Vertex *> &markovBlanket,
			std::set<g2o::HyperGraph::Edge *> &fallingEdges);

private:

	/**
	 * test if a give vertex is allowed to go in the vertex set of a GLC
	 *
	 * currently, IMU bias estimation is broken if the corresponding parameters
	 * are allowed to be under a GLC
	 */
	static bool isAllowedInGLC(const g2o::HyperGraph::Vertex *v);

};

} /* namespace ROAMestimation */
#endif /* GENERICLINEARCONSTRAINTFACTORY_H_ */
