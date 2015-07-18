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
 * SpatialIndex.h
 *
 *  Created on: Mar 20, 2014
 *      Author: davide
 */

#ifndef SPATIALINDEX_H_
#define SPATIALINDEX_H_

#include "types.h"

namespace ROAMestimation {

class SpatialIndex {
public:

	/**
	 * \brief builds a spatial index
	 *
	 * @param poses the set of poses among which there are the ones which have to go into the spatial index
	 *
	 * NOTES:
	 *
	 * how to test if a vertex has to be kept into the spatial index or not
	 *
	 * PoseVertex *v; // a pose vertex
	 * PoseVertexWrapper w(v);
	 * if (w.getKeepSpatialIndex() == true) ...
	 *
	 * how to get the {x, y, z, qw, qx, qy, qz} estimate of a pose vertex
	 *
	 * PoseVertex *v;
	 * const Eigen::Vector &x = v->estimate();
	 * if (std::sqrt( x(0)*x(0) + x(1)*x(1) ) < d ) ...
	 *
	 */

	void build(const PoseMap &poses);

	/**
	 * \brief performs a rande search query to the spatial index
	 *
	 * by means of the provided output iterator, this method fills a collection of the PoseVertex which are
	 * within d distance from x, with respect to a certain metric. The metric is to be defined in sublcasses.
	 *
	 * @param x a vector of 7 elements {x, y, z, qw, qx, qy, qz} encoding pose and orientation of the sample
	 * @param d max distance
	 * @param outIter the output_iterator by means of which this methods fills the output collection
	 *
	 * NOTES:
	 *
	 * how to add a PoseVertex to the output collection
	 *
	 * PoseVertex *toadd;
	 * OutputIterator outIter
	 * *(outIter++) = toadd;
	 */
	template <typename OutputIterator>
	void rangeSearch(const Eigen::VectorXd &x1, double d, OutputIterator outIter) {
		assert(x1.size() == 7); // people has to give me pose samples

		for (PoseMap::const_iterator it = _index.begin(); it !=_index.end(); ++it) {
			const Eigen::VectorXd &x2 = it->second->estimate();

			if (euclidean2DDistance(x1,x2) < d) {
				*(outIter++) = it->second;
			}
		}
	}

protected:
	PoseMap _index;

	double euclidean2DDistance(const Eigen::VectorXd &x1,
			const Eigen::VectorXd &x2) const ;
};

} /* namespace ROAMestimation */

#endif /* SPATIALINDEX_H_ */
