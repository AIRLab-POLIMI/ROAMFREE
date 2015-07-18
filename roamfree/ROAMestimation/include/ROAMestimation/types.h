/*
 * types.h
 *
 *  Created on: Mar 20, 2014
 *      Author: davide
 */

#ifndef TYPES_H_
#define TYPES_H_

#include <map>

#include "GenericVertex.h"

namespace ROAMestimation {

typedef GenericVertex<ROAMfunctions::SE3V> PoseVertex; //!< a RF vertex containing a pose
typedef GenericVertex<ROAMfunctions::Eucl1DV> DtVertex; //!< a RF vertex containing a dt
typedef std::map<double, PoseVertex *> PoseMap; //! a collection of poses (the key of the map is the timestamp)

}

#endif /* TYPES_H_ */
