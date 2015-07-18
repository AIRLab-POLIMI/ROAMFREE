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
