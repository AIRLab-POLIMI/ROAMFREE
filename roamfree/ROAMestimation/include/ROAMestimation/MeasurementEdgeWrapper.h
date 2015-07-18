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
 * MeasurementEdgeWrapper.h
 *
 *  Created on: Mar 13, 2014
 *      Author: davide
 */

#ifndef MEASUREMENTEDGEWRAPPER_H_
#define MEASUREMENTEDGEWRAPPER_H_

namespace ROAMestimation {

class MeasurementEdgeWrapper {
public:
	/*
	 * \brief evaluates a prediction for the most recent pose in the edge
	 *
	 * The estimate for the  most recent pose involved in the edge is replaced
	 * with a prediction computed with the edge measurement model based
	 * on the measurement and possibly on the older poses (if they exist)
	 *
	 * @return false if the edge does not support prediction
	 */
	virtual bool predict() = 0;
};

} /* namespace ROAMestimation */

#endif /* MEASUREMENTEDGEWRAPPER_H_ */
