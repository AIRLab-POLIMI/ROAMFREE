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
 * interfaceTypes.h
 *
 *  Created on: Mar 31, 2014
 *      Author: davide
 */

#ifndef INTERFACETYPES_H_
#define INTERFACETYPES_H_

#include <vector>
#include <boost/shared_ptr.hpp>

namespace ROAMestimation {

class FactorGraphFilter;

class PoseVertexWrapper;
class MeasurementEdgeWrapper;
class ParameterWrapper;

typedef boost::shared_ptr<ParameterWrapper> ParameterWrapper_Ptr;
typedef boost::shared_ptr<PoseVertexWrapper> PoseVertexWrapper_Ptr;
typedef boost::shared_ptr<MeasurementEdgeWrapper> MeasurementEdgeWrapper_Ptr;

typedef std::vector<ParameterWrapper_Ptr> ParameterWrapperVector;
typedef std::vector<PoseVertexWrapper_Ptr> PoseVertexWrapperVector;
typedef std::vector<MeasurementEdgeWrapper_Ptr> MeasurementEdgeWrapperVector;

typedef boost::shared_ptr<ParameterWrapperVector> ParameterWrapperVector_Ptr;
typedef boost::shared_ptr<PoseVertexWrapperVector> PoseVertexWrapperVector_Ptr;
typedef boost::shared_ptr<MeasurementEdgeWrapperVector> MeasurementEdgeWrapperVector_Ptr;

}

#endif /* INTERFACETYPES_H_ */
