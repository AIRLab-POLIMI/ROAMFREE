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

class PoseVertexWrapper;
class MeasurementEdgeWrapper;
class ParameterWrapper;

typedef boost::shared_ptr<ParameterWrapper> ParameterWrapper_Ptr;
typedef boost::shared_ptr<PoseVertexWrapper> PoseVertexWrapper_Ptr;
typedef boost::shared_ptr<MeasurementEdgeWrapper> MeasurementEdgeWrapper_Ptr;

typedef std::vector<PoseVertexWrapper_Ptr> PoseVertexWrapperVector;
typedef std::vector<MeasurementEdgeWrapper_Ptr> MeasurementEdgeWrapperVector;

typedef boost::shared_ptr<PoseVertexWrapperVector> PoseVertexWrapperVector_Ptr;
typedef boost::shared_ptr<MeasurementEdgeWrapperVector> MeasurementEdgeWrapperVector_Ptr;

}

#endif /* INTERFACETYPES_H_ */
