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
 * EstimationEdgeCollectInterface.h
 *
 *  Created on: Nov 8, 2013
 *      Author: davide
 */

#ifndef ESTIMATIONEDGECOLLECTINTERFACE_H_
#define ESTIMATIONEDGECOLLECTINTERFACE_H_

/**
 * this class specifies the interface of the collect method which will be
 * implemented in all the templetized estimation edges (i.e. QuaternionGenericEdge)
 */

namespace ROAMestimation {

class EstimationEdgeCollectInterface {

public:
  virtual void collect(GenericVertex<ROAMfunctions::SE3V> *xtm2,
      GenericVertex<ROAMfunctions::SE3V> *xtm1,
      GenericVertex<ROAMfunctions::SE3V> *xt, double tstamp,
      GenericVertex<ROAMfunctions::Eucl1DV> *dt1,
      GenericVertex<ROAMfunctions::Eucl1DV> *dt2, const std::string &name,
      std::map<std::string, boost::shared_ptr<ParameterVerticesManager> > &params) = 0;

};

}

#endif /* ESTIMATIONEDGECOLLECTINTERFACE_H_ */
