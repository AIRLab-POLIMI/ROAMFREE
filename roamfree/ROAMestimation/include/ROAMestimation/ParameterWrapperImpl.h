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
 * ParameterVertexWrapperImpl.h
 *
 *  Created on: May 13, 2014
 *      Author: davide
 */

#ifndef PARAMETERVERTEXWRAPPERIMPL_H_
#define PARAMETERVERTEXWRAPPERIMPL_H_

#include "ParameterWrapper.h"
#include "ParameterVerticesManager.h"

namespace ROAMestimation {

class ParameterWrapper_Impl: public ParameterWrapper {

protected:
	ParameterVerticesManager *_param;

public:
	ParameterWrapper_Impl(ParameterVerticesManager *param);

	inline virtual bool getFixed() const {
		return _param->fixed();
	}

	inline virtual void setFixed(bool isFixed) {
		_param->setFixed(isFixed);
	}

	virtual const Eigen::VectorXd &getEstimate() const;
	virtual const Eigen::VectorXd &getEstimate(double t) const;

	virtual void setEstimate(const Eigen::VectorXd &x, double t);
	virtual void setEstimate(const Eigen::VectorXd &x);

	virtual void setEstimate(double x, double t);
	virtual void setEstimate(double x);

  virtual void getValueAt(Eigen::VectorXd &ret, double t);

  inline virtual void setProcessModelType(ProcessTypes t) {
    _param->setProcessModelType(t);
  }
  inline virtual void setRandomWalkNoiseCov(const Eigen::MatrixXd &cov) {
    _param->setRandomWalkProcessNoiseCov(cov);
  }

};

} /* namespace ROAMestimation */

#endif /* PARAMETERVERTEXWRAPPERIMPL_H_ */
