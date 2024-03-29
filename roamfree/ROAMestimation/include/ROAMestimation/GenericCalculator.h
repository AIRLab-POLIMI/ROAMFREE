/*
Copyright (c) 2013-2016 Politecnico di Milano.
All rights reserved. This program and the accompanying materials
are made available under the terms of the GNU Lesser Public License v3
which accompanies this distribution, and is available at
https://www.gnu.org/licenses/lgpl.html

Contributors:
    Davide A. Cucci (davide.cucci@epfl.ch)    
    Davide Tateo (davide.tateo@polimi.it)
*/

#ifndef GENERICCALCULATOR_H_
#define GENERICCALCULATOR_H_

#include "OriginFrameProperties.h"
#include "ParameterTemporaries.h"

namespace ROAMestimation
{
class GenericCalculator
{
public:
	GenericCalculator(std::vector<ParameterTemporaries>& params) :
	  _params(params),
	  so(params[0].value),
	  qos(params[1].value),
	  epshift(OriginFrameProperties::getInstance().epshift),
	  earthrate(OriginFrameProperties::getInstance().earthrate),
	  gravity(_params[2].value(0)){
	}

	virtual bool calculate(const Eigen::VectorXd& x2) = 0;
	virtual bool calculate(const Eigen::VectorXd& x1, const Eigen::VectorXd& x2,
				const double& Dt12) = 0;
	virtual bool calculate(const Eigen::VectorXd& x0, const Eigen::VectorXd& x1,
				const Eigen::VectorXd& x2, const double &Dt01,
				const double& Dt12) = 0;

	virtual ~GenericCalculator() {}

	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

protected:
  
	const Eigen::VectorXd &so;
	const Eigen::VectorXd &qos;

	Eigen::Vector3d gravityVector;

	const static int _OFF = -1;

	std::vector<ParameterTemporaries>& _params;

	const Eigen::Vector3d &epshift;
	const double &earthrate;
	const double &gravity;

};

}

#endif /* GENERICCALCULATOR_H_ */
