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

#include "ParameterTemporaries.h"

namespace ROAMestimation
{
class GenericCalculator
{
public:
	GenericCalculator(std::vector<ParameterTemporaries>& params) :
	  _params(params), so(params[0].value), qos(params[1].value) { }

	virtual bool calculate(const Eigen::VectorXd& x2) = 0;
	virtual bool calculate(const Eigen::VectorXd& x1, const Eigen::VectorXd& x2,
				const double& Dt12) = 0;
	virtual bool calculate(const Eigen::VectorXd& x0, const Eigen::VectorXd& x1,
				const Eigen::VectorXd& x2, const double &Dt01,
				const double& Dt12) = 0;

	virtual ~GenericCalculator() {}

protected:
  
	const Eigen::VectorXd &so;
	const Eigen::VectorXd &qos;

	const static int _OFF = -1;

	std::vector<ParameterTemporaries>& _params;

};

}

#endif /* GENERICCALCULATOR_H_ */
