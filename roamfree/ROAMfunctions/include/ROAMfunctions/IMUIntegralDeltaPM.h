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
 * IMUImtegralM.h
 *
 *  Created on: Sept 9, 2014
 *      Author: davide
 */

#ifndef IMUINTEGRALDELTAPM_H_
#define IMUINTEGRALDELTAPM_H_

#include <Eigen/Dense>

namespace ROAMfunctions {

class IMUImtegralDeltaPM {

public:
	static const bool _usedComponents[];

	static const std::string _paramsNames[];
	static const int _nParams = 2;

	static const unsigned int _ORDER = 2;

	static const unsigned int _ERROR_SIZE = 3;
	static const unsigned int _NOISE_SIZE = 3;
	static const unsigned int _MEASUREMENT_SIZE = 27;

	const std::string* getParamsList() {
		return _paramsNames;
	}
	const int getNParams() {
		return _nParams;
	}

	bool predict(const Eigen::VectorXd &x, double **params,
			const Eigen::VectorXd& z, double dt, Eigen::VectorXd &xhat) {
		return false;
	}

	template<typename T>
	bool error(const Eigen::VectorXd &x, double **params,
			const Eigen::VectorXd& z, Eigen::MatrixBase<T> const &const_ret) {

		Eigen::Map<Eigen::VectorXd> ba(params[0], 3);
		Eigen::Map<Eigen::VectorXd> bw(params[1], 3);

		Eigen::MatrixBase<T> & err = const_cast<Eigen::MatrixBase<T>&>(const_ret);

		const static int _OFF = -1;

#   include "generated/IMUIntegralDeltaP_Err.cppready"

		return false;
	}

	template<typename T>
	bool errorJacobian(const Eigen::VectorXd &x, double **params,
			const Eigen::VectorXd& z, int wrt,
			Eigen::MatrixBase<T> const &const_ret) {

		Eigen::Map<Eigen::VectorXd> ba(params[0], 3);
		Eigen::Map<Eigen::VectorXd> bw(params[1], 3);

		Eigen::MatrixBase<T> & J = const_cast<Eigen::MatrixBase<T>&>(const_ret);

		const static int _OFF = -1;

		switch (wrt) {

		case -9: // jacobian wrt to IMUintdP
		{
#     include "generated/IMUIntegralDeltaP_JErrdP.cppready"
			return false; // "it is useless to evaluate me again"
			break;
		}

		case 0: // jacobian wrt to noises
		{
#			include "generated/IMUIntegralDeltaP_JErrNoises.cppready"
			return false; // it is the identity matrix
			break;
		}
		case 1: // jacobian wrt accelerometer bias
		{
 #  	include "generated/IMUIntegralDeltaP_JErrBa.cppready"
			return true;
			break;
		}
		case 2: // jacobian wrt gyroscope bias
		{
#			include "generated/IMUIntegralDeltaP_JErrBw.cppready"
			return true;
			break;
		}

		}

		assert(false);
		return false;
	}
};

} /* namespace ROAMfunctions */

#endif /* IMUINTEGRALDELTAPM_H_ */
