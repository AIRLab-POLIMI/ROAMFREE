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
 * AbsolutePoseM.h
 *
 *  Created on: Sept 11, 2014
 *      Author: davide
 */

#ifndef ABSOLUTEPOSEM_H_
#define ABSOLUTEPOSEM_H_

#include <iostream>
#include <string>
#include <Eigen/Dense>

namespace ROAMfunctions {

class AbsolutePoseM {

public:
	static const bool _usedComponents[];

	static const std::string _paramsNames[];
	static const int _nParams = 0;

	static const unsigned int _ORDER = 0;

	static const unsigned int _ERROR_SIZE = 6;
	static const unsigned int _NOISE_SIZE = 6;
	static const unsigned int _MEASUREMENT_SIZE = 7;

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
	bool error(const Eigen::VectorXd &x, double **, const Eigen::VectorXd& z,
			Eigen::MatrixBase<T> const &const_ret) {

		Eigen::MatrixBase<T> & err = const_cast<Eigen::MatrixBase<T>&>(const_ret);

		const static int _OFF = -1;

		static Eigen::VectorXd w(1);
		double sign;

		{
#			include "generated/AbsolutePose_TestW.cppready"
		}

		sign = w(0) < 0 ? -1.0 : 1.0;

#   include "generated/AbsolutePose_Err.cppready"

		return false;
	}

	template<typename T>
	bool errorJacobian(const Eigen::VectorXd &x, double **,
			const Eigen::VectorXd& z, int wrt,
			Eigen::MatrixBase<T> const &const_ret) {

		Eigen::MatrixBase<T> & J = const_cast<Eigen::MatrixBase<T>&>(const_ret);

		const static int _OFF = -1;

		static Eigen::VectorXd w(1);
		double sign;

		{
#			include "generated/AbsolutePose_TestW.cppready"
		}

		sign = w(0) < 0 ? -1.0 : 1.0;

		switch (wrt) {
		case -2: // jacobian wrt to x
		{
#     include "generated/AbsolutePose_JErrQ.cppready"
			return true; // it is the identity
			break;
		}
		case -1: // jacobian wrt to x
		{
#     include "generated/AbsolutePose_JErrPOSE.cppready"
			return true; // it is not the identity
			break;
		}
		case 0: // jacobian wrt to the noise
		{
#     include "generated/AbsolutePose_JErrNoises.cppready"
			return true;
			break;
		}

		}

		assert(false);
		return false;
	}
};

} /* namespace ROAMfunctions */
#endif /* ABSOLUTEPOSEM_H_ */

