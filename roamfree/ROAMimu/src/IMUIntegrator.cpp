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
 * IMUIntegrator.cpp
 *
 *  Created on: Aug 18, 2014
 *      Author: davide
 */

#include "IMUIntegrator.h"

#include <cstring>

#include <iostream>

namespace ROAMimu {

IMUIntegrator::IMUIntegrator() {
	_F.setZero();
	_G.setZero();
}

void IMUIntegrator::step(const double *za, const double *zw, double dt) {

	// wrap provided vectors with an Eigen object
	Eigen::Map<const Eigen::Matrix<double, 3, 1>> a(za), w(zw);

	// compute the next state
	Eigen::Matrix<double, 10, 1> xnext;

	const static int _OFF = -1;

	{
# 	include "generated/IMUIntegrationStep.cppready"
	}

	// update the single step jacobian matrices

	{
#   include "generated/IMUIntegrationFMatrix.cppready"
	}
	{
#   include "generated/IMUIntegrationGMatrix.cppready"
	}

	// update the overall covariance and jacobian matrices

	_J = _F*_J;
	_R = _F*_R*_F.transpose() + _G*_Q*_G.transpose();

	// increment step counter

	++_cnt;

	// update the current state

	x = xnext;

}

void IMUIntegrator::reset(const double *ba_ptr, const double *bw_ptr, const double *Q_ptr) {
	// reset the state, covariance and Jacobian matrices
	x.setZero();
	x(3) = 1; // qw = 1, identity quaternion

	_J = Eigen::Matrix<double, 15, 15>::Identity();

	_R.setZero();

	// update the internal bias estimates

	Eigen::Map<const Eigen::Vector3d> ba_in(ba_ptr);
	Eigen::Map<const Eigen::Vector3d> bw_in(bw_ptr);

	ba = ba_in;
	bw = bw_in;

	// update the internal covariance matrix

	Eigen::Map<const Eigen::Matrix<double, 6, 6> > Q_in(Q_ptr);

	_Q = Q_in;

	// reset the step couter

	_cnt = 0;
}

void IMUIntegrator::invertIntegral() {
	// compute the next state
	Eigen::Matrix<double, 10, 1> xinv;

	const static int _OFF = -1;

	{
# 	include "generated/IMUIntegrationInverse.cppready"
	}

	// update the single step jacobian matrices

	{
#   include "generated/IMUIntegrationFinvMatrix.cppready"
	}

	// update the overall covariance and jacobian matrices

	_J = _F*_J;
	_R = _F*_R*_F.transpose();

	// update the current state

	x = xinv;
}

void IMUIntegrator::getDeltaPosition(double* x_to) const {
	memcpy(x_to, x.data(), 3*sizeof(double));
}

void IMUIntegrator::getDeltaOrientation(double* q_to) const {
	memcpy(q_to, x.data()+3, 4*sizeof(double));
}

void IMUIntegrator::getJDeltaPositionWrtBa(double* J_to) {
	Eigen::Map<Eigen::Matrix<double, 3,3, Eigen::RowMajor> > Jmap(J_to);
	Jmap = _J.block(0,9,3,3);
}

void IMUIntegrator::getJDeltaPositionWrtBw(double* J_to) {
	Eigen::Map<Eigen::Matrix<double, 3,3, Eigen::RowMajor> > Jmap(J_to);
	Jmap = _J.block(0,12,3,3);
}

void IMUIntegrator::getJDeltaOrientationWrtBw(double* J_to) {
	Eigen::Map<Eigen::Matrix<double, 3,3, Eigen::RowMajor> > Jmap(J_to);
	Jmap = _J.block(3,12,3,3);
}

void IMUIntegrator::getDeltaPositionCovariance(double* R_to) {
	Eigen::Map<Eigen::Matrix3d> Rmap(R_to); // the Row/Column major order is not important since R is symmetric
	Rmap = _R.block(0,0,3,3);
}

void IMUIntegrator::getDeltaOrientationCovariance(double* R_to) {
	Eigen::Map<Eigen::Matrix3d> Rmap(R_to); // the Row/Column major order is not important since R is symmetric
	Rmap = _R.block(3,3,3,3);
}

void IMUIntegrator::getAccelerometerBias(double* ba_to) {
	memcpy(ba_to, ba.data(), 3*sizeof(double));
}

void IMUIntegrator::getGyroscopeBias(double* bw_to) {
	memcpy(bw_to, bw.data(), 3*sizeof(double));
}


} /* namespace ROAMFREE */

