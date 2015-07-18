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
 * IMUIntegrator.h
 *
 *  Created on: Aug 18, 2014
 *      Author: davide
 */

#ifndef IMUINTEGRATOR_H_
#define IMUINTEGRATOR_H_

#include <Eigen/Dense>

namespace ROAMimu {

/**
 *
 * this class implements an IMU integration scheme as described in
 *
 * Efficient Integration of Inertial Ovservations into Visual SLAM without Initialization
 * Todd Lupton and Salah Sukkarieh
 *
 */

class IMUIntegrator {

public:

	IMUIntegrator();

	/**
	 * performs an integration step
	 *
	 * @param za accelerometer reading
	 * @param zw gyroscope reading
	 * @param dt time step
	 * @param dir +1 positive time flow, -1 negative time flow
	 */
	void step(const double *za, const double *zw, double dt);

	/**
	 * reset internal temporaries so that a new integration can be started
	 *
	 * @param ba current estimate of the accelerometer bias
	 * @param bw current estimate of the gyroscope bias
	 * @param Q covariance matrix of the noises on accelerometer (first, 3x3) and gyroscope.
	 */
	void reset(const double *ba, const double *bw, const double *Q);

	/**
	 * substitute current integral [x, q] with [-q^-1 * x, q^-1] and updates
	 * the covariance and jacobian matrices accordingly
	 */
	void invertIntegral();

	/**
	 * gets the current delta position
	 *
	 * @param x a pointer to a array of size 3 to be filled with delta position
	 */
	void getDeltaPosition(double *x) const;

	/**
	 * gets the current delta orientation
	 *
	 * @param x a pointer to a array of size 4 to be filled with delta quaternion
	 */
	void getDeltaOrientation(double *q) const;

	/**
	 * gets the current derivative of the delta position with respect to the accelerometer bias
	 *
	 * @param J a pointer to storage for a 3x3 matrix in ROW major order
	 */
	void getJDeltaPositionWrtBa(double *J);

	/**
	 * gets the current derivative of the delta position with respect to the gyroscope bias
	 *
	 * @param J a pointer to storage for a 3x3 matrix in ROW major order
	 */
	void getJDeltaPositionWrtBw(double *J);

	/**
	 * gets the current derivative of the delta orientation with respect to the gyroscope bias
	 *
	 * @param J a pointer to storage for a 3x3 matrix in ROW major order
	 */
	void getJDeltaOrientationWrtBw(double *J);

	/**
	 * gets the current covariance of delta position
	 *
	 * @param R a pointer to storage for a 3x3 matrix
	 */
	void getDeltaPositionCovariance(double *R);

	/**
	 * gets the current covariance of delta orientation
	 *
	 * @param R a pointer to storage for a 3x3 matrix in ROW major order
	 */
	void getDeltaOrientationCovariance(double *R);

	/**
	 * gets the accelerometer bias employed in integration
	 *
	 * @param ba a pointer to an array of size 3
	 */
	void getAccelerometerBias(double *ba);

	/**
	 * gets the gyroscope bias employed in integration
	 *
	 * @param bw a pointer to an array of size 3
	 */
	void getGyroscopeBias(double *bw);

protected:

	int _cnt; //!< how many integration steps have been performed

	Eigen::Matrix<double, 10, 1> x; //!< current state: {x,y,z, qw,qx,qy,qz, vx, vy, vz}.

	Eigen::Vector3d ba, bw; //!< estimates of the accelerometer and gyroscope biases at the beginning of integration

	Eigen::Matrix<double, 15, 15> _J, _R; //!< Jacobian and Covariance matrices

	/*
	 * thge next two matrices are sparse and are initialized to zero
	 * once for all in the constructor.
	 */

	Eigen::Matrix<double, 15, 15> _F; //!< single step Jacobian of the next state wrt old state
	Eigen::Matrix<double, 15, 6> _G; //!< single step Jacobian of the next state wrt measure

	Eigen::Matrix<double, 6, 6> _Q; //!< noise covariance matrix on imu readings, variables order: acceleration, angular velocity

public:

	EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

};

} /* namespace ROAMFREE */

#endif /* IMUINTEGRATOR_H_ */
