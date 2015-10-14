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
 * IMUIntegralHandler.h
 *
 *  Created on: Sep 3, 2014
 *      Author: davide
 */

#ifndef IMUINTEGRALHANDLER_H_
#define IMUINTEGRALHANDLER_H_

#include "ROAMestimation/ROAMestimation.h"

#include "IMUIntegrator.h"

namespace ROAMimu {

class IMUIntegralHandler {
  public:

    /**
     *  \brief constructor
     *
     *  configure proper sensors and accessory parameters in the filter
     *
     *  @param f the filter
     *  @param name the base name for the sensors to be configured
     *  @param N the number of IMU measurements to be integrated
     *  @param dt the IMU period (1/frequency)
     *  @param ba_par pointer to parameter for accelerometer bias (parameter name MUST be <name>_Ba)
     *  @param bw_par pointer to parameter for gyroscope bias (parameter name MUST be <name>_Bw)
     *  @param T_OS 7elem vector, transformation from Odometric center to IMU, if not provided identity will be used
     */
    IMUIntegralHandler(ROAMestimation::FactorGraphFilter* f,
        const std::string& name, int N, double dt,
        ROAMestimation::ParameterWrapper_Ptr ba_par,
        ROAMestimation::ParameterWrapper_Ptr bw_par,
        const Eigen::VectorXd &T_OS = Eigen::VectorXd());
    ~IMUIntegralHandler();

    /**
     *  \brief initialize the handler
     *
     *  @param isMaster if new pose vertices have to be added every N IMU readings
     *  @param t0 timestamp of first IMU reading
     *  @param x0 7elem vector, initial value for the first pose
     */
    void init(bool isMaster = true, double t0 = 0.0, const Eigen::VectorXd &x0 =
        Eigen::VectorXd());

    /**
     *  \brief toggles the pose predictor
     *
     *  next pose is predicted as a function of the last two and the relevant
     *  IMU readings. This can be disabled, so that the new pose vertex
     *  is initialized with the values of the previos one
     *
     *  @param enable if the predictor is enabled or not
     */
    void setPredictorEnabled(bool enable);

    /**
     *  \brief getters for the sensor noises
     *
     *  @return a reference to the 6x6 matrix, 3x3 block diagonal, storing the white noises affecting acc and gyro
     */

    inline Eigen::Matrix<double, 6, 6> &getSensorNoises() {
      return _sensorNoises;
    }
    inline const Eigen::Matrix<double, 6, 6> &getSensorNoises() const {
      return _sensorNoises;
    }

    /**
     *  \brief process an IMU measurement
     *
     *  timestamp is automatically generated incremented the last one by dt
     *
     *  @param za 3elem vector, accelerometer reading
     *  @param zw 3elem vector, gyroscope reading
     */
    bool step(double *za, double *zw);

    void getCurrentDeltaPosition(double *x);
    void getCurrendDeltaOrientation(double *q);

    ROAMestimation::ParameterWrapper_Ptr getAccelerometerBiasParameter() const {
      return _ba_par;
    }
    inline ROAMestimation::ParameterWrapper_Ptr getGyroscopeBiasParameter() const {
      return _bw_par;
    }

  protected:

    IMUIntegrator _itg;

    ROAMestimation::ParameterWrapper_Ptr _ba_par, _bw_par;

    bool _isMaster;
    ROAMestimation::PoseVertexWrapper_Ptr _x0, _x1, _x2;

    ROAMestimation::FactorGraphFilter *_filter;

    std::string _sensorNameDeltaP, _sensorNameDeltaQ;

    int _N;
    double _dt;

    int _cnt;
    bool isFirst;

    Eigen::Matrix<double, 6, 6> _sensorNoises;

    Eigen::VectorXd *_z12, *_z01; // keep pointers so that I can swap them
    Eigen::MatrixXd *_z12Cov, *_z01Cov;

    Eigen::VectorXd _zGyro;
    Eigen::MatrixXd _zGyroCov;

    bool _predictorEnabled;

  public:

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    ;

};

} /* namespace ROAMimu */

#endif /* IMUINTEGRALHANDLER_H_ */
