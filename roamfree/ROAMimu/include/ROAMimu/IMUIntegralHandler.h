/*
 Copyright (c) 2013-2014 Politecnico di Milano.
 All rights reserved. This program and the accompanying materials
 are made available under the terms of the GNU Lesser Public License v3
 which accompanies this distribution, and is available at
 https://www.gnu.org/licenses/lgpl.html

 Contributors:
 Davide A. Cucci (cucci@elet.polimi.it)
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

    enum ParameterTypes {
      DoNotConfigureParameters,
      ConfigureConstantParameters,
      ConfigureLinearlyInterpolatedParameters,
      ConfigureLimitedBandwidthParameters
    };

    IMUIntegralHandler(int N, double dt, ParameterTypes parType = ConfigureConstantParameters);
    ~IMUIntegralHandler();

    void init(ROAMestimation::FactorGraphFilter* f, const std::string& name,
        const Eigen::VectorXd &T_OS, const Eigen::VectorXd &ba0, bool isbafixed,
        const Eigen::VectorXd &bw0, bool isbwfixed, const Eigen::VectorXd &x0,
        double t0);

    bool step(double *za, double *zw);

    void getCurrentDeltaPosition(double *x);
    void getCurrendDeltaOrientation(double *q);

    void setPredictorEnabled(bool enable);

    ROAMestimation::ParameterWrapper_Ptr getAccelerometerBiasParameter() const {
      return _ba_par;
    }
    inline ROAMestimation::ParameterWrapper_Ptr getGyroscopeBiasParameter() const {
      return _bw_par;
    }

    inline Eigen::Matrix<double, 6, 6> &getSensorNoises() {
      return _sensorNoises;
    }
    inline const Eigen::Matrix<double, 6, 6> &getSensorNoises() const {
      return _sensorNoises;
    }

  protected:

    IMUIntegrator _itg;

    ParameterTypes _parType;
    ROAMestimation::ParameterWrapper_Ptr _ba_par, _bw_par;

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
