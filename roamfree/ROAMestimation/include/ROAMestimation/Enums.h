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
 * Enums.h
 *
 *  Created on: Mar 28, 2013
 *      Author: davide
 */

#ifndef ENUMS_H_
#define ENUMS_H_

namespace ROAMestimation {

enum MeasTypes {
  AbsolutePosition,
  AbsolutePose,
  LinearVelocity,
  AngularVelocity,
  LinearAcceleration,
  AckermannOdometer,
  AckermannConstraint,
  TriskarOdometer,
  DifferentialDriveOdometer,
  GenericOdometer,
  PlaneDynamicModel,
  Displacement,
  VectorField,
  VectorFieldAsCompass,
  FixedFeaturePosition,
  FixedFeaturePose,
  ImagePlaneProjection,
  FramedHomogeneousPoint,
  IMUintegralDeltaP,
  IMUintegralDeltaQ,
  PlanarConstraint,
  QuadDynamicModel,
  PoseDerivative,
  LiDARTieFeatures,
  LiDAR2ImgTieFeatures,
  AbsoluteVelocity
};

enum ParameterTypes {
  Euclidean1D,
  Euclidean2D,
  Euclidean3D,
  Euclidean4D,
  Quaternion,
  Matrix3D,
  SE3,
  PlaneDynamicModelParams
};

enum PriorEdgeTypes {
  Euclidean1DPrior,
  Euclidean2DPrior,
  Euclidean3DPrior,
  Euclidean4DPrior,
  QuaternionPrior,
  SE3Prior,
  FHPPriorOnHomogeneousPoint
};

enum ProcessTypes {
  None,
  RandomWalk,
  GaussMarkov
};

enum InterpolationTypes {
  Linear,
  Lanczos
};

enum AugStateComponents {
  POSITION,
  ORIENTATION,
  LINEARVELOCITY,
  ANGULARVELOCITY,
  ACCELERATION,
  ANGULARACCELERATION,
  DELTA_POSITION,
  DELTA_ORIENTATION,
  IMUINT_DELTAPOSE,
  PREVIOUS_ORIENTATION,
  PREVIOUS_LINEARVELOCITY,
  PREVIOUS_ANGULARVELOCITY,
  AUGSTATE_N_COMPONENTS,
};

enum SolverMethod {
  GaussNewton, LevenbergMarquardt
};

enum FusionFrameTypes {
  TangentPlane,
  ECEF
};

}

#endif /* ENUMS_H_ */
