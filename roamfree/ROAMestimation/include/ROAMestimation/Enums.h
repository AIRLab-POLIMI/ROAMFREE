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
  RectangularObject,
  AnchoredRectangularObject,
  AnchoredRectangularObjectFirst,
  IMUintegralDeltaP,
  IMUintegralDeltaQ,
  PlanarConstraint,
  RotatingPushbroom
};

enum ParameterTypes {
  Euclidean1D,
  Euclidean2D,
  Euclidean3D,
  Quaternion,
  Matrix3D,
  SE3,
  PlaneDynamicModelParams
};

enum PriorEdgeTypes {
  Euclidean1DPrior,
  Euclidean3DPrior,
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
  AUGSTATE_N_COMPONENTS
};

enum SolverMethod {
  GaussNewton, LevenbergMarquardt
};

}

#endif /* ENUMS_H_ */
