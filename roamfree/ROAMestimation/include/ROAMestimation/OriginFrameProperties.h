/*
 * Datum.h
 *
 *  Created on: Jul 2, 2021
 *      Author: davide
 */

#ifndef DATUM_H_
#define DATUM_H_

#include <Eigen/Dense>
#include "Enums.h"

/*! Specifies the properties of fusion frame (rotation, gravity model) */

/*!
  ROAMFREE assumes that your reference frame is ECEF (Earth-Centered Earth-Fixed), or in any case
  you are navigating around a body that has an oblate spheroid shape.

  The parameters specified in this class allow to configure the rotation rate of this body
  with respect to an inertial frame with the same origin, and the shape of the ellipsoid, so that
  the direction of the gravity vector can be computed as a function of the position.
*/

namespace ROAMestimation {

class OriginFrameProperties
{
    public:
        static OriginFrameProperties& getInstance()
        {
            static OriginFrameProperties instance;

            return instance;
        }

    public:
        OriginFrameProperties(OriginFrameProperties const&) = delete;
        void operator=(OriginFrameProperties const&) = delete;

        void evaluateGravityVectorAt(const Eigen::VectorXd &x, Eigen::Vector3d &gravityVector);
        void evaluateRLocalENUToWorldAt(const Eigen::VectorXd &x, Eigen::Matrix3d &Rltow);

        FusionFrameTypes frametype; //!< what is the type of fusion frame

        Eigen::Vector3d epshift; //!< three dimensional shift of the origin of the fusion frame with respect to the ellipsoid

        double epa, epb; //!< three dimensional shift of the origin of the fusion frame with respect to the center of the ellipsoid

        double earthrate; //!< the rotation rate of the fusion frame (e.g., ECEF) with respect to the inertial frame

    protected:
        const static int _OFF = -1;

    private:
        OriginFrameProperties();

};

}
#endif /* DATUM_H_ */
