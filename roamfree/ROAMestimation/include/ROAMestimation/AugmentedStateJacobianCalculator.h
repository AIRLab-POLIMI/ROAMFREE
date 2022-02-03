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

#ifndef AUGMENTEDSTATEJACOBIANCALCULATOR_H_
#define AUGMENTEDSTATEJACOBIANCALCULATOR_H_

#         include "GenericCalculator.h"
#         include "Enums.h"

namespace ROAMestimation {

template<typename Derived, int ORDER>
class AugmentedStateJacobianCalculator: public GenericCalculator {
  public:
    AugmentedStateJacobianCalculator(std::vector<ParameterTemporaries>& params,
        int y, int x, Eigen::MatrixBase<Derived> &J) :
        GenericCalculator(params), x(x), y(y), J(J) {
      // J is sparse! We clear the non zero entries without freeing the memory
      J.setZero();
    }

    /**
     * this function computes the (i,j) block of the Jacobian matrix of _x wrt _vertices()
     * and stores the results in J
     *
     * N.B. in the following it is not that everything is full of DENSE code... :)
     * a lot of files are empty or near empty
     *
     * @param y is the row [x,q,v,omega,a,alpha,dispx,dispq,imuintdp,imuintdq], y in 0-9
     * @param x is the col, i.e. the vertex, [x(t), x(t-1), x(t-2), SO, qOS]
     *          note that depending on the MT::_ORDER of the edge, older poses and dt may be missing.
     */

    bool calculate(const Eigen::VectorXd& x2) {

      if (x == 0) //TODO non si può mettere nell'altro switch?
          { // with respect to x(t)
        switch (y) {
        case POSITION: {
#         include "generated/BackwardAugmentedStateEstimator_v7_JAugPOSEX2.cppready"
          return true;
        }
        case ORIENTATION: {
#         include "generated/BackwardAugmentedStateEstimator_v7_JAugQX2.cppready"
          return true;
        }
        }
      }

      switch (y) {
      case POSITION: {
        switch (x) {
        case (ORDER + 1): {
#         include "generated/BackwardAugmentedStateEstimator_v7_JAugPOSESO.cppready"
          return true;
        }
        case (ORDER + 2): {
#         include "generated/BackwardAugmentedStateEstimator_v7_JAugPOSEqOS.cppready"
          return true;
        }
        }

        break;
      }
      case ORIENTATION: {
        switch (x) {
        case (ORDER + 1): {
#         include "generated/BackwardAugmentedStateEstimator_v7_JAugQSO.cppready"
          return true;
        }
        case (ORDER + 2): {
#         include "generated/BackwardAugmentedStateEstimator_v7_JAugQqOS.cppready"
          return true;
        }
        }

        break;
      }
      }

      return false;
    }

    bool calculate(const Eigen::VectorXd& x1, const Eigen::VectorXd& x2,
        const double& Dt12) {
      if (calculate(x2))
        return true;

      if (x == 0) //TODO idem come sopra...
          { // with respect to x(t)
        switch (y) {
        case LINEARVELOCITY: {
          assert(ORDER > 0);
#         include "generated/BackwardAugmentedStateEstimator_v7_JAugVX2.cppready"
          return true;
        }
        case ANGULARVELOCITY: {
          assert(ORDER > 0);
#         include "generated/BackwardAugmentedStateEstimator_v7_JAugWX2.cppready"
          return true;
        }
        case DELTA_POSITION: {
          assert(ORDER > 0);
#         include "generated/BackwardAugmentedStateEstimator_v7_JAugDispX2.cppready"
          return true;
        }
        case DELTA_ORIENTATION: {
          assert(ORDER > 0);
#         include "generated/BackwardAugmentedStateEstimator_v7_JAugDispQX2.cppready"
          return true;
        }

        }
      }

      if (x == 1)  //TODO idem come sopra...
          { // with respect to x(t-1)
        switch (y) {
        case POSITION: {
#         include "generated/BackwardAugmentedStateEstimator_v7_JAugPOSEX1.cppready"
          return true;
        }
        case ORIENTATION: {
#         include "generated/BackwardAugmentedStateEstimator_v7_JAugQX1.cppready"
          return true;
        }
        case LINEARVELOCITY: {
#         include "generated/BackwardAugmentedStateEstimator_v7_JAugVX1.cppready"
          return true;
        }
        case ANGULARVELOCITY: {
#         include "generated/BackwardAugmentedStateEstimator_v7_JAugWX1.cppready"
          return true;
        }
        case DELTA_POSITION: {
#         include "generated/BackwardAugmentedStateEstimator_v7_JAugDispX1.cppready"
          return true;
        }
        case DELTA_ORIENTATION: {
#         include "generated/BackwardAugmentedStateEstimator_v7_JAugDispQX1.cppready"
          return true;
        }

        }
      }

      switch (y) {
      case LINEARVELOCITY: {
        switch (x) {
        case (ORDER + 1): {
#         include "generated/BackwardAugmentedStateEstimator_v7_JAugVSO.cppready"
          return true;
        }
        case (ORDER + 2): {
#         include "generated/BackwardAugmentedStateEstimator_v7_JAugVqOS.cppready"
          return true;
        }
        }

        break;
      }
      case ANGULARVELOCITY: {
        switch (x) {
        case (ORDER + 1): {
#         include "generated/BackwardAugmentedStateEstimator_v7_JAugWSO.cppready"
          return true;
        }
        case (ORDER + 2): {
#         include "generated/BackwardAugmentedStateEstimator_v7_JAugWqOS.cppready"
          return true;
        }
        }

        break;
      }
      case DELTA_POSITION: {
        switch (x) {
        case (ORDER + 1): {
#         include "generated/BackwardAugmentedStateEstimator_v7_JAugDispSO.cppready"
          return true;
        }
        case (ORDER + 2): {
#         include "generated/BackwardAugmentedStateEstimator_v7_JAugDispqOS.cppready"
          return true;
        }
        }

        break;
      }
      case DELTA_ORIENTATION: {
        switch (x) {
        case (ORDER + 1): {
#         include "generated/BackwardAugmentedStateEstimator_v7_JAugDispQSO.cppready"
          return true;
        }
        case (ORDER + 2): {
#         include "generated/BackwardAugmentedStateEstimator_v7_JAugDispQqOS.cppready"
          return true;
        }
        }

        break;
      }

      }

      return false;
    }

    bool calculate(const Eigen::VectorXd& x0, const Eigen::VectorXd& x1,
        const Eigen::VectorXd& x2, const double &Dt01, const double& Dt12) {
      if (calculate(x1, x2, Dt12))
        return true;

      if (x == 0) { // with respect to x(t)
        switch (y) {
        case ACCELERATION: {
          assert(ORDER > 1);
#         include "generated/BackwardAugmentedStateEstimator_v7_JAugAX2.cppready"
          return true;
        }
        case ANGULARACCELERATION: {
          assert(ORDER > 1);
#         include "generated/BackwardAugmentedStateEstimator_v7_JAugAlphaX2.cppready"
          return true;
        }
        case IMUINT_DELTAPOSE: {
          assert(ORDER > 1);

          const double gravity = _params[2].value(0);

#         include "generated/BackwardAugmentedStateEstimator_v7_JAugIMUintdPX2.cppready"
          return true;
        }
        case PREVIOUS_ORIENTATION: {
          assert(ORDER > 1);
          // do nothing
          return true;
        }
        case PREVIOUS_LINEARVELOCITY: {
          assert(ORDER > 1);
          // do nothing
          return true;
        }
        case PREVIOUS_ANGULARVELOCITY: {
          assert(ORDER > 1);
          // do nothing
          return true;
        }
        }
      }

      if (x == 1) { // with respect to x(t-1)
        switch (y) {
        case ACCELERATION: {
          assert(ORDER > 1);
#         include "generated/BackwardAugmentedStateEstimator_v7_JAugAX1.cppready"
          return true;
        }
        case ANGULARACCELERATION: {
          assert(ORDER > 1);
#         include "generated/BackwardAugmentedStateEstimator_v7_JAugAlphaX1.cppready"
          return true;
        }
        case IMUINT_DELTAPOSE: {
          assert(ORDER > 1);

          const double gravity = _params[2].value(0);

#         include "generated/BackwardAugmentedStateEstimator_v7_JAugIMUintdPX1.cppready"
          return true;
        }
        case PREVIOUS_ORIENTATION: {
          assert(ORDER > 1);
#         include "generated/BackwardAugmentedStateEstimator_v7_JAugQprevX1.cppready"
          return true;
        }
        case PREVIOUS_LINEARVELOCITY: {
          assert(ORDER > 1);
#         include "generated/BackwardAugmentedStateEstimator_v7_JAugVprevX1.cppready"
          return true;
        }
        case PREVIOUS_ANGULARVELOCITY: {
          assert(ORDER > 1);
#         include "generated/BackwardAugmentedStateEstimator_v7_JAugWprevX1.cppready"
          return true;
        }
        }
      }

      if (x == 2) { // with respect to x(t-2)
        switch (y) {
        case POSITION: {
#         include "generated/BackwardAugmentedStateEstimator_v7_JAugPOSEX0.cppready"
          return true;
        }
        case ORIENTATION: {
#         include "generated/BackwardAugmentedStateEstimator_v7_JAugQX0.cppready"
          return true;
        }
        case LINEARVELOCITY: {
#         include "generated/BackwardAugmentedStateEstimator_v7_JAugVX0.cppready"
          return true;
        }
        case ANGULARVELOCITY: {
#         include "generated/BackwardAugmentedStateEstimator_v7_JAugWX0.cppready"
          return true;
        }
        case DELTA_POSITION: {
#         include "generated/BackwardAugmentedStateEstimator_v7_JAugDispX0.cppready"
          return true;
        }
        case DELTA_ORIENTATION: {
#         include "generated/BackwardAugmentedStateEstimator_v7_JAugDispQX0.cppready"
          return true;
        }
        case ACCELERATION: {
          assert(ORDER > 1);
#         include "generated/BackwardAugmentedStateEstimator_v7_JAugAX0.cppready"
          return true;
        }
        case ANGULARACCELERATION: {
          assert(ORDER > 1);
#         include "generated/BackwardAugmentedStateEstimator_v7_JAugAlphaX0.cppready"
          return true;
        }
        case IMUINT_DELTAPOSE: {
          assert(ORDER > 1);

          const double gravity = _params[2].value(0);

#         include "generated/BackwardAugmentedStateEstimator_v7_JAugIMUintdPX0.cppready"
          return true;
        }
        case PREVIOUS_ORIENTATION: {
          assert(ORDER > 1);
#         include "generated/BackwardAugmentedStateEstimator_v7_JAugQprevX0.cppready"
          return true;
        }
        case PREVIOUS_LINEARVELOCITY: {
          assert(ORDER > 1);
#         include "generated/BackwardAugmentedStateEstimator_v7_JAugVprevX0.cppready"
          return true;
        }
        case PREVIOUS_ANGULARVELOCITY: {
          assert(ORDER > 1);
#         include "generated/BackwardAugmentedStateEstimator_v7_JAugWprevX0.cppready"
          return true;
        }
        }
      }

      switch (y) {
      case ACCELERATION: {
        assert(ORDER > 1);
        switch (x) {
        case (ORDER + 1): {
#         include "generated/BackwardAugmentedStateEstimator_v7_JAugASO.cppready"
          return true;
        }
        case (ORDER + 2): {
#         include "generated/BackwardAugmentedStateEstimator_v7_JAugAqOS.cppready"
          return true;
        }
        }

        break;
      }
      case ANGULARACCELERATION: {
        assert(ORDER > 1);
        switch (x) {
        case (ORDER + 1): {
#         include "generated/BackwardAugmentedStateEstimator_v7_JAugAlphaSO.cppready"
          return true;
        }
        case (ORDER + 2): {
#         include "generated/BackwardAugmentedStateEstimator_v7_JAugAlphaqOS.cppready"
          return true;
        }
        }

        break;
      }
      case IMUINT_DELTAPOSE: {
        assert(ORDER > 1);
        switch (x) {
        case (ORDER + 1): {
#         include "generated/BackwardAugmentedStateEstimator_v7_JAugIMUintdPSO.cppready"
          return true;
        }
        case (ORDER + 2): {

          const double gravity = _params[2].value(0);

#         include "generated/BackwardAugmentedStateEstimator_v7_JAugIMUintdPqOS.cppready"
          return true;
        }
        }

        break;
      }
      case PREVIOUS_ORIENTATION: {
        assert(ORDER > 1);
        switch (x) {
        case (ORDER + 1): {
#         include "generated/BackwardAugmentedStateEstimator_v7_JAugQprevSO.cppready"
          return true;
        }
        case (ORDER + 2): {
#         include "generated/BackwardAugmentedStateEstimator_v7_JAugQprevqOS.cppready"
          return true;
        }
        }

        break;
      }
      case PREVIOUS_LINEARVELOCITY: {
        assert(ORDER > 1);
        switch (x) {
        case (ORDER + 1): {
#         include "generated/BackwardAugmentedStateEstimator_v7_JAugVprevSO.cppready"
          return true;
        }
        case (ORDER + 2): {
#         include "generated/BackwardAugmentedStateEstimator_v7_JAugVprevqOS.cppready"
          return true;
        }
        }

        break;
      }
      case PREVIOUS_ANGULARVELOCITY: {
        assert(ORDER > 1);
        switch (x) {
        case (ORDER + 1): {
#         include "generated/BackwardAugmentedStateEstimator_v7_JAugWprevSO.cppready"
          return true;
        }
        case (ORDER + 2): {
#         include "generated/BackwardAugmentedStateEstimator_v7_JAugWprevqOS.cppready"
          return true;
        }
        }

        break;
      }
      }

      return false;
    }

  private:
    //Data needed by the algorithm
    int x;
    int y;
    Eigen::MatrixBase<Derived>& J;

};

}

#endif /* AUGMENTEDSTATEJACOBIANCALCULATOR_H_ */
