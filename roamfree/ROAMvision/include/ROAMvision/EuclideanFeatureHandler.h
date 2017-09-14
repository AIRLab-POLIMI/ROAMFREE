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
 * EuclideanFeatureHandler.h
 *
 *  Created on: Jan 6, 2016
 *      Author: davide
 */

#ifndef EUCLIDEANFEATUREHANDLER_H_
#define EUCLIDEANFEATUREHANDLER_H_

#include <map>
#include <vector>

#include <opencv/cv.h>

#include "ImageFeatureHandler.h"

#include "EuclideanTrackDescriptor.h"

namespace ROAMvision {

class EuclideanFeatureHandler: public ImageFeatureHandler {

  public:
    EuclideanFeatureHandler();

    virtual bool init(ROAMestimation::FactorGraphFilter* f,
        const std::string &name, const Eigen::VectorXd & T_OS,
        const Eigen::VectorXd & K, const Eigen::VectorXd & RD =
            Eigen::VectorXd::Zero(3), const Eigen::VectorXd & TD =
            Eigen::VectorXd::Zero(2));
    virtual bool addFeatureObservation(long int id, double t,
        const Eigen::VectorXd &z, const Eigen::MatrixXd &cov);

    virtual bool getFeaturePositionInWorldFrame(long int id,
        Eigen::VectorXd &lw) const;
    virtual bool getFeaturesIds(std::vector<long int> &to) const;
    virtual long int getNActiveFeatures() const;

    virtual void setTimestampOffsetTreshold(double dt);

    virtual void fixOlderPosesWRTVisibleFeatures();

    virtual std::string getFeatureParameterName(long int id) const;

  protected:

    inline const ObservationDescriptor &getNewestObservation(
        long int id) const {
      assert(_features.count(id) == 1);

      return _features.find(id)->second.zHistory.rbegin()->second;
    }

    typedef std::map<long int, EuclideanTrackDescriptor> FeatureMap;

    FeatureMap _features;

    // methods for track initialization
    // TODO: these are based on opencv, this dependency should be dropped
    bool initialize(const EuclideanTrackDescriptor &track,
        const Eigen::VectorXd &K, Eigen::VectorXd &Lw);
    int GaussNewton(const std::vector<cv::Mat> &cameras,
        const std::vector<cv::Point2f> &points, cv::Point3d init3Dpoint,
        cv::Point3d &optimizedPoint);
    int point2D3DJacobian(const std::vector<cv::Mat> &cameras,
        const cv::Mat &cur3Dpoint, cv::Mat &J, cv::Mat &hessian);

    void buildProjectionMatrix(const Eigen::VectorXd &T_WS, const cv::Mat &K,
        cv::Mat &projMat);

    ROAMestimation::ParameterWrapper_Ptr K_par;
};

} /* namespace ROAMvision */

#endif /* EUCLIDEANFEATUREHANDLER_H_ */
