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
 * FactorGraphFilter.h
 *
 *  Created on: Mar 12, 2014
 *      Author: davide
 */

#ifndef FACTORGRAPHFILTER_H_
#define FACTORGRAPHFILTER_H_

#include <string>
#include <vector>
#include <boost/shared_ptr.hpp>

#include <Eigen/Dense>

#include "Enums.h"
#include "interfaceTypes.h"

namespace ROAMestimation {

class FactorGraphFilter {

  public:

    /* --------------------------- SOLVER LEVEL METHODS ---------------------------- */

    /**
     *  \brief Set the solver method, i.e. GaussNewton, LevenbergMarquardt
     */
    virtual void setSolverMethod(SolverMethod method) = 0;

    /**
     *  \brief set initial pose estimate and timestamp
     *
     * A call to this method is required before any addSequentialMeasurement call can be issued.
     * Conversely, it is not required if only addMeasurement methods are issued, since the
     * control of the pose window is left to the user
     *
     * @param x0 the initial pose estimate
     * @param t the timestamp the first pose refers to
     */
    virtual PoseVertexWrapper_Ptr setInitialPose(const Eigen::VectorXd &x0,
        double t) = 0;

    /**
     *  \brief set dead reckoning mode (defaults on)
     *
     * In dead reckoning mode, it is assumed that no complete information on the position/orientation
     * of the robot w.r.t. world is available, given the provided set of sensors. In this case,
     * a prior with unitary information is maintained on the oldest available pose and the computed
     * covariance for the pose estimates becomes relative.
     *
     * @param deadReckoning mode to be set.	 *
     */
    virtual void setDeadReckoning(bool deadReckoning) = 0;

    /**
     *  \brief enables logging
     *
     * Maintain log files of every meaningful quantity, such as, readings, error function values, pose estimates
     * in the provided folder. This could slow down the pose tracking loop. It is suggested to use a RAM
     * location such as /tmp/roamfree, as long as sufficient memory is available
     *
     * @param folder the folder where the log files are stored. No '/' at the end of the path
     */
    virtual void setLowLevelLogging(bool lowLevelLogging, std::string folder =
        "/tmp/roamfree") = 0;

    /**
     *  \brief gets the current window length
     *
     * returns the difference between the timestamp of the newest pose and the one of the oldest
     */
    virtual double getWindowLenght() = 0;

    /* --------------------------- SENSOR LEVEL METHODS ---------------------------- */

    /**
     *  \brief Confiugres a logical sensor.
     *
     * This method tells the FactorGraphFilter to configure a new logical sensor
     *
     * @param name the name of the sensor. It is the sensor unique reference.
     * @param type the sensor measurement domain
     * @param isMaster the master sensor is the one which triggers new pose vertices to be added to the graph.
     * @param isSequential if the filter has to handle the vertex which incoming measurements are related to.
     */
    virtual bool addSensor(const std::string &name, MeasTypes type,
        bool isMaster, bool isSequential) = 0;

    /*
     *  \brief set the transofrmation from robot to sensor frame
     *
     * @param sensor the name of the sensor
     * @param s the transformation in the form [x, ty, tz, qw, qx, qy, qz]
     *
     * @return true on success
     */
    virtual bool setSensorFrame(const std::string &sensor,
        const Eigen::VectorXd &s) = 0;

    /*
     *  \brief allows to share frame parameters between sensors
     *
     *  @param from the name and the sensor donor (the one that already has the parameter defined)
     *  @param to the sensor receiver
     */
    virtual bool shareSensorFrame(const std::string &from,
        const std::string &to) = 0;

    /*
     *  \brief allows to share one parameter among multiple sensors (e.g. camera calibration matrices)
     *
     *  @param from the name and the sensor donor (the one that already has the parameter defined)
     *  @param to the sensor receiver
     */
    virtual bool shareParameter(const std::string &from,
        const std::string &to) = 0;

    /**
     *  \brief Toggles robust kernel.
     *
     * Set or unset a robust window for the selected sensor. this affects all the measurement added post to this action
     *
     * @param sensor the target sensor
     * @param enabled if the robust kernel has to be enabled or disabled
     * @param huberWidth the width of the Huber window. Meaningful only if enabled == true. Roughly, if err > sigma * huberWidth, it is scaled so that its influence is linear and no longer squared.
     */
    virtual bool setRobustKernel(const std::string &sensor, bool enabled,
        double huberWidth) = 0;

    /**
     *  \brief ugly suff to be removed
     *
     * Since we keep separate each component of the misalignment parameter, so that one can enable the
     * estimation of only one, e.g. the rotation around the z axis, there is nothing that prevents
     * qx^2+qy^2+qz^2 to be greather than 1 and thus sqrt(1 - (qx^2+qy^2+qz^2) ) to be nan.
     *
     * Until a better solution which both allows to estimate only one component of the misalignment while
     * maintaining the quaternion health, we have to include this guard
     */
    virtual bool addMisalignmentGuard(const std::string &sensor) = 0;

    /* --------------------------- PARAMETER LEVEL METHODS ---------------------------- */

    /**
     *  \brief adds a time-invariant parameter.
     *
     * This adds a parameter to be used by some sensor error function. This parameter is supposed to be independent with respect to time.
     *
     * @param type parameter domain
     * @param name the name of this parameter. It is its unique reference. It *has* to be <sensor-name>_<param-name> as specified in the sensor error function.
     * @param t allows to specify a timestamp for the parameter (e.g., time of creation for landmarks)
     * @param x0 initial guess
     * @param isFixed specifies if the parameter is fixed to its initial guess or if its value has to be tracked.
     */
    virtual ParameterWrapper_Ptr addConstantParameter(ParameterTypes type,
        const std::string &name, double t, const Eigen::VectorXd &x0,
        bool isFixed) = 0;
    virtual ParameterWrapper_Ptr addConstantParameter(ParameterTypes type,
        const std::string &name, const Eigen::VectorXd &x0, bool isFixed) = 0;
    virtual ParameterWrapper_Ptr addConstantParameter(const std::string &name,
        double x0, bool isFixed) = 0;

    /**
     *  \brief makes a parameter vertex out of a pose vertex
     *
     * A parameter can be made out of a pose vertex. The underneath vertex in the pose graph is not duplicated. Fundamental for edges such as FramedHomogeneousPoint
     *
     * @param pose the pose vertex out of which a parameter has to be created
     * @param name the name of this paramete. It is its unique reference. It *has* to be <sensor-name>_<param-name> as specified in the sensor error function.
     */

    virtual ParameterWrapper_Ptr poseVertexAsParameter(
        PoseVertexWrapper_Ptr pose, const std::string &name) = 0;

    /**
     *  \brief adds a limited bandwith parameter.
     *
     * This adds a parameter to be used by some sensor error function.
     *
     * Its value depends on time and for each t it is computed convolving its discrete samples with a Lanczos window
     *
     * @param name the name of this parameter. It is its unique reference. It *has* to be <sensor-name>_<param-name> as specified in the sensor error function.
     * @param x0 initial guess
     * @param isFixed specifies if the parameter is fixed to its initial guess or if its value has to be tracked.
     * @param bandwidth the highest frequency at which the parameter is supposed to change, in Hertz.
     * @param a is the order of the Lanczos window employed to interpolate the pose vertices. a = 2 or a = 3 is a good choice
     */
    virtual ParameterWrapper_Ptr addLimitedBandwithParameter(
        ParameterTypes type, const std::string &name, const Eigen::VectorXd &x0,
        bool isFixed, double bandwidth, int a) = 0;
    virtual ParameterWrapper_Ptr addLimitedBandwithParameter(
        const std::string &name, double x0, bool isFixed, double bandwidth,
        int a) = 0;

    /**
     *  \brief add a linearly interpolated Euclidean parameter
     *
     *  This adds a parameter to be used by some sensor error function.
     *
     * Its value depends on time and for each t it is computed interpolating between the two samples for which the timestamp is nearest wrt t
     *
     * @param name the name of this parameter. It is its unique reference. It *has* to be <sensor-name>_<param-name> as specified in the sensor error function.
     * @param x0 initial guess
     * @param isFixed specifies if the parameter is fixed to its initial guess or if its value has to be tracked.
     * @param spacing the time spacing between samples
     */

    virtual ParameterWrapper_Ptr addLinearlyInterpolatedParameter(
        ParameterTypes type, const std::string &name, const Eigen::VectorXd &x0,
        bool isFixed, double spacing) = 0;
    virtual ParameterWrapper_Ptr addLinearlyInterpolatedParameter(
        const std::string &name, double x0, bool isFixed, double spacing) = 0;

    /**
     *
     * \brief add a parameter blender
     *
     * A parameter blender is a parameter whose value is the sum of the
     * ones passed in constructor. It is a virtual parameter, in the sense that
     * no extra vertices are cretaed and it inherits the properties of the
     * existing parameters it blends.
     *
     * @param type the type of the parameter, must match each one in toblend
     * @param name the name of this parameter.
     * @oaram toblend vector of parameters to be blended.
     */

    virtual ParameterWrapper_Ptr addParameterBlender(ParameterTypes type,
        const std::string &name, ParameterWrapperVector_Ptr toblend) = 0;

    /**
     *   \brief parameter getter
     *
     * Returns a parameter wrapper given the parameter name
     *
     * @param name the name of the parameter to retrieve
     *
     * @return a ParameterWrapper_Ptr in case of success, null pointer in case of failure.
     */
    virtual ParameterWrapper_Ptr getParameterByName(
        const std::string &name) = 0;

    /* --------------------------- POSES AND EDGES LEVEL METHODS ---------------------- */

    /**
     * \brief adds a new pose node at time t.
     *
     * This method may fail if there exist a pose vertex whose timestamp is nearer than 1e-6s
     * with respect to the provided one. Poses which are too near in time may lead to an ill
     * conditioned hessian matrix.
     *
     * @param t the pose timestamp
     *
     * @return a wrapper to the created pose or NULL in case of failure.
     */
    virtual PoseVertexWrapper_Ptr addPose(double t) = 0;

    /**
     *  \brief Adds a measurement.
     *
     * This is a low level method to feed the FactorGraphFilter with sensor readings. The edge of maximum order
     * is directed from v0 to v1 to v2, so v0 is the oldest vertex.
     *
     * @param sensorName the name of the sensor this reading comes from.
     * @param timestamp the timestamp of the sensor reading
     * @param z the sensor reading
     * @param cov the covariance matrix associated to this reading
     * @param v2 first vertex (i.e. newest)
     * @param v1 second vertex, NULL if the measurement type is of order 0
     * @param v0 third vertex (i.e. oldes) NULL if the measurement type is of order 0 or 1
     *
     * @return a wrapper to the created edge, or NULL in case of failure.
     */
    virtual MeasurementEdgeWrapper_Ptr addMeasurement(
        const std::string& sensorName, double timestamp,
        const Eigen::VectorXd &z, const Eigen::MatrixXd &cov,
        PoseVertexWrapper_Ptr v2, PoseVertexWrapper_Ptr v1 =
            PoseVertexWrapper_Ptr(), PoseVertexWrapper_Ptr v0 =
            PoseVertexWrapper_Ptr()) = 0;

    /**
     *  \brief Adds a measurement for a sequential sensor.
     *
     * This method manages the poses which have to be connected by the provided measurement
     *
     * @param sensorName the name of the sensor this reading comes from.
     * @param timestamp the timestamp of the sensor reading
     * @param z the sensor reading
     * @param cov the covariance matrix associated to this reading
     *
     * @return a wrapper to the created edge, or NULL in case of failure.
     */
    virtual MeasurementEdgeWrapperVector_Ptr addSequentialMeasurement(
        const std::string& sensorName, double timestamp,
        const Eigen::VectorXd &z, const Eigen::MatrixXd &cov) = 0;

    /**
     *  \brief Handles the queue of deferred measurements
     *
     * When non master measurements are added, it might be that the corresponding poses
     * are not available in the graph (i.e., they are newer with resepct to the newest pose)
     *
     * This method is to be called when new poses are manually added to the graph
     */
    virtual MeasurementEdgeWrapperVector_Ptr handleDeferredMeasurements() = 0;

    /**
     *  \brief Marginalize the supplied nodes
     *
     * The nodes provided are removed from the graph and an equivalent
     * linear constraint is added over their markov blanket.
     *
     * After this call the referenced nodes will be deleted, thus the
     * supplied PoseVertexWrapper pointers will be invalid.
     */
    virtual bool marginalizeNodes(std::vector<PoseVertexWrapper *> &nodes) = 0;

    /**
     * \brief Marginalize old nodes
     *
     * The oldest nodes in the pose window are marginalized so that the overall
     * pose window length is lower than l seconds
     *
     * @param l the lenght of the resulting pose window
     */
    virtual bool marginalizeOldNodes(double l) = 0;

    /**
     * \ brief Forget old nodes
     *
     * The oldest nodes in the pose window are removed and the involved edges removed
     * so thast the overall window length is lower than l seconds
     *
     * @param l the lenght of the resulting pose window
     */
    virtual bool forgetOldNodes(double l) = 0;

    /* --------------------------- PRIOR CONTROL METHODS ------------------------------ */

    /**
     *  \brief adds a prior on a pose vertex
     *
     * Adds a constraint specifying prior knowledge on the value of a pose vertex
     *
     * @param type the prior edge type
     * @param pose the pose vertex wrapper
     * @param x0 the mean of the gaussian prior
     * @param cov the covariance of the gaussian prior
     */
    virtual MeasurementEdgeWrapper_Ptr addPriorOnPose(
        PoseVertexWrapper_Ptr pose, const Eigen::VectorXd &x0,
        const Eigen::MatrixXd &cov) = 0;

    /**
     *  \brief add a prior edge on a constant parameter
     *
     * Adds a constraint specifying prior knowledge on the value of a constant parameter
     *
     * @param type the prior edge type
     * @param param the wrapper for the target parameter
     * @param x0 the mean of the gaussian prior
     * @param cov the covariance of the gaussian prior
     */
    virtual MeasurementEdgeWrapper_Ptr addPriorOnConstantParameter(
        PriorEdgeTypes type, const std::string &name, const Eigen::VectorXd &x0,
        const Eigen::MatrixXd &cov) = 0;

    /**
     *  \brief add a prior edge on a time varying parameter
     *
     * Adds a constraint specifying prior knowledge on the value of a constant parameter
     * at a given timestamp
     *
     * @param type the prior edge type
     * @param param the wrapper for the target parameter
     * @param t the timestamp at which the prior knowledge is availale (nearest vertex will be selected)
     * @param x0 the mean of the gaussian prior
     * @param cov the covariance of the gaussian prior
     */
    virtual MeasurementEdgeWrapper_Ptr addPriorOnTimeVaryingParameter(
        PriorEdgeTypes type, const std::string &name, double t,
        const Eigen::VectorXd &x0, const Eigen::MatrixXd &cov) = 0;

    /* --------------------------- POSES GETTERS -------------------------------------- */

    /**
     * \brief returns the pose whose timestamp is nearer with respect to t
     *
     * @param t the searched timestamp
     * @param onlyBefore set to true if you require the returned pose to be prior w.r.t. t
     *
     * @return a wrapper to the pose vertex found, or NULL if there are no poses in the graph
     */
    virtual PoseVertexWrapper_Ptr getNearestPoseByTimestamp(double t,
        bool onlyBefore = false) = 0;

    /**
     *  \brief perform a spatial query
     *
     * @param sample the sample pose
     * @param d the maximum distance
     *
     * @return a vector composed of all the Poses for which the flag keepSpatialIndex was enabled
     * that lie within d from sample.
     *
     * TODO: For now, the metric is 2D Euclidean and does not take into account the orientation
     */
    virtual PoseVertexWrapperVector_Ptr getNeighbourPosesByDistance(
        PoseVertexWrapper_Ptr sample, double d) = 0;

    /**
     * \brief get the newest pose available
     *
     * @return a wrapper to the newest pose available.
     */
    virtual PoseVertexWrapper_Ptr getNewestPose() = 0;

    /**
     * \brief get the oldest pose available
     *
     * @return a wrapper to the oldest pose available.
     */
    virtual PoseVertexWrapper_Ptr getOldestPose() = 0;

    /**
     * \brief get the N-th pose starting from the newest, if N is set to zero the most recent pose is returned
     *
     * @param n the pose count starting from the most recent
     *
     * @return a wrapper to the oldest pose available.
     */
    virtual PoseVertexWrapper_Ptr getNthPose(int n) = 0;

    /**
     * \brief get the N-th pose starting from the oldest, if N is set to zero the oldest pose is returned
     *
     * @param n the pose count starting from the oldest
     *
     * @return a wrapper to the oldest pose available.
     */
    virtual PoseVertexWrapper_Ptr getNthOldestPose(int n) = 0;

    /* --------------------------- ESTIMATION CONTROL METHODS ------------------------- */

    /**
     *  \brief runs the estimation considering all the available pose vertices
     *
     * according to their 'fixed' property
     *
     * @param nIterations the number of Gauss-Newton/Levenberg-Marquardt iteration to perform.
     */
    virtual bool estimate(int nIterations) = 0;

    /**
     *  \brief runs the estimations considering the markov blanket of the provided pose vector
     *
     * all the poses, parameters and landmarks involved in any edge incident in the provided
     * poses will be included in estimation, according to its 'fixed' property.
     *
     * @param poses the pose vertices to be included in the estimation
     * @param nIterations the number of Gauss-Newton/Levenberg-Marquardt iteration to perform.
     */
    virtual bool estimate(PoseVertexWrapperVector poses, int nIterations) = 0;

    virtual ~FactorGraphFilter() {
    }
};

}

#endif /* FACTORGRAPHFILTER_H_ */
