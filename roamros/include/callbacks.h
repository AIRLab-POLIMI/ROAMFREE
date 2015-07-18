/*
 * functors.hpp
 *
 *  Created on: Feb 26, 2014
 *      Author: davide
 */

#ifndef CALLBACKS_H_
#define CALLBACKS_H_

#include <ros/ros.h>

#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/TwistWithCovarianceStamped.h"
#include "geometry_msgs/TwistStamped.h"
#include "geometry_msgs/Vector3Stamped.h"
#include "sensor_msgs/Imu.h"
#include "sensor_msgs/MagneticField.h"
#include "nav_msgs/Odometry.h"
#include "roamros_msgs/SingleTrackAckermannOdometryStamped.h"

namespace ROAMestimation {
class FactorGraphFilter;
}

namespace roamros {

class SensorConfiguration;

/*
 * Handles a PoseStampedWithCovariance message as a AbsolutePosition source
 */
void PoseWithCovarianceStampedToAbsolutePosition(
    const ros::MessageEvent<geometry_msgs::PoseWithCovarianceStamped const>& event,
    const SensorConfiguration *sensor, ROAMestimation::FactorGraphFilter* filter);

/*
 * Handles a PoseStamped message as a AbsolutePosition source
 */
void PoseStampedToAbsolutePosition(
    const ros::MessageEvent<geometry_msgs::PoseStamped const>& event,
    const SensorConfiguration *sensor, ROAMestimation::FactorGraphFilter* filter);

/*
 * Handles a TwistWithCovarianceStamped message as a LinearVelocity source
 */
void TwistWithCovarianceStampedToLinearVelocity(
    const ros::MessageEvent<geometry_msgs::TwistWithCovarianceStamped const>& event,
    const SensorConfiguration *sensor, ROAMestimation::FactorGraphFilter* filter);

/*
 * Handles a TwistStamped message as a LinearVelocity source
 */
void TwistStampedToLinearVelocity(
    const ros::MessageEvent<geometry_msgs::TwistStamped const>& event,
    const SensorConfiguration *sensor, ROAMestimation::FactorGraphFilter* filter);

/*
 * Handles a TwistWithCovarianceStamped message as an AngularVelocity source
 */
void TwistWithCovarianceStampedToAngularVelocity(
    const ros::MessageEvent<geometry_msgs::TwistWithCovarianceStamped const>& event,
    const SensorConfiguration *sensor, ROAMestimation::FactorGraphFilter* filter);

/*
 * Handles a TwistStamped message as an AngularVelocity source
 */
void TwistStampedToAngularVelocity(
    const ros::MessageEvent<geometry_msgs::TwistStamped const>& event,
    const SensorConfiguration *sensor, ROAMestimation::FactorGraphFilter* filter);

/*
 * Handles a TwistWithCovarianceStamped as a GenericOdometer source
 */
void TwistWithCovarianceStampedToGenericOdometer(
    const ros::MessageEvent<geometry_msgs::TwistWithCovarianceStamped const>& event,
    const SensorConfiguration *sensor, ROAMestimation::FactorGraphFilter* filter);

/*
 * Handles a TwistStamped as a GenericOdometer source
 */
void TwistStampedToGenericOdometer(
    const ros::MessageEvent<geometry_msgs::TwistStamped const>& event,
    const SensorConfiguration *sensor, ROAMestimation::FactorGraphFilter* filter);

/*
 * Handles a SingleTrackAckermannOdometry as a AckermannOdometer source
 */
void SingleTrackAckermannOdometryStampedToAckermannOdometer (
    const ros::MessageEvent<roamros_msgs::SingleTrackAckermannOdometryStamped const>& event,
    const SensorConfiguration *sensor, ROAMestimation::FactorGraphFilter* filter);

/*
 * Handles an Odometry as a GenericOdometer source
 */
void OdometryToGenericOdometer(
    const ros::MessageEvent<nav_msgs::Odometry const>& event,
    const SensorConfiguration *sensor, ROAMestimation::FactorGraphFilter* filter);

/*
 * Handles a Imu message as an AngularVelocity source
 */
void ImuToAngularVelocity(
    const ros::MessageEvent<sensor_msgs::Imu const>& event,
    const SensorConfiguration *sensor, ROAMestimation::FactorGraphFilter* filter);

/*
 * Handles a Imu message as an AngularVelocity source
 */
void ImuToLinearAcceleration(
    const ros::MessageEvent<sensor_msgs::Imu const>& event,
    const SensorConfiguration *sensor, ROAMestimation::FactorGraphFilter* filter);

/*
 * Handles a MagneticField message as VectorField soruce
 */
void MagneticFieldToVectorField(
    const ros::MessageEvent<sensor_msgs::MagneticField const>& event,
    const SensorConfiguration *sensor, ROAMestimation::FactorGraphFilter* filter);

/*
 * Handles a Vector3Stamped message as generic 3DOF measurement source
 */
void Vector3StampedToGeneric3DOFsensor(
    const ros::MessageEvent<geometry_msgs::Vector3Stamped const>& event,
    const SensorConfiguration *sensor, ROAMestimation::FactorGraphFilter* filter);

}

#endif /* CALLBACKS_H_ */
