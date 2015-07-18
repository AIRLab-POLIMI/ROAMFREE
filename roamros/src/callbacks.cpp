/*
 * functors.cpp
 *
 *  Created on: Feb 26, 2014
 *      Author: davide
 */

#include "callbacks.h"

#include <cstring>
#include <Eigen/Dense>

#include <ros/ros.h>

#include "ROAMestimation/FactorGraphFilter.h"

#include "sensor_configuration.h"

namespace roamros {

void PoseWithCovarianceStampedToAbsolutePosition(
		const ros::MessageEvent<geometry_msgs::PoseWithCovarianceStamped const>& event,
		const SensorConfiguration *sensor, ROAMestimation::FactorGraphFilter* filter) {
	const geometry_msgs::PoseWithCovarianceStamped &msg = *(event.getMessage());

	static Eigen::VectorXd z(3);
	static Eigen::MatrixXd cov(3, 3);

	double t =
			sensor->use_header_stamp_ ?
					msg.header.stamp.toSec() : event.getReceiptTime().toSec();

	z << msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z;

	if (sensor->static_covariance_) {
		ROS_ASSERT(sensor->covariance_ != NULL);
		filter->addSequentialMeasurement(sensor->name_, t, z, *sensor->covariance_);
	} else {
		for (int r = 0; r < 3; r++) {
			for (int c = 0; c < 3; c++) {
				cov(r, c) = msg.pose.covariance.elems[6 * r + c];
			}
		}

		filter->addSequentialMeasurement(sensor->name_, t, z, cov);
	}
}

void PoseStampedToAbsolutePosition(
		const ros::MessageEvent<const geometry_msgs::PoseStamped>& event,
		const SensorConfiguration *sensor, ROAMestimation::FactorGraphFilter* filter) {
	const geometry_msgs::PoseStamped &msg = *(event.getMessage());

	static Eigen::VectorXd z(3);

	double t =
			sensor->use_header_stamp_ ?
					msg.header.stamp.toSec() : event.getReceiptTime().toSec();

	z << msg.pose.position.x, msg.pose.position.y, msg.pose.position.z;

	filter->addSequentialMeasurement(sensor->name_, t, z, *sensor->covariance_);
}

void TwistWithCovarianceStampedToLinearVelocity(
		const ros::MessageEvent<const geometry_msgs::TwistWithCovarianceStamped>& event,
		const SensorConfiguration *sensor, ROAMestimation::FactorGraphFilter* filter) {
	const geometry_msgs::TwistWithCovarianceStamped &msg = *(event.getMessage());

	static Eigen::VectorXd z(3);
	static Eigen::MatrixXd cov(3, 3);

	double t =
			sensor->use_header_stamp_ ?
					msg.header.stamp.toSec() : event.getReceiptTime().toSec();

	z << msg.twist.twist.linear.x, msg.twist.twist.linear.y, msg.twist.twist.linear.z;

	if (sensor->static_covariance_) {
		ROS_ASSERT(sensor->covariance_ != NULL);
		filter->addSequentialMeasurement(sensor->name_, t, z, *sensor->covariance_);
	} else {
		for (int r = 0; r < 3; r++) {
			for (int c = 0; c < 3; c++) {
				cov(r, c) = msg.twist.covariance[6 * r + c];
			}
		}

		filter->addSequentialMeasurement(sensor->name_, t, z, cov);
	}
}

void TwistStampedToLinearVelocity(
		const ros::MessageEvent<const geometry_msgs::TwistStamped>& event,
		const SensorConfiguration *sensor, ROAMestimation::FactorGraphFilter* filter) {
	const geometry_msgs::TwistStamped &msg = *(event.getMessage());

	static Eigen::VectorXd z(3);

	double t =
			sensor->use_header_stamp_ ?
					msg.header.stamp.toSec() : event.getReceiptTime().toSec();

	z << msg.twist.linear.x, msg.twist.linear.y, msg.twist.linear.z;

	filter->addSequentialMeasurement(sensor->name_, t, z, *sensor->covariance_);
}

void TwistWithCovarianceStampedToAngularVelocity(
		const ros::MessageEvent<const geometry_msgs::TwistWithCovarianceStamped>& event,
		const SensorConfiguration *sensor, ROAMestimation::FactorGraphFilter* filter) {
	const geometry_msgs::TwistWithCovarianceStamped &msg = *(event.getMessage());

	static Eigen::VectorXd z(3);
	static Eigen::MatrixXd cov(3, 3);

	double t =
			sensor->use_header_stamp_ ?
					msg.header.stamp.toSec() : event.getReceiptTime().toSec();

	z << msg.twist.twist.angular.x, msg.twist.twist.angular.y, msg.twist.twist.angular.z;

	if (sensor->static_covariance_) {
		ROS_ASSERT(sensor->covariance_ != NULL);
		filter->addSequentialMeasurement(sensor->name_, t, z, *sensor->covariance_);
	} else {
		for (int r = 3; r < 6; r++) {
			for (int c = 3; c < 6; c++) {
				cov(r, c) = msg.twist.covariance[6 * r + c];
			}
		}

		filter->addSequentialMeasurement(sensor->name_, t, z, cov);
	}
}

void TwistStampedToAngularVelocity(
		const ros::MessageEvent<const geometry_msgs::TwistStamped>& event,
		const SensorConfiguration *sensor, ROAMestimation::FactorGraphFilter* filter) {
	const geometry_msgs::TwistStamped &msg = *(event.getMessage());

	static Eigen::VectorXd z(3);

	double t =
			sensor->use_header_stamp_ ?
					msg.header.stamp.toSec() : event.getReceiptTime().toSec();

	z << msg.twist.angular.x, msg.twist.angular.y, msg.twist.angular.z;

	filter->addSequentialMeasurement(sensor->name_, t, z, *sensor->covariance_);
}

void SingleTrackAckermannOdometryStampedToAckermannOdometer(
		const ros::MessageEvent<roamros_msgs::SingleTrackAckermannOdometryStamped const>& event,
		const SensorConfiguration *sensor, ROAMestimation::FactorGraphFilter* filter) {
	const roamros_msgs::SingleTrackAckermannOdometryStamped &msg = *(event.getMessage());

	static Eigen::VectorXd z(2);

	double t =
			sensor->use_header_stamp_ ?
					msg.header.stamp.toSec() : event.getReceiptTime().toSec();

	z << msg.odometry.speed, msg.odometry.steer;

	filter->addSequentialMeasurement(sensor->name_, t, z, *sensor->covariance_);
}

void TwistWithCovarianceStampedToGenericOdometer(
		const ros::MessageEvent<const geometry_msgs::TwistWithCovarianceStamped>& event,
		const SensorConfiguration *sensor, ROAMestimation::FactorGraphFilter* filter) {
	const geometry_msgs::TwistWithCovarianceStamped &msg = *(event.getMessage());

	static Eigen::VectorXd z(6);
	static Eigen::MatrixXd cov(6, 6);

	double t =
			sensor->use_header_stamp_ ?
					msg.header.stamp.toSec() : event.getReceiptTime().toSec();

	z << msg.twist.twist.linear.x, msg.twist.twist.linear.y, msg.twist.twist.linear.z, msg.twist.twist.angular.x, msg.twist.twist.angular.y, msg.twist.twist.angular.z;

	if (sensor->static_covariance_) {
		ROS_ASSERT(sensor->covariance_ != NULL);
		filter->addSequentialMeasurement(sensor->name_, t, z, *sensor->covariance_);
	} else {
		for (int r = 0; r < 6; r++) {
			for (int c = 0; c < 6; c++) {
				cov(r, c) = msg.twist.covariance[6 * r + c];
			}
		}

		filter->addSequentialMeasurement(sensor->name_, t, z, cov);
	}
}

void TwistStampedToGenericOdometer(
		const ros::MessageEvent<const geometry_msgs::TwistStamped>& event,
		const SensorConfiguration *sensor, ROAMestimation::FactorGraphFilter* filter) {
	const geometry_msgs::TwistStamped &msg = *(event.getMessage());

	static Eigen::VectorXd z(6);
	static Eigen::MatrixXd cov(6, 6);

	double t =
			sensor->use_header_stamp_ ?
					msg.header.stamp.toSec() : event.getReceiptTime().toSec();

	z << msg.twist.linear.x, msg.twist.linear.y, msg.twist.linear.z, msg.twist.angular.x, msg.twist.angular.y, msg.twist.angular.z;

	filter->addSequentialMeasurement(sensor->name_, t, z, *sensor->covariance_);
}

void OdometryToGenericOdometer(
		const ros::MessageEvent<const nav_msgs::Odometry>& event,
		const SensorConfiguration *sensor, ROAMestimation::FactorGraphFilter* filter) {
	const nav_msgs::Odometry &msg = *(event.getMessage());

	static Eigen::VectorXd z(6);
	static Eigen::MatrixXd cov(6, 6);

	double t =
			sensor->use_header_stamp_ ?
					msg.header.stamp.toSec() : event.getReceiptTime().toSec();

	z << msg.twist.twist.linear.x, msg.twist.twist.linear.y, msg.twist.twist.linear.z, msg.twist.twist.angular.x, msg.twist.twist.angular.y, msg.twist.twist.angular.z;

	if (sensor->static_covariance_) {
		ROS_ASSERT(sensor->covariance_ != NULL);
		filter->addSequentialMeasurement(sensor->name_, t, z, *sensor->covariance_);
	} else {
		for (int r = 0; r < 6; r++) {
			for (int c = 0; c < 6; c++) {
				cov(r, c) = msg.twist.covariance[6 * r + c];
			}
		}

		filter->addSequentialMeasurement(sensor->name_, t, z, cov);
	}
}

void ImuToAngularVelocity(
		const ros::MessageEvent<const sensor_msgs::Imu>& event,
		const SensorConfiguration *sensor, ROAMestimation::FactorGraphFilter* filter) {
	const sensor_msgs::Imu &msg = *(event.getMessage());

	static Eigen::VectorXd z(3);
	static Eigen::MatrixXd cov(3, 3);

	double t =
			sensor->use_header_stamp_ ?
					msg.header.stamp.toSec() : event.getReceiptTime().toSec();

	z << msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z;

	if (sensor->static_covariance_) {
		ROS_ASSERT(sensor->covariance_ != NULL);
		filter->addSequentialMeasurement(sensor->name_, t, z, *sensor->covariance_);
	} else {
		for (int r = 0; r < 3; r++) {
			for (int c = 0; c < 3; c++) {
				cov(r, c) = msg.angular_velocity_covariance[3 * r + c];
			}
		}

		filter->addSequentialMeasurement(sensor->name_, t, z, cov);
	}
}

void ImuToLinearAcceleration(
		const ros::MessageEvent<const sensor_msgs::Imu>& event,
		const SensorConfiguration *sensor, ROAMestimation::FactorGraphFilter* filter) {
	const sensor_msgs::Imu &msg = *(event.getMessage());

	static Eigen::VectorXd z(3);
	static Eigen::MatrixXd cov(3, 3);

	double t =
			sensor->use_header_stamp_ ?
					msg.header.stamp.toSec() : event.getReceiptTime().toSec();

	z << msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z;

	if (sensor->static_covariance_) {
		ROS_ASSERT(sensor->covariance_ != NULL);
		filter->addSequentialMeasurement(sensor->name_, t, z, *sensor->covariance_);
	} else {
		for (int r = 0; r < 3; r++) {
			for (int c = 0; c < 3; c++) {
				cov(r, c) = msg.linear_acceleration_covariance[3 * r + c];
			}
		}

		filter->addSequentialMeasurement(sensor->name_, t, z, cov);
	}
}

void MagneticFieldToVectorField(
		const ros::MessageEvent<sensor_msgs::MagneticField const>& event,
		const SensorConfiguration *sensor, ROAMestimation::FactorGraphFilter* filter) {
	const sensor_msgs::MagneticField &msg = *(event.getMessage());

	static Eigen::VectorXd z(3);
	static Eigen::MatrixXd cov(3, 3);

	double t =
			sensor->use_header_stamp_ ?
					msg.header.stamp.toSec() : event.getReceiptTime().toSec();

	z << msg.magnetic_field.x, msg.magnetic_field.y, msg.magnetic_field.z;

	if (sensor->static_covariance_) {
		ROS_ASSERT(sensor->covariance_ != NULL);
		filter->addSequentialMeasurement(sensor->name_, t, z, *sensor->covariance_);
	} else {
		for (int r = 0; r < 3; r++) {
			for (int c = 0; c < 3; c++) {
				cov(r, c) = msg.magnetic_field_covariance[3 * r + c];
			}
		}

		filter->addSequentialMeasurement(sensor->name_, t, z, cov);
	}
}

void Vector3StampedToGeneric3DOFsensor(
		const ros::MessageEvent<const geometry_msgs::Vector3Stamped>& event,
		const SensorConfiguration* sensor, ROAMestimation::FactorGraphFilter* filter) {
	const geometry_msgs::Vector3Stamped &msg = *(event.getMessage());

	static Eigen::VectorXd z(3);

	double t =
			sensor->use_header_stamp_ ?
					msg.header.stamp.toSec() : event.getReceiptTime().toSec();

	z << msg.vector.x, msg.vector.y, msg.vector.z;

	filter->addSequentialMeasurement(sensor->name_, t, z, *sensor->covariance_);
}

}
