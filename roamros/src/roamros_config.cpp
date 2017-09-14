/*
 * roamros_config.cpp
 *
 *  Created on: Feb 24, 2014
 *      Author: davide
 */

#include "roamros_config.h"

#include <Eigen/Dense>
#include <boost/bind.hpp>

#include "geometry_msgs/PoseWithCovarianceStamped.h"

#include "roamros_msgs/SingleTrackAckermannOdometryStamped.h"

#include "ROAMestimation/ROAMestimation.h"
#include "ROAMestimation/StochasticProcessFactory.h"
#include "ROAMimu/IMUIntegrator.h"
#include "ROAMimu/IMUIntegralHandler.h"

#include <tf/transform_listener.h>

#include "sensor_configuration.h"
#include "solver_configuration.h"

#include "callbacks.h"

using namespace ROAMestimation;
namespace roamros {

bool loadSensorConfig(std::vector<SensorConfiguration *> *sensors_conf) {

	ros::NodeHandle n("~");

	XmlRpc::XmlRpcValue sensors_field;
	if (!n.getParam("sensors", sensors_field)) {
		ROS_FATAL("sensors undefined.");
		ROS_BREAK();
	}

	for (XmlRpc::XmlRpcValue::iterator it = sensors_field.begin();
			it != sensors_field.end(); ++it) {
		SensorConfiguration *s = new SensorConfiguration(it->first, it->second);

		sensors_conf->push_back(s);
	}

	return true;
}

bool loadSolverConf(SolverConfiguration **solver_conf) {

	ros::NodeHandle n("~");

	XmlRpc::XmlRpcValue solver_field;
	if (!n.getParam("solver", solver_field)) {
		ROS_FATAL("solver structure undefined.");
		ROS_BREAK();
	}

	*solver_conf = new SolverConfiguration(solver_field);

	return true;

}

// TODO: ugly, with a global variables
geometry_msgs::PoseWithCovarianceStamped *initial_pose_msg_ret = NULL;

bool setUpSolver(SolverConfiguration *solver_conf,
		ROAMestimation::FactorGraphFilter **solver) {

	*solver =
			ROAMestimation::FactorGraphFilterFactory::getNewFactorGraphFilter();

	(*solver)->setDeadReckoning(solver_conf->dead_reckoning_);

	if (solver_conf->low_level_logging_ == true) {
		(*solver)->setLowLevelLogging(true);
		system("rm /tmp/roamfree/*.log");
	}

	return true;
}

bool waitForInitialPose(const std::string &topic) {

	ros::NodeHandle n("~");

	ros::Subscriber sub = n.subscribe(topic, 1, initialPoseCallback);

	ROS_INFO("subscribing to %s, waiting for initial pose...", topic.c_str());

	while (initial_pose_msg_ret == NULL && ros::ok()) {
		ros::spinOnce();
		ros::Duration(1).sleep();
	}

	ROS_INFO("got initial pose");

	sub.shutdown();

	return true;
}

void initialPoseCallback(const geometry_msgs::PoseWithCovarianceStamped &msg) {
	ros::NodeHandle n("~");

	initial_pose_msg_ret = new geometry_msgs::PoseWithCovarianceStamped(msg);
}

bool initializeSolver(ROAMestimation::FactorGraphFilter* solver,
		ROAMimu::IMUIntegralHandler* handler) {
	/* get the initial pose */
	waitForInitialPose("/initialpose");

	Eigen::VectorXd gyroBias0(3);
	gyroBias0 << 0.0, 0.0, 0.0;

	Eigen::VectorXd accBias0(3);
	accBias0 << 0.0, 0.0, 0.0;
	// -- white noise on accelerometer and gyro

	Eigen::Matrix<double, 6, 6> & sensorNoises = handler->getSensorNoises();
	sensorNoises.diagonal() << 0.0016, 0.0016, 0.0016, 1.15172e-05, 1.15172e-05, 1.15172e-05;

	Eigen::VectorXd x0(7);

	x0 << initial_pose_msg_ret->pose.pose.position.x, initial_pose_msg_ret->pose.pose.position.y, initial_pose_msg_ret->pose.pose.position.z, initial_pose_msg_ret->pose.pose.orientation.w, initial_pose_msg_ret->pose.pose.orientation.x, initial_pose_msg_ret->pose.pose.orientation.y, initial_pose_msg_ret->pose.pose.orientation.z;

	double t0 = initial_pose_msg_ret->header.stamp.toSec();
	handler->init(true, t0, x0);

	// on the gyro gauss markov process
	/*	solver->addPriorOnTimeVaryingParameter(ROAMestimation::Euclidean3DPrior,
	 "IMUintegralDeltaP_Bw_GM", solver->getOldestPose()->getTimestamp(),
	 gyroBias0_GM, 1e-6 * Eigen::MatrixXd::Identity(3, 3));*/

	handler->setPredictorEnabled(true);
	/*solver->addPriorOnConstantParameter(Euclidean3DPrior,
	 "IMUintegralDeltaP_Ba", accBias0,
	 0.01 / 3.0 * Eigen::MatrixXd::Identity(3, 3));
	 solver->addPriorOnConstantParameter(Euclidean3DPrior,
	 "IMUintegralDeltaP_Bw", gyroBias0,
	 pow((0.05 / 3.0), 2) * Eigen::MatrixXd::Identity(3, 3));*/

	//ROAMestimation::PoseVertexWrapper_Ptr p = solver->setInitialPose(x0, t0 != 0.0 ? t0 : ros::Time::now().toSec());
	//p->setFixed(true);
}

bool setUpSensors(const std::vector<SensorConfiguration*> *sensors,
		const std::string &base_link_frame_id,
		ROAMestimation::FactorGraphFilter* filter,
		ROAMimu::IMUIntegralHandler** handler) {

	ros::NodeHandle n("~");

	tf::TransformListener tf_listener(n);

	for (std::vector<SensorConfiguration *>::const_iterator it =
			sensors->begin(); it != sensors->end(); ++it) {
		const SensorConfiguration &s = **it;

		//ROS_INFO("configuring sensor %s", s.name_.c_str());
		//if the sensors is the IMU we don't have to work with the filter, but with the imuhandler
		if (s.name_.compare("IMUintegral")) {
			ROS_INFO("configuring sensor %s", s.name_.c_str());

			// add the sensor
			filter->addSensor(s.name_, s.type_, s.is_master_, true); // only sequential sensors

			// for each parameter, stuff it into the solver
			for (std::map<std::string, Eigen::VectorXd *>::const_iterator pit =
					s.parameters_.begin(); pit != s.parameters_.end(); ++pit) {

				const std::string &p_name = pit->first;
				Eigen::VectorXd &p_value = *pit->second;

				// TODO: now I decide the parameter type from the vector length. This one-to-one map may not hold on future

				ROAMestimation::ParameterTypes p_type;
				switch (p_value.rows()) {
				case 1:
					p_type = ROAMestimation::Euclidean1D;
					break;
				case 3:
					p_type = ROAMestimation::Euclidean3D;
					break;
				case 4:
					p_type = ROAMestimation::Quaternion;
					break;
				case 9:
					p_type = ROAMestimation::Matrix3D;
					break;
				}

				filter->addConstantParameter(p_type,
						buildParameterName(s.name_, p_name), p_value, true);
			}

			ROS_INFO("wait for transformation from frame %s to frame %s",
					base_link_frame_id.c_str(), s.frame_id_.c_str());

			tf::StampedTransform T;

			// wait for up to 60 s to a transform between robot and sensor to become available
			// note that we assume these are static so we don't care about time. (ros::Time(0) <- whatever time)

			std::string error = "";
			try {
				tf_listener.waitForTransform(base_link_frame_id, s.frame_id_,
						ros::Time(0.0), ros::Duration(60.0),
						ros::Duration(0.05), &error);
				tf_listener.lookupTransform(base_link_frame_id, s.frame_id_,
						ros::Time(0.0), T);

			} catch (tf::TransformException &ex) {
				ROS_FATAL("transformation from frame %s to frame %s undefined",
						base_link_frame_id.c_str(), s.frame_id_.c_str());
				ROS_FATAL("%s", ex.what());
				ROS_FATAL("%s", error.c_str());
				ROS_BREAK();
			}

			filter->addConstantParameter(buildParameterName(s.name_, "SOx"),
					T.getOrigin().x(), true);
			filter->addConstantParameter(buildParameterName(s.name_, "SOy"),
					T.getOrigin().y(), true);
			filter->addConstantParameter(buildParameterName(s.name_, "SOz"),
					T.getOrigin().z(), true);
			filter->addConstantParameter(buildParameterName(s.name_, "qOSx"),
					T.getRotation().x(), true);
			filter->addConstantParameter(buildParameterName(s.name_, "qOSy"),
					T.getRotation().y(), true);
			filter->addConstantParameter(buildParameterName(s.name_, "qOSz"),
					T.getRotation().z(), true);
		} else {

			ROS_INFO("Handle IMU %s", s.name_.c_str());
			//TODO
			//read the imu parameter and setup the handler

			// -- accelerometer bias
			Eigen::VectorXd accBias0(3);  // initial value
			accBias0 << 0.0, 0.0, 0.0;

			/*Eigen::MatrixXd randomWalkNoiseVar = 10
			 * Eigen::MatrixXd::Identity(3, 3);

			 ParameterWrapper_Ptr ba_par =
			 StochasticProcessFactory::addEucl3DRandomWalk(filter,
			 "IMUintegralDeltaP_Ba", accBias0,
			 randomWalkNoiseVar, 1.0); // 1.0 -> keep one sample every 1.0s
			 */
			ParameterWrapper_Ptr ba_par =
					StochasticProcessFactory::addEucl3DRandomConstant(filter,
							"IMUintegralDeltaP_Ba", accBias0);
			// -- gyroscope bias
			/*Eigen::VectorXd gyroBias0_RC(3); // initial values for the RC and GM processes
			 gyroBias0_RC << 0.0, 0.0, 0.0;
			 Eigen::VectorXd gyroBias0_GM(3);
			 gyroBias0_GM << 0.0, 0.0, 0.0;*/

			/*Eigen::MatrixXd gaussMarkovNoiseVar = 1e-4
			 * Eigen::MatrixXd::Identity(3, 3);
			 Eigen::VectorXd gaussMarkovBeta = 1 / 30.0
			 * Eigen::VectorXd::Ones(3);*/

			/*ParameterWrapper_Ptr bw_par =
			 StochasticProcessFactory::addEucl3DGaussMarkovPlusRandomConstant(
			 filter, "IMUintegralDeltaP_Bw", gyroBias0_RC,
			 gyroBias0_GM, gaussMarkovBeta, gaussMarkovNoiseVar,
			 1.0); // 1.0 -> keep one sample every 1.0s*/

			Eigen::VectorXd gyroBias0(3); // initial values for the RC and GM processes
			gyroBias0 << 0.0, 0.0, 0.0;
			ParameterWrapper_Ptr bw_par =
					StochasticProcessFactory::addEucl3DRandomConstant(filter,
							"IMUintegralDeltaP_Bw", gyroBias0);
			/**
			 * SET UP TRANSFORMATION
			 */
			ROS_INFO("wait for transformation from frame %s to frame %s",
					base_link_frame_id.c_str(), s.frame_id_.c_str());

			tf::StampedTransform T;

			// wait for up to 60 s to a transform between robot and sensor to become available
			// note that we assume these are static so we don't care about time. (ros::Time(0) <- whatever time)

			std::string error = "";
			try {
				tf_listener.waitForTransform(base_link_frame_id, s.frame_id_,
						ros::Time(0.0), ros::Duration(60.0),
						ros::Duration(0.05), &error);
				tf_listener.lookupTransform(base_link_frame_id, s.frame_id_,
						ros::Time(0.0), T);

			} catch (tf::TransformException &ex) {
				ROS_FATAL("transformation from frame %s to frame %s undefined",
						base_link_frame_id.c_str(), s.frame_id_.c_str());
				ROS_FATAL("%s", ex.what());
				ROS_FATAL("%s", error.c_str());
				ROS_BREAK();
			}
			Eigen::VectorXd T_OS_IMU(7); // Transformation between IMU and robot frame
			//T_OS_IMU << T.getOrigin().getX(), T.getOrigin().getY(),	T.getOrigin().getZ(), T.getRotation().getW(),T.getRotation().getX(),T.getRotation().getY(),T.getRotation().getZ();
			T_OS_IMU << 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0;

			// -- IMU integral handler construction
			*handler = new ROAMimu::IMUIntegralHandler(filter, "IMUintegral",
					round(100.0 / 10.0), 1.0 / 100.0, ba_par, bw_par, T_OS_IMU);

		}
	}
	return true;
}

bool setUpSubscriptions(const std::vector<SensorConfiguration*>* sensors,
		ROAMestimation::FactorGraphFilter* filter,
		std::vector<ros::Subscriber>* subscribers,
		ROAMimu::IMUIntegralHandler* handler) {

	ros::NodeHandle n("~");

	for (std::vector<SensorConfiguration *>::const_iterator it =
			sensors->begin(); it != sensors->end(); ++it) {
		const SensorConfiguration *s = *it;

		// this is very verbose since there is no introspection in C++
		// we want to subscribe to sensor topics and to employ a different callback function
		// depending on the topic type.

		try {
			switch (s->type_) {
			case ROAMestimation::AbsolutePosition:
				if (s->topic_type_
						== "geometry_msgs/PoseWithCovarianceStamped") {
					subscribers->push_back(
							n.subscribe
									< geometry_msgs::PoseWithCovarianceStamped
									> (s->topic_, 1024, boost::bind(
											roamros::PoseWithCovarianceStampedToAbsolutePosition,
											_1, s, filter)));
				} else if (s->topic_type_ == "geometry_msgs/PoseStamped") {
					if (!s->static_covariance_)
						throw -2;

					subscribers->push_back(
							n.subscribe < geometry_msgs::PoseStamped
									> (s->topic_, 1024, boost::bind(
											roamros::PoseStampedToAbsolutePosition,
											_1, s, filter)));
				} else if (s->topic_type_ == "geometry_msgs/Vector3Stamped") {
					if (!s->static_covariance_)
						throw -2;

					subscribers->push_back(
							n.subscribe < geometry_msgs::Vector3Stamped
									> (s->topic_, 1024, boost::bind(
											roamros::Vector3StampedToGeneric3DOFsensor,
											_1, s, filter)));
				} else
					throw -1;
				break;

			case ROAMestimation::LinearVelocity:
				if (s->topic_type_
						== "geometry_msgs/TwistWithCovarianceStamped") {
					subscribers->push_back(
							n.subscribe
									< geometry_msgs::TwistWithCovarianceStamped
									> (s->topic_, 1024, boost::bind(
											roamros::TwistWithCovarianceStampedToLinearVelocity,
											_1, s, filter)));
				} else if (s->topic_type_ == "geometry_msgs/TwistStamped") {
					if (!s->static_covariance_)
						throw -2;

					subscribers->push_back(
							n.subscribe < geometry_msgs::TwistStamped
									> (s->topic_, 1024, boost::bind(
											roamros::TwistStampedToLinearVelocity,
											_1, s, filter)));
				} else if (s->topic_type_ == "geometry_msgs/Vector3Stamped") {
					if (!s->static_covariance_)
						throw -2;

					subscribers->push_back(
							n.subscribe < geometry_msgs::Vector3Stamped
									> (s->topic_, 1024, boost::bind(
											roamros::Vector3StampedToGeneric3DOFsensor,
											_1, s, filter)));
				} else
					throw -1;
				break;

			case ROAMestimation::AngularVelocity:
				if (s->topic_type_ == "sensor_msgs/Imu") {
					subscribers->push_back(
							n.subscribe < sensor_msgs::Imu
									> (s->topic_, 1024, boost::bind(
											roamros::ImuToAngularVelocity, _1,
											s, filter)));
				} else if (s->topic_type_
						== "geometry_msgs/TwistWithCovarianceStamped") {
					subscribers->push_back(
							n.subscribe
									< geometry_msgs::TwistWithCovarianceStamped
									> (s->topic_, 1024, boost::bind(
											roamros::TwistWithCovarianceStampedToAngularVelocity,
											_1, s, filter)));
				} else if (s->topic_type_ == "geometry_msgs/TwistStamped") {
					if (!s->static_covariance_)
						throw -2;

					subscribers->push_back(
							n.subscribe < geometry_msgs::TwistStamped
									> (s->topic_, 1024, boost::bind(
											roamros::TwistStampedToAngularVelocity,
											_1, s, filter)));
				} else if (s->topic_type_ == "geometry_msgs/Vector3Stamped") {
					if (!s->static_covariance_)
						throw -2;

					subscribers->push_back(
							n.subscribe < geometry_msgs::Vector3Stamped
									> (s->topic_, 1024, boost::bind(
											roamros::Vector3StampedToGeneric3DOFsensor,
											_1, s, filter)));
				} else
					throw -1;
				break;

			case ROAMestimation::GenericOdometer:
				if (s->topic_type_
						== "geometry_msgs/TwistWithCovarianceStamped") {
					subscribers->push_back(
							n.subscribe
									< geometry_msgs::TwistWithCovarianceStamped
									> (s->topic_, 1024, boost::bind(
											roamros::TwistWithCovarianceStampedToGenericOdometer,
											_1, s, filter)));
				} else if (s->topic_type_ == "geometry_msgs/TwistStamped") {
					if (!s->static_covariance_)
						throw -2;

					subscribers->push_back(
							n.subscribe < geometry_msgs::TwistStamped
									> (s->topic_, 1024, boost::bind(
											roamros::TwistStampedToGenericOdometer,
											_1, s, filter)));
				} else if (s->topic_type_ == "nav_msgs/Odometry") {
					subscribers->push_back(
							n.subscribe < nav_msgs::Odometry
									> (s->topic_, 1024, boost::bind(
											roamros::OdometryToGenericOdometer,
											_1, s, filter)));
				} else
					throw -1;
				break;

			case ROAMestimation::AckermannOdometer:
				if (s->topic_type_
						== "roamros/SingleTrackAckermannOdometryStamped") {
					subscribers->push_back(
							n.subscribe
									< roamros_msgs::SingleTrackAckermannOdometryStamped
									> (s->topic_, 1024, boost::bind(
											roamros::SingleTrackAckermannOdometryStampedToAckermannOdometer,
											_1, s, filter)));
				} else
					throw -1;
				break;

			case ROAMestimation::LinearAcceleration:
				if (s->topic_type_ == "sensor_msgs/Imu") {
					subscribers->push_back(
							n.subscribe < sensor_msgs::Imu
									> (s->topic_, 1024, boost::bind(
											roamros::ImuToLinearAcceleration,
											_1, s, filter)));
				} else if (s->topic_type_ == "geometry_msgs/Vector3Stamped") {
					if (!s->static_covariance_)
						throw -2;

					subscribers->push_back(
							n.subscribe < geometry_msgs::Vector3Stamped
									> (s->topic_, 1024, boost::bind(
											roamros::Vector3StampedToGeneric3DOFsensor,
											_1, s, filter)));
				} else
					throw -1;
				break;
				//case to manage imuhandler
			case ROAMestimation::IMUHandler:
				ROS_INFO("within case imuhandler");
				if (s->topic_type_ == "sensor_msgs/Imu") {
					subscribers->push_back(
							n.subscribe < sensor_msgs::Imu
									> (s->topic_, 1024, boost::bind(
											roamros::ImuToIMUHandler, _1, s,
											handler)));
				}
				break;

			case ROAMestimation::VectorField:
			case ROAMestimation::VectorFieldAsCompass:
				ROS_INFO("magnetometer");
				if (s->topic_type_ == "sensor_msgs/MagneticField") {
					subscribers->push_back(
							n.subscribe < sensor_msgs::MagneticField
									> (s->topic_, 1024, boost::bind(
											roamros::MagneticFieldToVectorField,
											_1, s, filter)));
				} else if (s->topic_type_ == "geometry_msgs/Vector3Stamped") {
					if (!s->static_covariance_)
						throw -2;

					subscribers->push_back(
							n.subscribe < geometry_msgs::Vector3Stamped
									> (s->topic_, 1024, boost::bind(
											roamros::Vector3StampedToGeneric3DOFsensor,
											_1, s, filter)));
				} else
					throw -1;
				break;

			default:
				ROS_FATAL("unsupported sensor type for sensor %s",
						s->name_.c_str());
				ROS_BREAK();
			}
		} catch (int e) {

			/*
			 *  provide nice messages for common error sources
			 */

			switch (e) {
			case -1:
				ROS_FATAL("unsupported topic type type for sensor %s",
						s->name_.c_str());
				ROS_BREAK();
				break;
			case -2:
				ROS_FATAL(
						"topic type '%s' does not contain covariance information and sensor %s.static_covariance = False",
						s->topic_type_.c_str(), s->name_.c_str());
				ROS_BREAK();
				break;
			}

		}
	}
	ROS_INFO("all subscriptions ready");
	return true;
}

}
