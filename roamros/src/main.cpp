/*
 * main.cpp
 *
 *  Created on: Feb 21, 2014
 *      Author: davide
 */

#include <ros/ros.h>
#include <tf/transform_broadcaster.h>

#include <geometry_msgs/PoseStamped.h>

#include "ROAMestimation/ROAMestimation.h"
#include "ROAMimu/IMUIntegrator.h"
#include "ROAMimu/IMUIntegralHandler.h"

#include "roamros_config.h"
#include "solver_configuration.h"
#include "sensor_configuration.h"

int main(int argc, char *argv[]) {

	ros::init(argc, argv, "roamros_node");
	ros::NodeHandle n("~");

	// load solver configuration
	roamros::SolverConfiguration *solver;
	roamros::loadSolverConf(&solver);

	// load sensor configuration
	std::vector<roamros::SensorConfiguration *> sensors;
	roamros::loadSensorConfig(&sensors);

	// set up the solver
	ROAMestimation::FactorGraphFilter *filter;
	roamros::setUpSolver(solver, &filter);

	//set up the IMUHandler
	//the initialization will be performed into setUpSensors, because we need to read the imuRate and poseRate parameters
	ROAMimu::IMUIntegralHandler* handler;

	// set up the tf transformation broadcaster
	tf::TransformBroadcaster tf_broadcaster;

	// set up the output topic
	ros::Publisher pose_pub = n.advertise<geometry_msgs::PoseStamped>("pose", 16);

	// in case of simulated time we have to wait since time is available
	ROS_INFO("waiting for time to be available");
	ros::Duration(0.1).sleep(); // this does not return until we have something on /clock

	// set up the sensors
	roamros::setUpSensors(&sensors, solver->base_link_frame_id_, filter, &handler);

	ROS_INFO("sensors ready");
	// initialize the filter
	roamros::initializeSolver(filter, handler);

	ROS_INFO("Sensor noise 0,0 %f", handler->getSensorNoises()(0,0));


	// and subscribe to sensor topics
	std::vector<ros::Subscriber> subscribers;
	roamros::setUpSubscriptions(&sensors, filter, &subscribers, handler);

	// main loop
	ROS_INFO("Running.");

	ros::Rate r(solver->frequency_);

	while (filter->getWindowLenght() < 2.0*solver->pose_window_length_) {
		ROS_INFO("within get windows lenght");
		ros::spinOnce();
		r.sleep();
	}

	while (ros::ok()) {
		ROS_INFO("while ros ok");
		// handle all the callbacks up to now
		// all the sensor readings up to this time are
		ros::spinOnce();

		// run the estimation
		if (filter->estimate(solver->n_gauss_newton_iterations_) == false) {
			ROS_FATAL("exiting");
			ROS_BREAK();
 		}

		// get the estimate

		// the oldest
		ROAMestimation::PoseVertexWrapper_Ptr newest_pose = filter->getNewestPose();
		const Eigen::VectorXd &x = newest_pose->getEstimate();
		//*/

		/* the second oldest
		ROAMestimation::PoseVertexWrapper_Ptr newest_pose = filter->getNthPose(1);
		const Eigen::VectorXd &x = newest_pose->getEstimate();
		//*/

		// publish the pose as a tf transform from global_frame to base_link_frame_id
		tf::Transform tf_transform;
		tf_transform.setOrigin(tf::Vector3(x(0), x(1), x(2)));
		tf_transform.setRotation(tf::Quaternion(x(4), x(5), x(6), x(3)));
		tf_broadcaster.sendTransform(
				tf::StampedTransform(tf_transform,
						ros::Time(newest_pose->getTimestamp()), solver->global_frame_,
						solver->base_link_frame_id_));

		// publish the pose as a Pose
		geometry_msgs::PoseStamped pose_msg;

		pose_msg.header.seq = 0.0; //TODO: output a sequence counter
		pose_msg.header.stamp = ros::Time(newest_pose->getTimestamp());
		pose_msg.header.frame_id = solver->global_frame_;

		pose_msg.pose.position.x = x(0);
		pose_msg.pose.position.y = x(1);
		pose_msg.pose.position.z = x(2);

		pose_msg.pose.orientation.x = x(4); // in roamfree w is the first component of unit quaternions
		pose_msg.pose.orientation.y = x(5);
		pose_msg.pose.orientation.z = x(6);
		pose_msg.pose.orientation.w = x(3);

		pose_pub.publish(pose_msg);

		// marginalize old nodes
		filter->marginalizeOldNodes(solver->pose_window_length_);

		r.sleep();

	}
}
