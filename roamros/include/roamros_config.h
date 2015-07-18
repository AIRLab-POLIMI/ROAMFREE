/*
 * roamros_config.h
 *
 *  Created on: Feb 24, 2014
 *      Author: davide
 */

#ifndef ROAMROS_CONFIG_H_
#define ROAMROS_CONFIG_H_

#include <ros/ros.h>

namespace ROAMestimation {
class FactorGraphFilter;
}

namespace roamros {
class SensorConfiguration;
class SolverConfiguration;
}

namespace geometry_msgs {
ROS_DECLARE_MESSAGE(PoseWithCovarianceStamped)

}

namespace roamros {

/*
 * loads solver configuration from the parameter server
 */
bool loadSolverConf(SolverConfiguration **solver_conf);

/*
 * loads sensors configuration from the parameter server
 */
bool loadSensorConfig(std::vector<SensorConfiguration *> *sensors_conf);

/*
 * create a new g2oFilter and configures it according to SolverConfiguration
 */
bool setUpSolver(SolverConfiguration *solver_conf,
		ROAMestimation::FactorGraphFilter **solver);

/*
 * wait for the initial pose and set it into the filter
 */
bool initializeSolver(ROAMestimation::FactorGraphFilter *solver);

/*
 * subscribes to the /initialpose topic and waits till the first message is available
 */
bool waitForInitialPose(const std::string &topic);
void initialPoseCallback(const geometry_msgs::PoseWithCovarianceStamped &msg);

/*
 * configure a g2oFilter with all the SensorConfiguration provided
 */
bool setUpSensors(const std::vector<SensorConfiguration*> *sensors,
		const std::string &base_link_frame_id,
		ROAMestimation::FactorGraphFilter* filter);

/*
 * binds callbacks accordint to sensor type and subscribe to sensor reading topics
 */

bool setUpSubscriptions(const std::vector<SensorConfiguration *> *sensors,
		ROAMestimation::FactorGraphFilter *filter,
		std::vector<ros::Subscriber> *subscribers);

/*
 * helper method, builds a parameter name starting from the sensor name and the parameter name
 */

inline std::string buildParameterName(const std::string &sensor,
		const std::string &param) {
	std::stringstream s;
	s << sensor << "_" << param;
	return s.str();
}

}

#endif /* ROAMROS_CONFIG_H_ */
