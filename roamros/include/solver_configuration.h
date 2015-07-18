/*
 * solver_configuration.h
 *
 *  Created on: Feb 25, 2014
 *      Author: davide
 */

#ifndef SOLVER_CONFIGURATION_H_
#define SOLVER_CONFIGURATION_H_

#include <ros/ros.h>
#include <Eigen/Dense>

namespace roamros {

class SolverConfiguration {
public:
  SolverConfiguration(XmlRpc::XmlRpcValue &conf);

  double pose_window_length_; /* lenght of the pose window in seconds */
  bool dead_reckoning_; /* whatever we will have enough information to determine the pose wrt world */
  double frequency_; /* main loop frequency [Hz] */
  int n_gauss_newton_iterations_; /* how many gauss newton iterations have to be performed */
  bool low_level_logging_; /* whatever we have to write detailed logging info or not */

  std::string base_link_frame_id_;
  std::string global_frame_;
};

std::ostream& operator<<(std::ostream& s, const SolverConfiguration& conf);

} /* namespace roamros */
#endif /* SOLVER_CONFIGURATION_H_ */
