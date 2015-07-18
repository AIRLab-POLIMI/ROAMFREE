/*
 * solver_configuration.cpp
 *
 *  Created on: Feb 25, 2014
 *      Author: davide
 */

#include "solver_configuration.h"

namespace roamros {

SolverConfiguration::SolverConfiguration(XmlRpc::XmlRpcValue& conf) {

  if (conf.getType() != XmlRpc::XmlRpcValue::TypeStruct) {
    ROS_FATAL("solver: bad configuration, expecting structure.");
    ROS_BREAK();
  }

  if (!conf.hasMember("base_link_frame_id")
      || conf["base_link_frame_id"].getType()
          != XmlRpc::XmlRpcValue::TypeString) {

    std::cerr << conf["type"].getType() << std::endl;
    ROS_FATAL("solver: 'base_link_frame_id' field undefined or not a string");
    ROS_BREAK();
  }

  base_link_frame_id_ = static_cast<std::string &>(conf["base_link_frame_id"]);

  if (!conf.hasMember("global_frame_id")
      || conf["global_frame_id"].getType() != XmlRpc::XmlRpcValue::TypeString) {
    ROS_FATAL("solver: 'global_frame' field undefined or not a string");
    ROS_BREAK();
  }

  global_frame_ = static_cast<std::string &>(conf["global_frame_id"]);

  if (!conf.hasMember("pose_window_length")
      || conf["pose_window_length"].getType() != XmlRpc::XmlRpcValue::TypeDouble) {
    ROS_FATAL("solver: 'pose_window_length' field undefined or not a double");
    ROS_BREAK();
  }

  pose_window_length_ = static_cast<double>(conf["pose_window_length"]);

  if (pose_window_length_ < 0) {
    ROS_FATAL("solver: 'pose_window_length' field must be greater than zero");
    ROS_BREAK();
  }

  if (!conf.hasMember("dead_reckoning")
      || conf["dead_reckoning"].getType() != XmlRpc::XmlRpcValue::TypeBoolean) {
    ROS_FATAL("solver: 'dead_reckoning' field undefined or not a boolean");
    ROS_BREAK();
  }

  dead_reckoning_ = static_cast<bool>(conf["dead_reckoning"]);

  if (!conf.hasMember("frequency")
      || conf["frequency"].getType() != XmlRpc::XmlRpcValue::TypeDouble) {
    ROS_FATAL("solver: 'frequency' field undefined or not a double");
    ROS_BREAK();
  }

  frequency_ = static_cast<double>(conf["frequency"]);

  if (!conf.hasMember("n_gauss_newton_iterations")
      || conf["n_gauss_newton_iterations"].getType()
          != XmlRpc::XmlRpcValue::TypeInt) {
    ROS_FATAL(
        "solver: 'n_gauss_newton_iterations' field undefined or not an integer");
    ROS_BREAK();
  }

  n_gauss_newton_iterations_ =
      static_cast<int>(conf["n_gauss_newton_iterations"]);

  if (!conf.hasMember("low_level_logging")
      || conf["low_level_logging"].getType() != XmlRpc::XmlRpcValue::TypeBoolean) {
    ROS_FATAL("solver: 'low_level_logging' field undefined or not a boolean");
    ROS_BREAK();
  }

  low_level_logging_ = static_cast<bool>(conf["low_level_logging"]);
}

std::ostream& operator <<(std::ostream& s, const SolverConfiguration& conf) {

  s << "Solver: " << std::endl;

  s << "\tbase_link frame id: " << conf.base_link_frame_id_ << std::endl;
  s << "\tglobal frame id: " << conf.global_frame_ << std::endl;
  s << "\tpose window length: " << conf.pose_window_length_ << std::endl;
  s << "\tfrequency: " << conf.frequency_ << std::endl;
  s << "\tnumber of Gauss-Newton iterations: "
      << conf.n_gauss_newton_iterations_ << std::endl;
  s << "\tdead reckoning: " << conf.dead_reckoning_ << std::endl;

  return s;
}

} /* namespace roamros */
