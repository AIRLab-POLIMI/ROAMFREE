/*
 * ConfiguredSensor.h
 *
 *  Created on: Feb 24, 2014
 *      Author: davide
 */

#ifndef SENSORCONFIGURATION_H_
#define SENSORCONFIGURATION_H_

#include <iostream>
#include <Eigen/Dense>

#include <ros/ros.h>
#include "ROAMestimation/Enums.h"

namespace roamros {

class SensorConfiguration {

public:

  /*
   * loads all the configuration parameters from the entry in the parameter server
   * and properly initialize all the public fields
   */

  SensorConfiguration(std::string name, XmlRpc::XmlRpcValue &conf);
  ~SensorConfiguration();

  std::string name_; /* sensor name */
  ROAMestimation::MeasTypes type_; /* sensor measurement domain */
  bool is_master_; /* whatever this is the master sensor or not */
  std::string frame_id_; /* tf sensor frame. */
  std::string topic_; /* the topic where the measurements will be published */
  std::string topic_type_; /* the type of the topic, it is used to determine which callback to employ */

  bool use_header_stamp_; /* if we have to employ msg.header.stamp or the MessageEvent recipit_time */

  bool static_covariance_;
  Eigen::MatrixXd *covariance_;

  std::map<std::string, Eigen::VectorXd *> parameters_; /* map which stores all the sensor dependent parameters */

private:

  bool XmlRpcValueToEigenXd(XmlRpc::XmlRpcValue& field,
      Eigen::VectorXd *target);
  bool XmlRpcValueToEigenXd(XmlRpc::XmlRpcValue& field,
      Eigen::MatrixXd *target);

};

std::ostream& operator<<(std::ostream& s, const SensorConfiguration& conf);

} /* namespace roamros */
#endif /* SENSORCONFIGURATION_H_ */
