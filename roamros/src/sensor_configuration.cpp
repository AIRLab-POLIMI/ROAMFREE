/*
 * ConfiguredSensor.cpp
 *
 *  Created on: Feb 24, 2014
 *      Author: davide
 */

#include "sensor_configuration.h"

namespace roamros {

SensorConfiguration::SensorConfiguration(std::string name,
    XmlRpc::XmlRpcValue& conf) :
    name_(name), covariance_(NULL) {

  if (conf.getType() != XmlRpc::XmlRpcValue::TypeStruct) {
    ROS_FATAL("sensor %s: bad sensor configuration, expecting structure.",
        name.c_str());
    ROS_BREAK();
  }

  // set sensor type

  if (!conf.hasMember("type")
      || conf["type"].getType() != XmlRpc::XmlRpcValue::TypeString) {
    ROS_FATAL("sensor %s: 'type' field undefined or not a string",
        name.c_str());
    ROS_BREAK();
  }

  std::string type = static_cast<std::string &>(conf["type"]);

  if (type == "AbsolutePosition") {
    type_ = ROAMestimation::AbsolutePosition;
  } else if (type == "LinearVelocity") {
    type_ = ROAMestimation::LinearVelocity;
  } else if (type == "AngularVelocity") {
    type_ = ROAMestimation::AngularVelocity;
  } else if (type == "LinearAcceleration") {
    type_ = ROAMestimation::LinearAcceleration;
  } else if (type == "AckermannOdometer") {
    type_ = ROAMestimation::AckermannOdometer;
  } else if (type == "TriskarOdometer") {
    type_ = ROAMestimation::TriskarOdometer;
  } else if (type == "DifferentialDriveOdometer") {
    type_ = ROAMestimation::DifferentialDriveOdometer;
  } else if (type == "GenericOdometer") {
    type_ = ROAMestimation::GenericOdometer;
  } else if (type == "VectorField") {
    type_ = ROAMestimation::VectorField;
  } else if (type == "VectorFieldAsCompass") {
    type_ = ROAMestimation::VectorFieldAsCompass;
  } else if (type == "FixedFeaturePosition") {
    type_ = ROAMestimation::FixedFeaturePosition;
  } else if (type == "FixedFeaturePose") {
    type_ = ROAMestimation::FixedFeaturePose;
  } else if (type == "PlanarConstraint") {
    type_ = ROAMestimation::PlanarConstraint;
  } else {
    ROS_FATAL("sensor %s: unknown sensor type '%s'", name.c_str(),
        type.c_str());
    ROS_BREAK();
  }

  // set master

  if (!conf.hasMember("is_master")
      || conf["is_master"].getType() != XmlRpc::XmlRpcValue::TypeBoolean) {
    ROS_FATAL("sensor %s: 'is_master' field undefined or not a boolean",
        name.c_str());
    ROS_BREAK();
  }

  is_master_ = (bool &) conf["is_master"];

  // set frame id

  if (!conf.hasMember("frame_id")
      || conf["frame_id"].getType() != XmlRpc::XmlRpcValue::TypeString) {
    ROS_FATAL("sensor %s: 'frame_id' field undefined or not a string",
        name.c_str());
    ROS_BREAK();
  }

  frame_id_ = static_cast<std::string &>(conf["frame_id"]);

  // set topic

  if (!conf.hasMember("topic")
      || conf["topic"].getType() != XmlRpc::XmlRpcValue::TypeString) {
    ROS_FATAL("sensor %s: 'topic' field undefined or not a string",
        name.c_str());
    ROS_BREAK();
  }

  topic_ = static_cast<std::string &>(conf["topic"]);

  // set topic type

  if (!conf.hasMember("topic_type")
      || conf["topic_type"].getType() != XmlRpc::XmlRpcValue::TypeString) {
    ROS_FATAL("sensor %s: 'topic_type' field undefined or not a string",
        name.c_str());
    ROS_BREAK();
  }

  topic_type_ = static_cast<std::string &>(conf["topic_type"]);

  // set use header tstamp

  if (!conf.hasMember("use_header_stamp")
      || conf["use_header_stamp"].getType()
          != XmlRpc::XmlRpcValue::TypeBoolean) {
    ROS_FATAL("sensor %s: 'use_header_stamp' field undefined or not a boolean",
        name.c_str());
    ROS_BREAK();
  }

  use_header_stamp_ = static_cast<bool>(conf["use_header_stamp"]);

  // set whatever this sensor has static covariance matrix

  if (!conf.hasMember("static_covariance")
      || conf["static_covariance"].getType()
          != XmlRpc::XmlRpcValue::TypeBoolean) {
    ROS_FATAL("sensor %s: 'static_covariance' field undefined or not a boolean",
        name.c_str());
    ROS_BREAK();
  }

  static_covariance_ = static_cast<bool>(conf["static_covariance"]);

  if (static_covariance_ == true) {
    if (conf.hasMember("covariance")) {
      covariance_ = new Eigen::MatrixXd();

      if (conf["covariance"].getType() != XmlRpc::XmlRpcValue::TypeArray
          || !XmlRpcValueToEigenXd(conf["covariance"], covariance_)) {
        ROS_FATAL("sensor %s: malformed 'covariance' field",
            name.c_str());
        ROS_BREAK();
      }

    } else {
      ROS_INFO("sensor %s: ignoring static covariance", name.c_str());
    }
  }

  // load parameters

  if (conf.hasMember("parameters")) {
    XmlRpc::XmlRpcValue & params = conf["parameters"];

    if (params.getType() != XmlRpc::XmlRpcValue::TypeStruct) {
      ROS_FATAL("sensor %s: malformed parameters section, expecting structure",
          name.c_str());
      ROS_BREAK();
    }

    XmlRpc::XmlRpcValue::iterator it;
    for (it = params.begin(); it != params.end(); ++it) {
      Eigen::VectorXd *tmp = new Eigen::VectorXd;

      if (it->second.getType() != XmlRpc::XmlRpcValue::TypeArray
          || !XmlRpcValueToEigenXd(it->second, tmp)) {
        ROS_FATAL("sensor %s: malformed array at parameter '%s'", name.c_str(),
            it->first.c_str());
        ROS_BREAK();
      }

      // this check is needed since at the present we decide parameter type
      // depending on its size. 4 -> Quaternion, 3-> Euclidean3D, etc

      if (tmp->rows() != 1 && tmp->rows() != 3 && tmp->rows() != 4
          && tmp->rows() != 9) {
        ROS_FATAL("sensor %s: parameter '%s' has a wrong size: %ld",
            name.c_str(), it->first.c_str(), tmp->rows());
        ROS_BREAK();
      }

      parameters_[it->first] = tmp;
    }
  }
}

SensorConfiguration::~SensorConfiguration() {
  if (covariance_ != NULL) {
    delete covariance_;
  }
}

bool SensorConfiguration::XmlRpcValueToEigenXd(XmlRpc::XmlRpcValue& field,
    Eigen::VectorXd *target) {

  target->resize(field.size());

  for (int k = 0; k < field.size(); k++) {
    if (field[k].getType() != XmlRpc::XmlRpcValue::TypeDouble) {
      return false;
    }

    (*target)[k] = static_cast<double>(field[k]);
  }

  return true;
}

bool SensorConfiguration::XmlRpcValueToEigenXd(XmlRpc::XmlRpcValue& field,
    Eigen::MatrixXd *target) {

  int n = std::sqrt(field.size());
  if (field.size() != std::pow(n, 2)) {
    return false;
  }

  target->resize(n, n);

  for (int r = 0; r < n; r++) {
    for (int c = 0; c < n; c++) {
      if (field[r * n + c].getType() != XmlRpc::XmlRpcValue::TypeDouble) {
        return false;
      }
      (*target)(r, c) = static_cast<double>(field[r * n + c]);
    }
  }

  return true;
}

std::ostream& operator <<(std::ostream& s, const SensorConfiguration& conf) {

  s << "Sensor " << conf.name_ << std::endl;
  s << "\ttype: " << conf.type_ << std::endl;
  s << "\tis master?: " << (conf.is_master_ ? "Yes" : "No") << std::endl;
  s << "\tframe id: " << conf.frame_id_ << std::endl;
  s << "\ttopic: " << conf.topic_ << std::endl;
  s << "\ttopic type: " << conf.topic_type_ << std::endl;
  s << "\tuse header timestamp: " << conf.use_header_stamp_ << std::endl;
  s << "\tstatic covariance: " << conf.static_covariance_ << std::endl;

  if (conf.static_covariance_){
    s << *(conf.covariance_) << std::endl;
  }

  if (conf.parameters_.size() > 0) {
    s << "\tparameters:" << std::endl;

    for (std::map<std::string, Eigen::VectorXd *>::const_iterator it =
        conf.parameters_.begin(); it != conf.parameters_.end(); ++it) {
      s << "\t\t" << it->first << ": " << it->second->transpose() << std::endl;
    }
  }

  return s;
}

} /* namespace roamros */
