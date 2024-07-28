//
// Created by giuseppe on 01.03.21.
//

#pragma once
// #include <ros/ros.h>

namespace mppi_ros {

template <typename T>
bool getNonNegative(rclcpp::Node::SharedPtr& nh, const std::string& param_name,
                    T& obj) {
  if (!nh->get_parameter(param_name, obj)) {
    // ROS_ERROR_STREAM("Failed to parse param " << param_name);
    return false;
  }

  if (obj < 0) {
  //   ROS_ERROR_STREAM("Failed to parse param " << param_name
  //                                             << ". Invalid value: " << obj);
    return false;
  }
  return true;
}

template <typename T>
bool getNVector(rclcpp::Node::SharedPtr& nh, const std::string& param_name,
                std::vector<T>& obj, size_t dim) {
  if (!nh->get_parameter(param_name, obj)) {
    // ROS_ERROR_STREAM("Failed to parse param " << param_name);
    return false;
  }

  if (obj.size() != dim) {
    // ROS_ERROR_STREAM("Failed to parse param "
    //                  << param_name << ". Invalid vector size: " << obj.size());
    return false;
  }
  return true;
}

bool getString(rclcpp::Node::SharedPtr& nh, const std::string& param_name,
               std::string& obj) {
  if (!nh->get_parameter(param_name, obj)) {
    // ROS_ERROR_STREAM("Failed to parse param " << param_name);
    return false;
  }

  if (obj.empty()) {
    // ROS_ERROR_STREAM("Failed to parse param " << param_name
    //                                           << ". String is empty.");
    return false;
  }
  return true;
}

bool getBool(rclcpp::Node::SharedPtr& nh, const std::string& param_name, bool& obj) {
  if (!nh->get_parameter(param_name, obj)) {
    // ROS_ERROR_STREAM("Failed to parse param " << param_name);
    return false;
  }
  return true;
}

}  // namespace mppi_ros