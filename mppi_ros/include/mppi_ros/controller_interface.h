/*!
 * @file     controller_ros.h
 * @author   Giuseppe Rizzi
 * @date     08.09.2020
 * @version  1.0
 * @brief    description
 */
#pragma once
#include <mppi/controller/mppi.h>

#include <mppi_ros/threading/WorkerManager.hpp>

#include <mppi_ros_interfaces/msg/data.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <std_msgs/msg/float64.hpp>

#include "rclcpp/rclcpp.hpp"

using namespace mppi;

namespace mppi_ros {

class ControllerRos {
 public:
  ControllerRos() = delete;
  ControllerRos(rclcpp::Node::SharedPtr& nh);
  ~ControllerRos();

  rclcpp::Node::SharedPtr nh;

  mppi::threading::WorkerManager worker_manager_;
};

}  // namespace mppi_ros
