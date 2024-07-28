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
  rclcpp::Node::SharedPtr nh;
  ControllerRos(rclcpp::Node::SharedPtr& nh);
  ~ControllerRos();

  /**
   * @brief Must implement to set the reference for the controller
   * @return
   */
  virtual bool update_reference();

  double policy_update_rate_ = 0.0;
  double reference_update_rate_ = 0.0;
  bool publish_ros_ = false;
  double ros_publish_rate_ = 0.0;
  bool init_default_params();

  void init_default_ros();

  /**
   * @brief Must implement to set the sampling based controller
   * @param controller
   */
  virtual bool set_controller(std::shared_ptr<PathIntegral>& controller) = 0;

  virtual bool init_ros() { return true; };
  virtual void publish_ros(){};

  /**
   * @brief Init the ros pub/sub and set the controller object
   * @return true if initialization was successful
   */
  bool init();

  /**
   * @brief Starts all the threads
   * @return
   */
  bool start();

  /**
   * @brief Stop the running controller and associated threads
   */
  void stop();


  mppi::threading::WorkerManager worker_manager_;
};

}  // namespace mppi_ros
