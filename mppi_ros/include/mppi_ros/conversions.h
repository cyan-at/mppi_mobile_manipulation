//
// Created by giuseppe on 05.02.21.
//

#pragma once

#include <mppi/controller/data.h>
#include <mppi/controller/rollout.h>
#include <mppi/solver_config.h>

// #include "mppi_ros_interface/Array.h"
// #include "mppi_ros_interface/Config.h"
// #include "mppi_ros_interface/Data.h"
// #include "mppi_ros_interface/Rollout.h"

// must be hpp
#include "mppi_ros_interfaces/msg/array.hpp"
#include "mppi_ros_interfaces/msg/config.hpp"
#include "mppi_ros_interfaces/msg/data.hpp"
#include "mppi_ros_interfaces/msg/rollout.hpp"

namespace mppi_ros {

void to_msg(const mppi::SolverConfig& config, mppi_ros_interfaces::msg::Config& config_ros);
void to_msg(const mppi::Rollout& rollout, mppi_ros_interfaces::msg::Rollout& rollout_ros);
void to_msg(const mppi::data_t& data, mppi_ros_interfaces::msg::Data& data_ros);

}  // namespace mppi_ros