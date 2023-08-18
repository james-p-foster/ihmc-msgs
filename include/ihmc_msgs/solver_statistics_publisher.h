///////////////////////////////////////////////////////////////////////////////
// BSD 3-Clause License
//
// Copyright (C) 2020-2023, Heriot-Watt University, University of Oxford
// Copyright note valid unless otherwise stated in individual files.
// All rights reserved.
///////////////////////////////////////////////////////////////////////////////

#ifndef CROCODDYL_MSG_SOLVER_STATISTICS_PUBLISHER_H_
#define CROCODDYL_MSG_SOLVER_STATISTICS_PUBLISHER_H_

#include <realtime_tools/realtime_publisher.h>

#include <rclcpp/rclcpp.hpp>
#include "ihmc_msgs//msg/solver_statistics.hpp"

namespace ihmc_msgs {

class SolverStatisticsRosPublisher {
 public:
  /**
   * @brief Initialize the solver statistic publisher
   *
   * @param[in] topic  Topic name
   */
  SolverStatisticsRosPublisher(const std::string &topic = "/crocoddyl/solver_statistics")
      : node_("solver_statistics_publisher"),
        pub_(node_.create_publisher<ihmc_msgs::msg::SolverStatistics>(topic, 1)) {
  }
  ~SolverStatisticsRosPublisher() = default;

  /**
   * @brief Publish a solver statistic ROS message
   *
   * @param iterations[in]      Number of solver iterations
   * @param totaltime[in]       Total time
   * @param solvetime[in]       Solving time
   * @param cost[in]            Total cost
   * @param regularization[in]  Regularization value
   * @param steplength[in]      Step length applied by the solver
   * @param dynfeas[in]         Dynamic feasibility
   * @param equafeas[in]        Equality constraints feasibility
   * @param ineqfeas[in]        Inequality constraints feasibility
   */
  void publish(const std::size_t iterations, const double totaltime, const double solvetime, const double cost,
               const double regularization, const double steplength, const double dynfeas, const double equafeas,
               const double ineqfeas) {
    if (pub_.trylock()) {
      pub_.msg_.header.stamp = node_.now();
      pub_.msg_.iterations = iterations;
      pub_.msg_.total_time = totaltime;
      pub_.msg_.solve_time = solvetime;
      pub_.msg_.cost = cost;
      pub_.msg_.regularization = regularization;
      pub_.msg_.step_length = steplength;
      pub_.msg_.dynamic_feasibility = dynfeas;
      pub_.msg_.equality_feasibility = equafeas;
      pub_.msg_.inequality_feasibility = ineqfeas;
      pub_.unlockAndPublish();
    }
  }

 private:
  rclcpp::Node node_;
  realtime_tools::RealtimePublisher<ihmc_msgs::msg::SolverStatistics> pub_;
};

}  // namespace crocoddyl_msgs

#endif  // CROCODDYL_MSG_SOLVER_STATISTICS_PUBLISHER_H_
