///////////////////////////////////////////////////////////////////////////////
// BSD 3-Clause License
//
// Copyright (C) 2023-2023, Heriot-Watt University
// Copyright note valid unless otherwise stated in individual files.
// All rights reserved.
///////////////////////////////////////////////////////////////////////////////

#ifndef IHMC_MSG_SOLVER_TRAJECTORY_PUBLISHER_H_
#define IHMC_MSG_SOLVER_TRAJECTORY_PUBLISHER_H_

#include "ihmc_msgs/conversions.h"

#include <Eigen/Dense>
#include <realtime_tools/realtime_publisher.h>

#include <rclcpp/rclcpp.hpp>
#include "ihmc_msgs/msg/solver_trajectory.hpp"

namespace ihmc_msgs {

class SolverTrajectoryRosPublisher {
 public:
  /**
   * @brief Initialize the solver trajectory publisher
   *
   * @param[in] topic  Topic name
   * @param[in] frame  Odometry frame
   */
  SolverTrajectoryRosPublisher(const std::string &topic = "/ihmc/solver_trajectory",
                               const std::string &frame = "odom")
      : node_("solver_trajectory_publisher"),
        pub_(node_.create_publisher<ihmc_msgs::msg::SolverTrajectory>(topic, 1)) {
    pub_.msg_.header.frame_id = frame;
    std::cout << "Publish SolverTrajectory messages on " << topic << " (frame: " << frame << ")" << std::endl;
  }
  ~SolverTrajectoryRosPublisher() = default;

  /**
   * @brief Publish a solver trajectory ROS message
   *
   * @param ts[in]      Vector of time at the beginning of the interval
   * @param dts[in]     Vector of time durations of each interval
   * @param xs[in]      Vector of states of each interval
   * @param dxs[in]     Vector of state's rate of changes of each interval
   * @param us[in]      Vector of feed-forward control parameters of each
   * interval
   * @param Ks[in]      Vector of feedback gain in the control parametrized
   * space of each interval
   * @param types[in]   Vector of control types of each interval
   * @param params[in]  Vector of control parametrizations of each interval
   */
  void publish(const std::vector<double> &ts, const std::vector<double> &dts, const std::vector<Eigen::VectorXd> &xs,
               const std::vector<Eigen::VectorXd> &dxs, const std::vector<Eigen::VectorXd> &us = {},
               const std::vector<Eigen::MatrixXd> &Ks = {}, const std::vector<ControlType> &types = {},
               const std::vector<ControlParametrization> &params = {}) {
    if (pub_.trylock()) {
      if (ts.size() != dts.size()) {
        throw std::invalid_argument(
            "The size of the ts vector needs to equal "
            "the size of the dts vector.");
      }
      if (ts.size() != xs.size()) {
        throw std::invalid_argument(
            "The size of the ts vector needs to equal "
            "the size of the xs vector.");
      }
      if (ts.size() != dxs.size()) {
        throw std::invalid_argument(
            "The size of the ts vector needs to equal "
            "the size of the dxs vector.");
      }
      if (us.size() != 0 && ts.size() != us.size()) {
        throw std::invalid_argument(
            "The size of the ts vector needs to equal "
            "the size of the us vector.");
      }
      if (Ks.size() != 0 && ts.size() != Ks.size()) {
        throw std::invalid_argument(
            "The size of the ts vector needs to equal "
            "the size of the Ks vector.");
      }
      if (types.size() != 0 && ts.size() != types.size()) {
        throw std::invalid_argument(
            "The size of the ts vector needs to equal "
            "the size of the types.");
      }
      if (params.size() != 0 && ts.size() != params.size()) {
        throw std::invalid_argument(
            "The size of the ts vector needs to equal "
            "the size of the params.");
      }
      const std::size_t N = ts.size();
      pub_.msg_.header.stamp = node_.now();
      pub_.msg_.intervals.resize(N);
      pub_.msg_.state_trajectory.resize(N);
      pub_.msg_.control_trajectory.resize(N);
      for (std::size_t i = 0; i < N; ++i) {
        pub_.msg_.intervals[i].time = ts[i];
        pub_.msg_.intervals[i].duration = dts[i];
        ihmc_msgs::toMsg(pub_.msg_.state_trajectory[i], xs[i], dxs[i]);
        ihmc_msgs::toMsg(pub_.msg_.control_trajectory[i], us[i], Ks[i], types[i], params[i]);
      }
      pub_.unlockAndPublish();
    }
  }

 private:
  rclcpp::Node node_;
  realtime_tools::RealtimePublisher<ihmc_msgs::msg::SolverTrajectory> pub_;
};

}  // namespace ihmc_msgs

#endif  // IHMC_MSG_SOLVER_TRAJECTORY_PUBLISHER_H_
