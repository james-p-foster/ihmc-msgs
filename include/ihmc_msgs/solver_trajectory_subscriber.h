///////////////////////////////////////////////////////////////////////////////
// BSD 3-Clause License
//
// Copyright (C) 2023-2023, Heriot-Watt University
// Copyright note valid unless otherwise stated in individual files.
// All rights reserved.
///////////////////////////////////////////////////////////////////////////////

#ifndef IHMC_MSG_SOLVER_TRAJECTORY_SUBSCRIBER_H_
#define IHMC_MSG_SOLVER_TRAJECTORY_SUBSCRIBER_H_

#include "ihmc_msgs/conversions.h"

#include <Eigen/Dense>
#include <mutex>
#include <rclcpp/rclcpp.hpp>
#include "ihmc_msgs/msg/solver_trajectory.hpp"

namespace ihmc_msgs {

typedef msg::SolverTrajectory SolverTrajectory;
typedef const SolverTrajectory::SharedPtr SolverTrajectorySharedPtr;

class SolverTrajectoryRosSubscriber {
 public:
  /**
   * @brief Initialize the solver trajectory subscriber
   *
   * @param[in] topic  Topic name
   * @param[in] frame  Odometry frame
   */
  SolverTrajectoryRosSubscriber(const std::string &topic = "/ihmc/solver_trajectory")
      : node_(rclcpp::Node::make_shared("solver_trajectory_subscriber")),
        sub_(node_->create_subscription<SolverTrajectory>(
            topic, 1, std::bind(&SolverTrajectoryRosSubscriber::callback, this, std::placeholders::_1))),
        has_new_msg_(false),
        is_processing_msg_(false),
        last_msg_time_(0.) {
    spinner_.add_node(node_);
    thread_ = std::thread([this]() { this->spin(); });
    thread_.detach();
    std::cout << "Subscribe to SolverTrajectory messages on " << topic << std::endl;
  }
  ~SolverTrajectoryRosSubscriber() = default;

  /**
   * @brief Get the latest solver trajectory
   *
   * @return  A tuple with the vector of time at the beginning of the interval,
   * its durations, initial state, state's rate of change, feed-forward control,
   * feedback gain, type of control and control parametrization.
   */
  std::tuple<std::vector<double>, std::vector<double>, std::vector<Eigen::VectorXd>, std::vector<Eigen::VectorXd>,
             std::vector<Eigen::VectorXd>, std::vector<Eigen::MatrixXd>, std::vector<ihmc_msgs::ControlType>,
             std::vector<ihmc_msgs::ControlParametrization>>
  get_solver_trajectory() {
    // start processing the message
    is_processing_msg_ = true;
    std::lock_guard<std::mutex> guard(mutex_);
    const std::size_t N = msg_.intervals.size();
    if (msg_.state_trajectory.size() != N) {
      throw std::invalid_argument(
          "The size of the state trajectory vector needs to equal "
          "the size of the intervals vector.");
    }
    if (msg_.control_trajectory.size() != 0 && msg_.control_trajectory.size() != N) {
      throw std::invalid_argument(
          "The size of the control trajectory vector needs to equal "
          "the size of the intervals vector.");
    }
    ts_.resize(N);
    dts_.resize(N);
    xs_.resize(N);
    dxs_.resize(N);
    us_.resize(N);
    Ks_.resize(N);
    types_.resize(N);
    params_.resize(N);
    for (std::size_t i = 0; i < N; ++i) {
      const TimeInterval &interval = msg_.intervals[i];
      const State &state = msg_.state_trajectory[i];
      const Control &control = msg_.control_trajectory[i];
      ts_[i] = interval.time;
      dts_[i] = interval.duration;
      xs_[i].resize(state.x.size());
      dxs_[i].resize(state.dx.size());
      us_[i].resize(control.u.size());
      Ks_[i].resize(control.gain.nu, control.gain.nx);
      ihmc_msgs::fromMsg(state, xs_[i], dxs_[i]);
      ihmc_msgs::fromMsg(control, us_[i], Ks_[i], types_[i], params_[i]);
    }
    // finish processing the message
    is_processing_msg_ = false;
    has_new_msg_ = false;
    return {ts_, dts_, xs_, dxs_, us_, Ks_, types_, params_};
  }

  /**
   * @brief Indicate whether we have received a new message
   */
  bool has_new_msg() const { return has_new_msg_; }

 private:
  std::shared_ptr<rclcpp::Node> node_;
  rclcpp::executors::SingleThreadedExecutor spinner_;
  std::thread thread_;
  void spin() { spinner_.spin(); }
  rclcpp::Subscription<SolverTrajectory>::SharedPtr sub_;  //!< ROS subscriber
  std::mutex mutex_;        //!< Mutex to prevent race condition on callback
  SolverTrajectory msg_;    //!< Solver trajectory message
  bool has_new_msg_;        //!< Indcate when a new message has been received
  bool is_processing_msg_;  //!< Indicate when we are processing the message
  double last_msg_time_;    //!< Last message time needed to ensure each message is
                            //!< newer
  std::vector<double> ts_;
  std::vector<double> dts_;
  std::vector<Eigen::VectorXd> xs_;
  std::vector<Eigen::VectorXd> dxs_;
  std::vector<Eigen::VectorXd> us_;
  std::vector<Eigen::MatrixXd> Ks_;
  std::vector<ihmc_msgs::ControlType> types_;
  std::vector<ihmc_msgs::ControlParametrization> params_;
  void callback(SolverTrajectorySharedPtr msg) {
    if (!is_processing_msg_) {
      double t = rclcpp::Time(msg->header.stamp).seconds();
      // Avoid out of order arrival and ensure each message is newer (or equal
      // to) than the preceeding:
      if (last_msg_time_ <= t) {
        std::lock_guard<std::mutex> guard(mutex_);
        msg_ = *msg;
        has_new_msg_ = true;
        last_msg_time_ = t;
      } else {
        RCLCPP_WARN_STREAM(node_->get_logger(), "Out of order message. Last timestamp: "
                                                    << std::fixed << last_msg_time_ << ", current timestamp: " << t);
      }
    }
  }
};

}  // namespace ihmc_msgs

#endif  // IHMC_MSG_SOLVER_TRAJECTORY_SUBSCRIBER_H_
