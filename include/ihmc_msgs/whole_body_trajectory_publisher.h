///////////////////////////////////////////////////////////////////////////////
// BSD 3-Clause License
//
// Copyright (C) 2023-2023, Heriot-Watt University
// Copyright note valid unless otherwise stated in individual files.
// All rights reserved.
///////////////////////////////////////////////////////////////////////////////

#ifndef IHMC_MSG_WHOLE_BODY_TRAJECTORY_PUBLISHER_H_
#define IHMC_MSG_WHOLE_BODY_TRAJECTORY_PUBLISHER_H_

#include "ihmc_msgs/conversions.h"

#include <realtime_tools/realtime_publisher.h>

#include <rclcpp/rclcpp.hpp>

namespace ihmc_msgs {

class WholeBodyTrajectoryRosPublisher {
 public:
  /**
   * @brief Initialize the whole-body trajectory publisher
   *
   * @param[in] model  Pinocchio model
   * @param[in] topic  Topic name
   * @param[in] frame  Odometry frame
   * @param[in] queue  Queue size
   */
  WholeBodyTrajectoryRosPublisher(pinocchio::Model &model,
                                  const std::string &topic = "/ihmc/whole_body_trajectory",
                                  const std::string &frame = "odom", int queue = 10)
      : node_("whole_body_trajectory_publisher"),
        pub_(node_.create_publisher<WholeBodyTrajectory>(topic, queue)),
        model_(model),
        data_(model),
        odom_frame_(frame),
        a_null_(model.nv) {
    pub_.msg_.header.frame_id = frame;
    a_null_.setZero();
    std::cout << "Publish WholeBodyTrajectory messages on " << topic << " (frame: " << frame << ")" << std::endl;
  }
  ~WholeBodyTrajectoryRosPublisher() = default;

  /**
   * @brief Publish a whole-body trajectory ROS message
   *
   * @param ts[in]   Vector of interval times in secs
   * @param xs[in]   Vector of states (dimension: model.nq)
   * @param us[in]   Vector of joint efforts (dimension: model.nv)
   * @param ps[in]   Vector of contact positions
   * @param pds[in]  Vector of contact velocities
   * @param fs[in]   Vector of contact forces, types and statuses
   * @param ss[in]   Vector of contact surfaces and friction coefficients
   */
  void publish(const std::vector<double> &ts, const std::vector<Eigen::VectorXd> &xs,
               const std::vector<Eigen::VectorXd> &us, const std::vector<std::map<std::string, pinocchio::SE3>> &ps,
               const std::vector<std::map<std::string, pinocchio::Motion>> &pds,
               const std::vector<std::map<std::string, std::tuple<pinocchio::Force, ContactType, ContactStatus>>> &fs,
               const std::vector<std::map<std::string, std::pair<Eigen::Vector3d, double>>> &ss) {
    if (pub_.trylock()) {
      if (ts.size() != xs.size()) {
        throw std::invalid_argument(
            "The size of the ts vector needs to equal "
            "the size of the xs vector.");
      }
      if (ts.size() != us.size()) {
        throw std::invalid_argument(
            "If provided, the size of the us vector needs to equal the size of "
            "the ts vector.");
      }
      if (ts.size() != ps.size()) {
        throw std::invalid_argument(
            "If provided, the size of the ps vector needs to equal the size of "
            "the ts vector.");
      }
      if (ts.size() != pds.size()) {
        throw std::invalid_argument(
            "If provided, the size of the pds vector "
            "needs to equal the size of "
            "the ts vector.");
      }
      if (ts.size() != fs.size()) {
        throw std::invalid_argument(
            "If provided, the size of the fs vector needs to equal the size of "
            "the ts vector.");
      }
      if (ts.size() != ss.size()) {
        throw std::invalid_argument(
            "If provided, the size of the ss vector needs to equal the size of "
            "the ts vector.");
      }
      pub_.msg_.header.frame_id = odom_frame_;
      pub_.msg_.header.stamp = node_.now();
      pub_.msg_.trajectory.resize(ts.size());
      for (std::size_t i = 0; i < ts.size(); ++i) {
        pub_.msg_.trajectory[i].header.frame_id = odom_frame_;
        ihmc_msgs::toMsg(model_, data_, pub_.msg_.trajectory[i], ts[i], xs[i].head(model_.nq),
                              xs[i].tail(model_.nv), a_null_, us[i], ps[i], pds[i], fs[i], ss[i]);
      }
      pub_.unlockAndPublish();
    } else {
      RCLCPP_WARN_STREAM(node_.get_logger(), "[publish] Could not lock publisher, not published feedback policy");
    }
  }

 private:
  rclcpp::Node node_;
  realtime_tools::RealtimePublisher<WholeBodyTrajectory> pub_;
  pinocchio::Model model_;
  pinocchio::Data data_;
  std::string odom_frame_;
  Eigen::VectorXd a_null_;
};

}  // namespace ihmc_msgs

#endif  // IHMC_MSG_WHOLE_BODY_TRAJECTORY_PUBLISHER_H_
