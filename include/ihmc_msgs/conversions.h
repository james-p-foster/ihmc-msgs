///////////////////////////////////////////////////////////////////////////////
// BSD 3-Clause License
//
// Copyright (C) 2020-2023, Heriot-Watt University, University of Oxford
// Copyright note valid unless otherwise stated in individual files.
// All rights reserved.
///////////////////////////////////////////////////////////////////////////////

#ifndef CONVERSIONS_H_
#define CONVERSIONS_H_

#include <pinocchio/fwd.hpp>

#include <pinocchio/algorithm/center-of-mass.hpp>
#include <pinocchio/algorithm/centroidal.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/container/aligned-vector.hpp>
#include <pinocchio/multibody/data.hpp>
#include <pinocchio/multibody/model.hpp>
#include <pinocchio/spatial/force.hpp>
#include <pinocchio/spatial/motion.hpp>

#include <pinocchio/bindings/python/pybind11.hpp>
#define SCALAR double
#define OPTIONS 0
#define JOINT_MODEL_COLLECTION ::pinocchio::JointCollectionDefaultTpl
#include <pinocchio/bindings/python/pybind11-all.hpp>

#include <rosidl_runtime_cpp/bounded_vector.hpp>

#include "ihmc_msgs/msg/time_interval.hpp"
#include "ihmc_msgs/msg/control.hpp"
#include "ihmc_msgs/msg/feedback_gain.hpp"
#include "ihmc_msgs/msg/state.hpp"
#include "ihmc_msgs/msg/capturability_based_status.hpp"
#include "ihmc_msgs/msg/robot_configuration_data.hpp"
#include <geometry_msgs/msg/point.hpp>
#include <whole_body_state_msgs/msg/whole_body_state.hpp>
#include <whole_body_state_msgs/msg/whole_body_trajectory.hpp>

namespace ihmc_msgs {

enum ControlType { EFFORT = 0, ACCELERATION_CONTACTFORCE };

enum ControlParametrization { POLYZERO = 0, POLYONE, POLYTWO };

enum ContactType { LOCOMOTION = 0, MANIPULATION };

enum ContactStatus { UNKNOWN = 0, SEPARATION, STICKING, SLIPPING };

typedef ihmc_msgs::msg::TimeInterval TimeInterval;
typedef ihmc_msgs::msg::State State;
typedef ihmc_msgs::msg::Control Control;
typedef ihmc_msgs::msg::FeedbackGain FeedbackGain;
typedef ihmc_msgs::msg::RobotConfigurationData RobotConfigurationData;
typedef ihmc_msgs::msg::CapturabilityBasedStatus CapturabilityBasedStatus;
typedef geometry_msgs::msg::Point Point;
typedef whole_body_state_msgs::msg::WholeBodyState WholeBodyState;
typedef whole_body_state_msgs::msg::WholeBodyTrajectory WholeBodyTrajectory;
typedef whole_body_state_msgs::msg::ContactState ContactState;

/**
 * @brief Conversion of Eigen to message for a given
 * ihmc_msgs::FeedbackGain message reference
 *
 * @param[out] msg  ROS message that contains the feedback gain
 * @param[in] K     Feedback gain (size nu * nx)
 */
static inline void toMsg(FeedbackGain &msg, const Eigen::Ref<const Eigen::MatrixXd> &K) {
  msg.nu = static_cast<uint32_t>(K.rows());
  msg.nx = static_cast<uint32_t>(K.cols());
  msg.data.resize(msg.nx * msg.nu);
  for (uint32_t i = 0; i < msg.nu; ++i) {
    for (uint32_t j = 0; j < msg.nx; ++j) {
      msg.data[i * msg.nx + j] = K(i, j);  // store in row-major order
    }
  }
}

/**
 * @brief Conversion of Eigen to message for a given ihmc_msgs::State
 * message reference
 *
 * @param[out] msg  ROS message that contains the state
 * @param[in] x     State at the beginning of the interval
 * @param[in] dx    State's rate of change during the interval
 */
static inline void toMsg(State &msg, const Eigen::Ref<const Eigen::VectorXd> &x,
                         const Eigen::Ref<const Eigen::VectorXd> &dx) {
  msg.x.resize(x.size());
  msg.dx.resize(dx.size());
  for (int i = 0; i < x.size(); ++i) {
    msg.x[i] = x(i);
  }
  for (int i = 0; i < dx.size(); ++i) {
    msg.dx[i] = dx(i);
  }
}

/**
 * @brief Conversion of Eigen to message for a given ihmc_msgs::Control
 * message reference
 *
 * @param[out] msg             ROS message that contains the control
 * @param[in] u                Control parameters of the interval
 * @param[in] K                Feedback gain of the interval
 * @param[in] type             Control type
 * @param[in] parametrization  Control parametrization
 */
static inline void toMsg(Control &msg, const Eigen::Ref<const Eigen::VectorXd> &u,
                         const Eigen::Ref<const Eigen::MatrixXd> &K, const ControlType type,
                         const ControlParametrization parametrization) {
  msg.u.resize(u.size());
  for (int i = 0; i < u.size(); ++i) {
    msg.u[i] = u(i);
  }
  toMsg(msg.gain, K);
  switch (type) {
    case ControlType::EFFORT:
      msg.input = ihmc_msgs::Control::EFFORT;
      break;
    case ControlType::ACCELERATION_CONTACTFORCE:
      msg.input = ihmc_msgs::Control::ACCELERATION_CONTACTFORCE;
      break;
  }
  switch (parametrization) {
    case ControlParametrization::POLYZERO:
      msg.parametrization = ihmc_msgs::Control::POLYZERO;
      break;
    case ControlParametrization::POLYONE:
      msg.parametrization = ihmc_msgs::Control::POLYONE;
      break;
    case ControlParametrization::POLYTWO:
      msg.parametrization = ihmc_msgs::Control::POLYTWO;
      break;
  }
}

/**
 * @brief Conversion from vectors to `whole_body_state_msgs::WholeBodyState`
 *
 * @param model[in]  Pinocchio model
 * @param data[out]  Pinocchio data
 * @param msg[out]   ROS message that contains the whole-body state
 * @param t[in]      Time in secs
 * @param q[in]      Configuration vector (dimension: model.nq)
 * @param v[in]      Generalized velocity (dimension: model.nv)
 * @param a[in]      Generalized acceleration (dimension: model.nv)
 * @param tau[in]    Joint effort
 * @param p[in]      Contact position
 * @param pd[in]     Contact velocity
 * @param f[in]      Contact force, type and status
 * @param s[in]      Contact surface and friction coefficient
 */
template <int Options, template <typename, int> class JointCollectionTpl>
static inline void toMsg(const pinocchio::ModelTpl<double, Options, JointCollectionTpl> &model,
                         pinocchio::DataTpl<double, Options, JointCollectionTpl> &data, WholeBodyState &msg,
                         const double t, const Eigen::Ref<const Eigen::VectorXd> &q,
                         const Eigen::Ref<const Eigen::VectorXd> &v, const Eigen::Ref<const Eigen::VectorXd> &a,
                         const Eigen::Ref<const Eigen::VectorXd> &tau, const std::map<std::string, pinocchio::SE3> &p,
                         const std::map<std::string, pinocchio::Motion> &pd,
                         const std::map<std::string, std::tuple<pinocchio::Force, ContactType, ContactStatus>> &f,
                         const std::map<std::string, std::pair<Eigen::Vector3d, double>> &s) {
  if (q.size() != model.nq) {
    throw std::invalid_argument("Expected q to be " + std::to_string(model.nq) + " but received " +
                                std::to_string(q.size()));
  }
  if (v.size() != model.nv) {
    throw std::invalid_argument("Expected v to be " + std::to_string(model.nv) + " but received " +
                                std::to_string(v.size()));
  }
  if (a.size() != model.nv) {
    throw std::invalid_argument("Expected a to be " + std::to_string(model.nv) + " but received " +
                                std::to_string(a.size()));
  }
  const std::size_t nv_root = model.joints[1].idx_q() == 0 ? model.joints[1].nv() : 0;
  const std::size_t njoints = model.nv - nv_root;
  if (tau.size() != static_cast<int>(njoints) && tau.size() != 0) {
    throw std::invalid_argument("Expected tau to be 0 or " + std::to_string(njoints) + " but received " +
                                std::to_string(tau.size()));
  }
  if (p.size() != pd.size()) {
    throw std::invalid_argument("Dimension of contact pose and velocity does not match.");
  }
  if (p.size() != f.size()) {
    throw std::invalid_argument("Dimension of contact pose and force does not match.");
  }
  if (p.size() != s.size()) {
    throw std::invalid_argument("Dimension of contact pose and surface does not match.");
  }
  // Filling the time information
  msg.time = t;
  msg.header.stamp = rclcpp::Time(static_cast<uint64_t>(t * 1e9));
  // Filling the centroidal state
  pinocchio::centerOfMass(model, data, q, v, a);
  // Center of mass
  msg.centroidal.com_position.x = data.com[0].x();
  msg.centroidal.com_position.y = data.com[0].y();
  msg.centroidal.com_position.z = data.com[0].z();
  // Velocity of the CoM expressed in the global frame.
  msg.centroidal.com_velocity.x = data.vcom[0].x();
  msg.centroidal.com_velocity.y = data.vcom[0].y();
  msg.centroidal.com_velocity.z = data.vcom[0].z();
  // Base
  if (nv_root == 6) {  // TODO(cmastalli): handle other root joints
    msg.centroidal.base_orientation.x = q(3);
    msg.centroidal.base_orientation.y = q(4);
    msg.centroidal.base_orientation.z = q(5);
    msg.centroidal.base_orientation.w = q(6);
    msg.centroidal.base_angular_velocity.x = v(3);
    msg.centroidal.base_angular_velocity.y = v(4);
    msg.centroidal.base_angular_velocity.z = v(5);
  } else if (nv_root != 0) {
    std::cerr << "Warning: toMsg conversion does not yet support root joints "
                 "different to a floating base. We cannot publish base information."
              << std::endl;
  }
  // Momenta
  const pinocchio::Force &momenta = pinocchio::computeCentroidalMomentum(model, data);
  msg.centroidal.momenta.linear.x = momenta.linear().x();
  msg.centroidal.momenta.linear.y = momenta.linear().y();
  msg.centroidal.momenta.linear.z = momenta.linear().z();
  msg.centroidal.momenta.angular.x = momenta.angular().x();
  msg.centroidal.momenta.angular.y = momenta.angular().y();
  msg.centroidal.momenta.angular.z = momenta.angular().z();
  const pinocchio::Force &momenta_rate = pinocchio::computeCentroidalMomentumTimeVariation(model, data);
  msg.centroidal.momenta_rate.linear.x = momenta_rate.linear().x();
  msg.centroidal.momenta_rate.linear.y = momenta_rate.linear().y();
  msg.centroidal.momenta_rate.linear.z = momenta_rate.linear().z();
  msg.centroidal.momenta_rate.angular.x = momenta_rate.angular().x();
  msg.centroidal.momenta_rate.angular.y = momenta_rate.angular().y();
  msg.centroidal.momenta_rate.angular.z = momenta_rate.angular().z();
  // Filling the joint state
  msg.joints.resize(njoints);
  for (std::size_t j = 0; j < njoints; ++j) {
    msg.joints[j].name = model.names[2 + j];
    msg.joints[j].position = q(model.joints[1].nq() + j);
    msg.joints[j].velocity = v(model.joints[1].nv() + j);
    msg.joints[j].acceleration = a(model.joints[1].nv() + j);
    msg.joints[j].effort = tau(j);
  }
  // Contacts
  msg.contacts.resize(p.size());
  std::size_t i = 0;
  for (const auto &p_item : p) {
    const std::string &name = p_item.first;
    msg.contacts[i].name = name;
    pinocchio::FrameIndex frame_id = model.getFrameId(name);
    if (static_cast<int>(frame_id) > model.nframes) {
      throw std::runtime_error("Frame '" + name + "' not found.");
    }
    std::map<std::string, pinocchio::Motion>::const_iterator pd_it = pd.find(name);
    if (pd_it == pd.end()) {
      throw std::runtime_error("Frame '" + name + "' not found in pd.");
    }
    std::map<std::string, std::tuple<pinocchio::Force, ContactType, ContactStatus>>::const_iterator f_it =
        f.find(name);
    if (f_it == f.end()) {
      throw std::runtime_error("Frame '" + name + "' not found in f.");
    }
    std::map<std::string, std::pair<Eigen::Vector3d, double>>::const_iterator s_it = s.find(name);
    if (s_it == s.end()) {
      throw std::runtime_error("Frame '" + name + "' not found in s.");
    }
    ++i;
  }
  i = 0;
  for (const auto &p_item : p) {
    const pinocchio::SE3 &pose = p_item.second;
    pinocchio::SE3::Quaternion quaternion(pose.rotation());
    msg.contacts[i].pose.position.x = pose.translation().x();
    msg.contacts[i].pose.position.y = pose.translation().y();
    msg.contacts[i].pose.position.z = pose.translation().z();
    msg.contacts[i].pose.orientation.x = quaternion.x();
    msg.contacts[i].pose.orientation.y = quaternion.y();
    msg.contacts[i].pose.orientation.z = quaternion.z();
    msg.contacts[i].pose.orientation.w = quaternion.w();
    ++i;
  }
  i = 0;
  for (const auto &pd_item : pd) {
    const pinocchio::Motion &vel = pd_item.second;
    msg.contacts[i].velocity.linear.x = vel.linear().x();
    msg.contacts[i].velocity.linear.y = vel.linear().y();
    msg.contacts[i].velocity.linear.z = vel.linear().z();
    msg.contacts[i].velocity.angular.x = vel.angular().x();
    msg.contacts[i].velocity.angular.y = vel.angular().y();
    msg.contacts[i].velocity.angular.z = vel.angular().z();
    ++i;
  }
  i = 0;
  for (const auto &f_item : f) {
    const std::tuple<pinocchio::Force, ContactType, ContactStatus> &force = f_item.second;
    const pinocchio::Force &wrench = std::get<0>(force);
    const ContactType type = std::get<1>(force);
    switch (type) {
      case ContactType::LOCOMOTION:
        msg.contacts[i].type = ContactState::LOCOMOTION;
        break;
      case ContactType::MANIPULATION:
        msg.contacts[i].type = ContactState::MANIPULATION;
        break;
    }
    const ContactStatus status = std::get<2>(force);
    switch (status) {
      case ContactStatus::UNKNOWN:
        msg.contacts[i].status = ContactState::UNKNOWN;
        break;
      case ContactStatus::SEPARATION:
        msg.contacts[i].status = ContactState::INACTIVE;
        break;
      case ContactStatus::STICKING:
        msg.contacts[i].status = ContactState::ACTIVE;
        break;
      case ContactStatus::SLIPPING:
        msg.contacts[i].status = ContactState::SLIPPING;
        break;
    }
    msg.contacts[i].wrench.force.x = wrench.linear().x();
    msg.contacts[i].wrench.force.y = wrench.linear().y();
    msg.contacts[i].wrench.force.z = wrench.linear().z();
    msg.contacts[i].wrench.torque.x = wrench.angular().x();
    msg.contacts[i].wrench.torque.y = wrench.angular().y();
    msg.contacts[i].wrench.torque.z = wrench.angular().z();
    ++i;
  }
  i = 0;
  for (const auto &s_item : s) {
    const std::pair<Eigen::Vector3d, double> &surf = s_item.second;
    const Eigen::Vector3d &norm = std::get<0>(surf);
    msg.contacts[i].surface_normal.x = norm.x();
    msg.contacts[i].surface_normal.y = norm.y();
    msg.contacts[i].surface_normal.z = norm.z();
    msg.contacts[i].friction_coefficient = std::get<1>(surf);
    ++i;
  }
}

/**
 * @brief Conversion of a feedback gain from a ihmc_msgs::FeedbackGain
 * message to Eigen
 *
 * @param[in] msg  ROS message that contains the feedback gain
 * @param[out] K   Feedback gain (size nu * nx)
 */
static inline void fromMsg(const FeedbackGain &msg, Eigen::Ref<Eigen::MatrixXd> K) {
  if (K.rows() != msg.nu || K.cols() != msg.nx) {
    throw std::invalid_argument("The dimensions of K need to be: (" + std::to_string(msg.nu) + ", " +
                                std::to_string(msg.nx) + ").");
  }
  if (msg.data.size() != msg.nu * msg.nx) {
    throw std::invalid_argument(
        "Message incorrect - size of data does not "
        "match given dimensions (nu,nx)");
  }

  for (std::size_t i = 0; i < msg.nu; ++i) {
    for (std::size_t j = 0; j < msg.nx; ++j) {
      K(i, j) = msg.data[i * msg.nx + j];
    }
  }
}

/**
 * @brief Conversion of a state from a ihmc_msgs::State message to Eigen
 *
 * @param[in] msg  ROS message that contains the state
 * @param[out] x   State at the beginning of the interval
 * @param[out] dx  State's rate of change during the interval
 */
static inline void fromMsg(const State &msg, Eigen::Ref<Eigen::VectorXd> x, Eigen::Ref<Eigen::VectorXd> dx) {
  if (static_cast<std::size_t>(x.size()) != msg.x.size()) {
    throw std::invalid_argument("Expected x to be " + std::to_string(msg.x.size()) + " but received " +
                                std::to_string(x.size()));
  }
  if (static_cast<std::size_t>(dx.size()) != msg.dx.size()) {
    throw std::invalid_argument("Expected dx to be " + std::to_string(msg.dx.size()) + " but received " +
                                std::to_string(dx.size()));
  }
  for (std::size_t i = 0; i < msg.x.size(); ++i) {
    x(i) = msg.x[i];
  }
  for (std::size_t i = 0; i < msg.dx.size(); ++i) {
    dx(i) = msg.dx[i];
  }
}

/**
 * @brief Conversion of a control from a ihmc_msgs::Control message to
 * Eigen
 *
 * @param[in] msg               ROS message that contains the control
 * @param[out] u                Control parameters of the interval
 * @param[out] K                Feedback gain of the interval
 * @param[out] type             Control type
 * @param[out] parametrization  Control parametrization
 */
static inline void fromMsg(const Control &msg, Eigen::Ref<Eigen::VectorXd> u, Eigen::Ref<Eigen::MatrixXd> K,
                           ControlType &type, ControlParametrization &parametrization) {
  if (static_cast<std::size_t>(u.size()) != msg.u.size()) {
    throw std::invalid_argument("Expected u to be " + std::to_string(msg.u.size()) + " but received " +
                                std::to_string(u.size()));
  }
  for (std::size_t i = 0; i < msg.u.size(); ++i) {
    u(i) = msg.u[i];
  }
  fromMsg(msg.gain, K);
  type = static_cast<ControlType>(msg.input);
  parametrization = static_cast<ControlParametrization>(msg.parametrization);
}

/**
 * @brief Conversion from whole_body_state_msgs::WholeBodyState to deserialized
 * quantities
 *
 * @param model[in]  Pinocchio model
 * @param data[out]  Pinocchio data
 * @param msg[in]    ROS message that contains the whole-body state
 * @param t[out]     Time in secs
 * @param q[out]     Configuration vector (dimension: model.nq)
 * @param v[out]     Generalized velocity (dimension: model.nv)
 * @param a[out]     Generalized acceleratio (dimension: model.nv)
 * @param tau[out]   Joint effort
 * @param p[out]     Contact position
 * @param pd[out]    Contact velocity
 * @param f[out]     Contact force, type and status
 * @param s[out]     Contact surface and friction coefficient
 */
template <int Options, template <typename, int> class JointCollectionTpl>
static inline void fromMsg(const pinocchio::ModelTpl<double, Options, JointCollectionTpl> &model,
                           pinocchio::DataTpl<double, Options, JointCollectionTpl> &data, const WholeBodyState &msg,
                           double &t, Eigen::Ref<Eigen::VectorXd> q, Eigen::Ref<Eigen::VectorXd> v,
                           Eigen::Ref<Eigen::VectorXd> a, Eigen::Ref<Eigen::VectorXd> tau,
                           std::map<std::string, pinocchio::SE3> &p, std::map<std::string, pinocchio::Motion> &pd,
                           std::map<std::string, std::tuple<pinocchio::Force, ContactType, ContactStatus>> &f,
                           std::map<std::string, std::pair<Eigen::Vector3d, double>> &s) {
  if (q.size() != model.nq) {
    throw std::invalid_argument("Expected q to be " + std::to_string(model.nq) + " but received " +
                                std::to_string(q.size()));
  }
  if (v.size() != model.nv) {
    throw std::invalid_argument("Expected v to be " + std::to_string(model.nv) + " but received " +
                                std::to_string(v.size()));
  }
  if (a.size() != model.nv) {
    throw std::invalid_argument("Expected a to be " + std::to_string(model.nv) + " but received " +
                                std::to_string(v.size()));
  }
  const std::size_t nv_root = model.joints[1].idx_q() == 0 ? model.joints[1].nv() : 0;
  const std::size_t njoints = model.nv - nv_root;
  if (tau.size() != static_cast<int>(njoints)) {
    throw std::invalid_argument("Expected tau to be " + std::to_string(njoints) + " but received " +
                                std::to_string(tau.size()));
  }
  if (msg.joints.size() != static_cast<std::size_t>(njoints)) {
    throw std::invalid_argument("Message incorrect - msg.joints size is " + std::to_string(msg.joints.size()) +
                                " but expected to be " + std::to_string(njoints));
  }
  t = msg.time;
  // Retrieve the generalized position and velocity, and joint efforts
  q.head<3>().setZero();
  v.head<3>().setZero();
  if (nv_root == 6) {
    q(3) = msg.centroidal.base_orientation.x;
    q(4) = msg.centroidal.base_orientation.y;
    q(5) = msg.centroidal.base_orientation.z;
    q(6) = msg.centroidal.base_orientation.w;
    v(3) = msg.centroidal.base_angular_velocity.x;
    v(4) = msg.centroidal.base_angular_velocity.y;
    v(5) = msg.centroidal.base_angular_velocity.z;
  } else if (nv_root != 0) {
    std::cerr << "Warning: fromMsg conversion does not yet support root joints "
                 "different to a floating base. We cannot publish base information."
              << std::endl;
  }
  for (std::size_t j = 0; j < njoints; ++j) {
    auto joint_id = model.getJointId(msg.joints[j].name);
    auto q_idx = model.idx_qs[joint_id];
    auto v_idx = model.idx_vs[joint_id];
    q(q_idx) = msg.joints[j].position;
    v(v_idx) = msg.joints[j].velocity;
    a(v_idx) = msg.joints[j].acceleration;
    tau(joint_id - 2) = msg.joints[j].effort;
  }
  if (nv_root == 6) {
    pinocchio::normalize(model, q);
    pinocchio::centerOfMass(model, data, q, v, a);
    q(0) = msg.centroidal.com_position.x - data.com[0](0);
    q(1) = msg.centroidal.com_position.y - data.com[0](1);
    q(2) = msg.centroidal.com_position.z - data.com[0](2);
    v(0) = msg.centroidal.com_velocity.x - data.vcom[0](0);
    v(1) = msg.centroidal.com_velocity.y - data.vcom[0](1);
    v(2) = msg.centroidal.com_velocity.z - data.vcom[0](2);
    v.head<3>() = Eigen::Quaterniond(q(6), q(3), q(4),
                                     q(5))
                      .toRotationMatrix()
                      .transpose() *
                  v.head<3>();  // local frame
  }
  // Retrieve the contact information
  for (const auto &contact : msg.contacts) {
    // Contact pose
    p[contact.name] =
        pinocchio::SE3(Eigen::Quaterniond(contact.pose.orientation.w, contact.pose.orientation.x,
                                          contact.pose.orientation.y, contact.pose.orientation.z),
                       Eigen::Vector3d(contact.pose.position.x, contact.pose.position.y, contact.pose.position.z));
    // Contact velocity
    pd[contact.name] = pinocchio::Motion(
        Eigen::Vector3d(contact.velocity.linear.x, contact.velocity.linear.y, contact.velocity.linear.z),
        Eigen::Vector3d(contact.velocity.angular.x, contact.velocity.angular.y, contact.velocity.angular.z));
    // Contact wrench
    ContactType type;
    switch (contact.type) {
      case ContactState::LOCOMOTION:
        type = ContactType::LOCOMOTION;
        break;
      case ContactState::MANIPULATION:
        type = ContactType::MANIPULATION;
        break;
    }
    ContactStatus status;
    switch (contact.status) {
      case ContactState::UNKNOWN:
        status = ContactStatus::UNKNOWN;
        break;
      case ContactState::INACTIVE:
        status = ContactStatus::SEPARATION;
        break;
      case ContactState::ACTIVE:
        status = ContactStatus::STICKING;
        break;
      case ContactState::SLIPPING:
        status = ContactStatus::SLIPPING;
        break;
    }
    f[contact.name] = {
        pinocchio::Force(Eigen::Vector3d(contact.wrench.force.x, contact.wrench.force.y, contact.wrench.force.z),
                         Eigen::Vector3d(contact.wrench.torque.x, contact.wrench.torque.y, contact.wrench.torque.z)),
        type, status};
    // Surface normal and friction coefficient
    s[contact.name] = {Eigen::Vector3d(contact.surface_normal.x, contact.surface_normal.y, contact.surface_normal.z),
                       contact.friction_coefficient};
  }
}
// TODO(james): doc me
template <int Options, template <typename, int> class JointCollectionTpl>
static inline void fromMsg(const pinocchio::ModelTpl<double, Options, JointCollectionTpl> &model,
                           pinocchio::DataTpl<double, Options, JointCollectionTpl> &data,
                           const RobotConfigurationData &robot_configuration_data_msg,
                           const CapturabilityBasedStatus &capturability_based_status_msg,
                           double &t, Eigen::Ref<Eigen::VectorXd> q, Eigen::Ref<Eigen::VectorXd> v,
                           Eigen::Ref<Eigen::VectorXd> a, Eigen::Ref<Eigen::VectorXd> tau,
                           std::map<std::string, pinocchio::SE3> &p, std::map<std::string, pinocchio::Motion> &pd,
                           std::map<std::string, std::tuple<pinocchio::Force, ContactType, ContactStatus>> &f,
                           std::map<std::string, std::pair<Eigen::Vector3d, double>> &s) {
  if (q.size() != model.nq) {
    throw std::invalid_argument("Expected q to be " + std::to_string(model.nq) + " but received " +
        std::to_string(q.size()));
  }
  if (v.size() != model.nv) {
    throw std::invalid_argument("Expected v to be " + std::to_string(model.nv) + " but received " +
        std::to_string(v.size()));
  }
  if (a.size() != model.nv) {
    throw std::invalid_argument("Expected a to be " + std::to_string(model.nv) + " but received " +
        std::to_string(v.size()));
  }
  const std::size_t nv_root = model.joints[1].idx_q() == 0 ? model.joints[1].nv() : 0;
  const std::size_t njoints = model.nv - nv_root;
  if (tau.size() != static_cast<int>(njoints)) {
    throw std::invalid_argument("Expected tau to be " + std::to_string(njoints) + " but received " +
        std::to_string(tau.size()));
  }
  if (robot_configuration_data_msg.joint_angles.size() != static_cast<std::size_t>(njoints)) {
    throw std::invalid_argument("Message incorrect - msg.joints size is " + std::to_string(robot_configuration_data_msg.joint_angles.size()) +
        " but expected to be " + std::to_string(njoints));
  }
  t = robot_configuration_data_msg.monotonic_time;
  // Retrieve the generalized position and velocity, and joint efforts
  if (nv_root == 6) {
    q(0) = robot_configuration_data_msg.root_position.x;
    q(1) = robot_configuration_data_msg.root_position.y;
    q(2) = robot_configuration_data_msg.root_position.z;
    q(3) = robot_configuration_data_msg.root_orientation.x;
    q(4) = robot_configuration_data_msg.root_orientation.y;
    q(5) = robot_configuration_data_msg.root_orientation.z;
    q(6) = robot_configuration_data_msg.root_orientation.w;
    // TODO(james): this is relative to the pelvis orientation with respect to world
    v(0) = robot_configuration_data_msg.pelvis_linear_velocity.x;
    v(1) = robot_configuration_data_msg.pelvis_linear_velocity.y;
    v(2) = robot_configuration_data_msg.pelvis_linear_velocity.z;
    v(3) = robot_configuration_data_msg.pelvis_angular_velocity.x;
    v(4) = robot_configuration_data_msg.pelvis_angular_velocity.y;
    v(5) = robot_configuration_data_msg.pelvis_angular_velocity.z;
  } else if (nv_root != 0) {
    std::cerr << "Warning: fromMsg conversion does not yet support root joints "
                 "different to a floating base. We cannot publish base information."
              << std::endl;
  }
  for (std::size_t j = 0; j < njoints; ++j) {
    // TODO(james): this is how URDF processing can affect the message. If the "first joint" is the first joint
    //  in the URDF and so on, we're fine
    // TODO(james): figure out if it's easy or not to serialize the joint_name_hash we get in the RobotConfigurationData
    //  message
//    const int32_t hash = robot_configuration_data_msg.joint_name_hash;
//    auto joint_id = model.getJointId(msg.joints[j].name);
//    auto q_idx = model.idx_qs[joint_id];
//    auto v_idx = model.idx_vs[joint_id];
    q(7 + j) = robot_configuration_data_msg.joint_angles[j];
    v(6 + j) = robot_configuration_data_msg.joint_velocities[j];
    a(6 + j) = 0.0; // TODO(james): we don't have acceleration in our messages
    tau(j) = robot_configuration_data_msg.joint_torques[j];
  }
  // Retrieve the contact information
  // TODO(james): this is where we would have LEFT_SOLE_LINK
  const std::size_t left_sole_id = model.getFrameId("LEFT_SOLE_LINK");
  const std::size_t right_sole_id = model.getFrameId("RIGHT_SOLE_LINK");
  if (left_sole_id == model.nframes)
    throw std::invalid_argument("The LEFT_SOLE_LINK does not exist");
  if (right_sole_id == model.nframes)
    throw std::invalid_argument("The RIGHT_SOLE_LINK does not exist");
  pinocchio::forwardKinematics(model, data, q, v);
  pinocchio::updateFramePlacement(model, data, left_sole_id);
  pinocchio::updateFramePlacement(model, data, right_sole_id);

  // Contact pose
  p["LEFT_SOLE_LINK"] = data.oMf[left_sole_id];
  p["RIGHT_SOLE_LINK"] = data.oMf[right_sole_id];

  // Contact velocity
  pd["LEFT_SOLE_LINK"] = pinocchio::getFrameVelocity(model, data, left_sole_id, pinocchio::LOCAL_WORLD_ALIGNED);
  pd["RIGHT_SOLE_LINK"] = pinocchio::getFrameVelocity(model, data, right_sole_id, pinocchio::LOCAL_WORLD_ALIGNED);

  // Contact wrench TODO(james): we don't currently need to publish wrenches as they aren't used
  // Left foot status TODO(james): not considering UNKNOWN or SLIPPING statuses yet
  ContactStatus left_foot_status;
  rosidl_runtime_cpp::BoundedVector<Point, 8> left_foot_support_polygon_3d = capturability_based_status_msg.left_foot_support_polygon_3d;
  if (left_foot_support_polygon_3d.empty())
    left_foot_status = ContactStatus::SEPARATION;
  else
    left_foot_status = ContactStatus::STICKING;
  // Right foot status
  ContactStatus right_foot_status;
  rosidl_runtime_cpp::BoundedVector<Point, 8> right_foot_support_polygon_3d = capturability_based_status_msg.right_foot_support_polygon_3d;
  if (right_foot_support_polygon_3d.empty())
    right_foot_status = ContactStatus::SEPARATION;
  else
    right_foot_status = ContactStatus::STICKING;

  // TODO(james): Only considering LOCOMOTION ContactType at the moment, no manipulation
  f["LEFT_SOLE_LINK"] = {
      pinocchio::Force(Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero()),
      ContactType::LOCOMOTION,
      left_foot_status
  };
  f["RIGHT_SOLE_LINK"] = {
      pinocchio::Force(Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero()),
      ContactType::LOCOMOTION,
      right_foot_status
  };

  // Surface normals and friction coefficient
  // TODO(james): Assuming flat ground for surface normals and 1.0 for friction coefficient (these are only used for
  //  ROS-side visualisation
  s["LEFT_SOLE_LINK"] = {Eigen::Vector3d(0.0, 0.0, 1.0), 1.0};
  s["RIGHT_SOLE_LINK"] = {Eigen::Vector3d(0.0, 0.0, 1.0), 1.0};
}

}  // namespace ihmc_msgs

#endif  // CONVERSIONS_H_
