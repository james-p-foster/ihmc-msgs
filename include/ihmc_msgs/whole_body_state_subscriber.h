///////////////////////////////////////////////////////////////////////////////
// BSD 3-Clause License
//
// Copyright (C) 2020-2023, Heriot-Watt University, University of Oxford
// Copyright note valid unless otherwise stated in individual files.
// All rights reserved.
///////////////////////////////////////////////////////////////////////////////

#ifndef IHMC_MSG_WHOLE_BODY_STATE_SUBSCRIBER_H_
#define IHMC_MSG_WHOLE_BODY_STATE_SUBSCRIBER_H_

#include <mutex>
#include <rclcpp/rclcpp.hpp>

namespace ihmc_msgs {

typedef const whole_body_state_msgs::msg::WholeBodyState::SharedPtr WholeBodyStateSharedPtr;
typedef const controller_msgs::msg::RobotConfigurationData::SharedPtr RobotConfigurationDataSharedPtr;
typedef const controller_msgs::msg::CapturabilityBasedStatus::SharedPtr CapturabilityBasedStatusSharedPtr;

class WholeBodyStateRosSubscriber {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  // TODO(james): modify doc for the two new messages
  /**
   * @brief Initialize the whole-body state subscriber
   *
   * @param[in] model  Pinocchio model
   * @param[in] topic  Topic name
   * @param[in] frame  Odometry frame
   */
  WholeBodyStateRosSubscriber(pinocchio::Model &model,
                              const std::string &robot_configuration_data_topic = "/ihmc/nadia/humanoid_control/output/robot_configuration_data",
                              const std::string &capturability_based_status_topic = "/ihmc/nadia/humanoid_control/output/capturability_based_status",
                              const std::string &frame = "odom")
      : node_(rclcpp::Node::make_shared("whole_body_state_subscriber")),
        robot_configuration_data_sub_(node_->create_subscription<RobotConfigurationData>(
            robot_configuration_data_topic, 1, std::bind(&WholeBodyStateRosSubscriber::callback_robot_configuration_data, this, std::placeholders::_1))),
        capturability_based_status_sub_(node_->create_subscription<CapturabilityBasedStatus>(
            capturability_based_status_topic, 1, std::bind(&WholeBodyStateRosSubscriber::callback_capturability_based_status, this, std::placeholders::_1))),
        t_(0.),
        q_(model.nq),
        v_(model.nv),
        a_(model.nv),
        has_new_robot_configuration_data_msg_(false),
        has_new_capturability_based_status_msg_(false),
        is_processing_msg_(false),
        last_robot_configuration_data_msg_id_(0),
        last_capturability_based_status_msg_id_(0),
        odom_frame_(frame),
        model_(model),
        data_(model) {
    spinner_.add_node(node_);
    thread_ = std::thread([this]() { this->spin(); });
    thread_.detach();

    const std::size_t nv_root = model.joints[1].idx_q() == 0 ? model.joints[1].nv() : 0;
    const std::size_t njoints = model.nv - nv_root;
    q_.setZero();
    v_.setZero();
    a_.setZero();
    tau_ = Eigen::VectorXd::Zero(njoints);
    std::cout << "Subscribe to RobotConfigurationData messages on " << robot_configuration_data_topic << std::endl;
    std::cout << "Subscribe to CapturabilityBasedStatus messages on " << capturability_based_status_topic << std::endl;
  }
  ~WholeBodyStateRosSubscriber() = default;

  /**
   * @brief Get the latest whole-body state
   *
   * @todo: Use Pinocchio objects once there is a pybind11 support of std
   * containers
   *
   * @return  A tuple with the time at the beginning of the interval,
   * generalized position, generalized velocity, joint efforts, contact
   * position, contact velocity, contact force (wrench, type, status), and
   * contact surface (norm and friction coefficient).
   */
  std::tuple<double, Eigen::VectorXd, Eigen::VectorXd, Eigen::VectorXd,
             std::map<std::string, std::pair<Eigen::Vector3d, Eigen::MatrixXd>>,
             std::map<std::string, Eigen::VectorXd>,
             std::map<std::string, std::tuple<Eigen::VectorXd, ContactType, ContactStatus>>,
             std::map<std::string, std::pair<Eigen::Vector3d, double>>>
  get_state() {
    // start processing the message
    is_processing_msg_ = true;
    std::lock_guard<std::mutex> guard(mutex_);
    ihmc_msgs::fromMsg(model_, data_, robot_configuration_data_msg_, capturability_based_status_msg_, t_, q_, v_, a_, tau_, p_, pd_, f_, s_);
    // create maps that do not depend on Pinocchio objects
    for (auto it = p_.cbegin(); it != p_.cend(); ++it) {
      p_tmp_[it->first] = std::make_pair(it->second.translation(), it->second.rotation());
    }
    for (auto it = pd_.cbegin(); it != pd_.cend(); ++it) {
      pd_tmp_[it->first] = it->second.toVector();
    }
    for (auto it = f_.cbegin(); it != f_.cend(); ++it) {
      f_tmp_[it->first] =
          std::make_tuple(std::get<0>(it->second).toVector(), std::get<1>(it->second), std::get<2>(it->second));
    }
    // finish processing the message
    is_processing_msg_ = false;
    has_new_robot_configuration_data_msg_ = false;
    has_new_capturability_based_status_msg_ = false;
    return {t_, q_, v_, tau_, p_tmp_, pd_tmp_, f_tmp_, s_};
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

  /**
   * @brief Indicate whether we have received a new message
   */
  bool has_new_msg() const { return has_new_robot_configuration_data_msg_ && has_new_capturability_based_status_msg_; }

 private:
  std::shared_ptr<rclcpp::Node> node_;
  rclcpp::executors::SingleThreadedExecutor spinner_;
  std::thread thread_;
  void spin() { spinner_.spin(); }
  rclcpp::Subscription<RobotConfigurationData>::SharedPtr robot_configuration_data_sub_;  //!< ROS subscriber
  rclcpp::Subscription<CapturabilityBasedStatus>::SharedPtr capturability_based_status_sub_;  //!< ROS subscriber
  std::mutex mutex_;                             ///< Mutex to prevent race condition on callback
  RobotConfigurationData robot_configuration_data_msg_;                           //!< ROS message
  CapturabilityBasedStatus capturability_based_status_msg_;                           //!< ROS message
  double t_;                                     //!< Time at the beginning of the interval
  Eigen::VectorXd q_;                            ///< Configuration vector (size nq)
  Eigen::VectorXd v_;                            ///< Tangent vector (size nv)
  Eigen::VectorXd a_;                            ///< System acceleration vector (size nv)
  Eigen::VectorXd tau_;                          ///< Torque vector (size njoints)
  std::map<std::string, pinocchio::SE3> p_;      //!< Contact position
  std::map<std::string, pinocchio::Motion> pd_;  //!< Contact velocity
  std::map<std::string, std::tuple<pinocchio::Force, ContactType, ContactStatus>>
      f_;                                                        //!< Contact force, type and status
  std::map<std::string, std::pair<Eigen::Vector3d, double>> s_;  //!< Contact surface and friction coefficient
  bool has_new_robot_configuration_data_msg_;                                             //!< Indcate when a new message has been received
  bool has_new_capturability_based_status_msg_;                                             //!< Indcate when a new message has been received
  bool is_processing_msg_;                                       //!< Indicate when we are processing the message
  std::size_t last_robot_configuration_data_msg_id_;
  std::size_t last_capturability_based_status_msg_id_;
  std::string odom_frame_;
  pinocchio::Model model_;
  pinocchio::Data data_;
  // TODO(cmastalli): Temporal variables as it is needed std container support
  // in Pinocchio
  std::map<std::string, std::pair<Eigen::Vector3d, Eigen::MatrixXd>> p_tmp_;
  std::map<std::string, Eigen::VectorXd> pd_tmp_;
  std::map<std::string, std::tuple<Eigen::VectorXd, ContactType, ContactStatus>> f_tmp_;

  void callback_robot_configuration_data(RobotConfigurationDataSharedPtr msg) {
    if (!is_processing_msg_) {
      const std::size_t sequence_id = msg->sequence_id;
      // Avoid out of order arrival and ensure each message is newer (or equal
      // to) than the preceeding:
      if (last_robot_configuration_data_msg_id_ <= sequence_id) {
        std::lock_guard<std::mutex> guard(mutex_);
        robot_configuration_data_msg_ = *msg;
        has_new_robot_configuration_data_msg_ = true;
        last_robot_configuration_data_msg_id_ = sequence_id;
      } else {
        RCLCPP_WARN_STREAM(node_->get_logger(), "Out of order message. Last sequence ID: "
          << std::fixed << last_robot_configuration_data_msg_id_ << ", current sequence ID: " << sequence_id);
      }
    }
  }

  void callback_capturability_based_status(CapturabilityBasedStatusSharedPtr msg) {
    if (!is_processing_msg_) {
      const std::size_t sequence_id = msg->sequence_id;
      // Avoid out of order arrival and ensure each message is newer (or equal
      // to) than the preceeding:
      if (last_capturability_based_status_msg_id_ <= sequence_id) {
        std::lock_guard<std::mutex> guard(mutex_);
        capturability_based_status_msg_ = *msg;
        has_new_capturability_based_status_msg_ = true;
        last_capturability_based_status_msg_id_ = sequence_id;
      } else {
        RCLCPP_WARN_STREAM(node_->get_logger(), "Out of order message. Last sequence ID: "
          << std::fixed << last_capturability_based_status_msg_id_ << ", current sequence ID: " << sequence_id);
      }
    }
  }
};

}  // namespace ihmc_msgs

#endif  // IHMC_MSG_WHOLE_BODY_STATE_SUBSCRIBER_H_
