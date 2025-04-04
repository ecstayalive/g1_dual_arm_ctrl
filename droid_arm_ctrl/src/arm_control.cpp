#include "droid_arm_ctrl/arm_control.h"

#include "droid_arm_ctrl/math_utils.h"

namespace g1_controller {

G1Controller::G1Controller(const ros::NodeHandle &handle)
    : handle_(handle),
      pickup_as_(handle, "/dual_arm_ctrl/pickup",
                 std::bind(&G1Controller::actionPickupBox, this,
                           std::placeholders::_1),
                 false),
      place_as_(
          handle, "/dual_arm_ctrl/place",
          std::bind(&G1Controller::actionPlaceBox, this, std::placeholders::_1),
          false) {
  arm_model_ = std::make_unique<g1_dual_arm::G1DualArmPlanner>(handle);
  pickup_as_.start();
  place_as_.start();
  goal_q_hist_.setZero(14);
  // clang-format off
  joint_tau_limit_ << 21, 21, 21, 21, 21, 4.0, 4.0,
                      21, 21, 21, 21, 21, 4.0, 4.0;
  // clang-format on
  //   arm_model_ = std::make_unique<g1_dual_arm::G1DualArmModel>(handle);
}

G1Controller::~G1Controller() {
  low_cmd_.setControlGain(0.f, 2.f);
  communication_enabled_ = false;
  if (communication_.joinable()) {
    communication_.join();
  }
}

void G1Controller::actionPickupAndPlaceBox() {
  std::lock_guard<std::mutex> lock(mtx_);
  Eigen::Isometry3d left_arm_target_pose = Eigen::Isometry3d::Identity(),
                    right_arm_target_pose = Eigen::Isometry3d::Identity();
  Eigen::VectorXf goal_q(14), prev_q(14);
  bool left_find_ik, right_find_ik;
  // stage 1
  left_arm_target_pose.rotate(
      Eigen::AngleAxisd(M_PI / 6, Eigen::Vector3d::UnitY()));
  right_arm_target_pose.rotate(
      Eigen::AngleAxisd(M_PI / 6, Eigen::Vector3d::UnitY()));
  left_arm_target_pose.translation() << 0.05, 0.3, 0.8;
  right_arm_target_pose.translation() = left_arm_target_pose.translation();
  right_arm_target_pose.translation().y() =
      -right_arm_target_pose.translation().y();
  left_find_ik = arm_model_->inverseKinematic(g1_dual_arm::G1ArmEnum::LeftArm,
                                              left_arm_target_pose,
                                              low_state_.getQ(), goal_q);
  right_find_ik = arm_model_->inverseKinematic(g1_dual_arm::G1ArmEnum::RightArm,
                                               right_arm_target_pose,
                                               low_state_.getQ(), goal_q);
  std::cout << "left_find_ik:" << left_find_ik
            << "\nright_find_ik: " << right_find_ik
            << "\ngoal_q: " << goal_q.transpose() << std::endl;
  if (is_init_) {
    prev_q = goal_q_hist_;
  } else {
    low_state_.getQ(prev_q);
    is_init_ = true;
  }
  if (left_find_ik && right_find_ik &&
      !simplePlanAndMove(3.0, prev_q, goal_q)) {
    return;
  }
  // stage 2
  left_arm_target_pose.setIdentity();
  right_arm_target_pose.setIdentity();
  left_arm_target_pose.rotate(
      Eigen::AngleAxisd(-M_PI / 6, Eigen::Vector3d::UnitZ()));
  right_arm_target_pose.rotate(
      Eigen::AngleAxisd(M_PI / 6, Eigen::Vector3d::UnitZ()));
  prev_q = goal_q;
  left_arm_target_pose.translation() << 0.33, 0.15, 0.69;
  right_arm_target_pose.translation() = left_arm_target_pose.translation();
  right_arm_target_pose.translation().y() =
      -right_arm_target_pose.translation().y();
  left_find_ik = arm_model_->inverseKinematic(g1_dual_arm::G1ArmEnum::LeftArm,
                                              left_arm_target_pose,
                                              low_state_.getQ(), goal_q);
  right_find_ik = arm_model_->inverseKinematic(g1_dual_arm::G1ArmEnum::RightArm,
                                               right_arm_target_pose,
                                               low_state_.getQ(), goal_q);
  std::cout << "left_find_ik:" << left_find_ik
            << "\nright_find_ik: " << right_find_ik
            << "\ngoal_q: " << goal_q.transpose() << std::endl;
  if (left_find_ik && right_find_ik &&
      !simplePlanAndMove(5.4, prev_q, goal_q)) {
    return;
  }
  // stage 3
  left_arm_target_pose.setIdentity();
  right_arm_target_pose.setIdentity();
  left_arm_target_pose.rotate(
      Eigen::AngleAxisd(-M_PI / 6, Eigen::Vector3d::UnitY()));
  right_arm_target_pose.rotate(
      Eigen::AngleAxisd(-M_PI / 6, Eigen::Vector3d::UnitY()));
  left_arm_target_pose.rotate(
      Eigen::AngleAxisd(-M_PI / 6, Eigen::Vector3d::UnitZ()));
  right_arm_target_pose.rotate(
      Eigen::AngleAxisd(M_PI / 6, Eigen::Vector3d::UnitZ()));
  prev_q = goal_q;
  left_arm_target_pose.translation() << 0.38, 0.15, 0.9;
  right_arm_target_pose.translation() = left_arm_target_pose.translation();
  right_arm_target_pose.translation().y() =
      -right_arm_target_pose.translation().y();
  left_find_ik = arm_model_->inverseKinematic(g1_dual_arm::G1ArmEnum::LeftArm,
                                              left_arm_target_pose,
                                              low_state_.getQ(), goal_q);
  right_find_ik = arm_model_->inverseKinematic(g1_dual_arm::G1ArmEnum::RightArm,
                                               right_arm_target_pose,
                                               low_state_.getQ(), goal_q);
  std::cout << "left_find_ik:" << left_find_ik
            << "\nright_find_ik: " << right_find_ik
            << "\ngoal_q: " << goal_q.transpose() << std::endl;
  if (left_find_ik && right_find_ik &&
      !simplePlanAndMove(4.2, prev_q, goal_q)) {
    return;
  }
  // stage 4
  left_arm_target_pose.setIdentity();
  right_arm_target_pose.setIdentity();
  prev_q = goal_q;
  left_arm_target_pose.rotate(
      Eigen::AngleAxisd(-M_PI / 6, Eigen::Vector3d::UnitZ()));
  right_arm_target_pose.rotate(
      Eigen::AngleAxisd(M_PI / 6, Eigen::Vector3d::UnitZ()));
  left_arm_target_pose.translation() << 0.32, 0.15, 0.68;
  right_arm_target_pose.translation() = left_arm_target_pose.translation();
  right_arm_target_pose.translation().y() =
      -right_arm_target_pose.translation().y();
  left_find_ik = arm_model_->inverseKinematic(g1_dual_arm::G1ArmEnum::LeftArm,
                                              left_arm_target_pose,
                                              low_state_.getQ(), goal_q);
  right_find_ik = arm_model_->inverseKinematic(g1_dual_arm::G1ArmEnum::RightArm,
                                               right_arm_target_pose,
                                               low_state_.getQ(), goal_q);
  std::cout << "left_find_ik:" << left_find_ik
            << "\nright_find_ik: " << right_find_ik
            << "\ngoal_q: " << goal_q.transpose() << std::endl;
  if (left_find_ik && right_find_ik &&
      !simplePlanAndMove(5.2, prev_q, goal_q)) {
    return;
  }
  // stage 5
  left_arm_target_pose.setIdentity();
  right_arm_target_pose.setIdentity();
  prev_q = goal_q;
  left_arm_target_pose.translation() << 0.12, 0.32, 0.63;
  right_arm_target_pose.translation() = left_arm_target_pose.translation();
  right_arm_target_pose.translation().y() =
      -right_arm_target_pose.translation().y();
  left_find_ik = arm_model_->inverseKinematic(g1_dual_arm::G1ArmEnum::LeftArm,
                                              left_arm_target_pose,
                                              low_state_.getQ(), goal_q);
  right_find_ik = arm_model_->inverseKinematic(g1_dual_arm::G1ArmEnum::RightArm,
                                               right_arm_target_pose,
                                               low_state_.getQ(), goal_q);
  std::cout << "left_find_ik:" << left_find_ik
            << "\nright_find_ik: " << right_find_ik
            << "\ngoal_q: " << goal_q.transpose() << std::endl;
  if (left_find_ik && right_find_ik && !simplePlanAndMove(2., prev_q, goal_q)) {
    return;
  }
}

void G1Controller::actionPickupBox(
    const actionlib::SimpleActionServer<dual_arm_as::PickupAction>::GoalConstPtr
        &goal) {
  std::lock_guard<std::mutex> lock(mtx_);
  if (!goal->pickup) {
    pickup_result_.success = false;
    pickup_as_.setAborted(pickup_result_, "Pickup action not performed");
    return;
  }
  pickup_feedback_.progress = 0.f;
  pickup_as_.publishFeedback(pickup_feedback_);
  Eigen::Isometry3d left_arm_target_pose = Eigen::Isometry3d::Identity(),
                    right_arm_target_pose = Eigen::Isometry3d::Identity();
  Eigen::VectorXf goal_q(14), prev_q(14);
  bool left_find_ik, right_find_ik;
  // stage 1
  left_arm_target_pose.rotate(
      Eigen::AngleAxisd(M_PI / 6, Eigen::Vector3d::UnitY()));
  right_arm_target_pose.rotate(
      Eigen::AngleAxisd(M_PI / 6, Eigen::Vector3d::UnitY()));
  left_arm_target_pose.translation() << 0.05, 0.3, 0.8;
  right_arm_target_pose.translation() = left_arm_target_pose.translation();
  right_arm_target_pose.translation().y() =
      -right_arm_target_pose.translation().y();
  left_find_ik = arm_model_->inverseKinematic(g1_dual_arm::G1ArmEnum::LeftArm,
                                              left_arm_target_pose,
                                              low_state_.getQ(), goal_q);
  right_find_ik = arm_model_->inverseKinematic(g1_dual_arm::G1ArmEnum::RightArm,
                                               right_arm_target_pose,
                                               low_state_.getQ(), goal_q);
  std::cout << "find_ik: " << (left_find_ik && right_find_ik)
            << "\ngoal q: " << goal_q.transpose() << std::endl;
  if (is_init_) {
    prev_q = goal_q_hist_;
  } else {
    low_state_.getQ(prev_q);
    is_init_ = true;
  }
  if (left_find_ik && right_find_ik && !simplePlanAndMove(3., prev_q, goal_q)) {
    pickup_result_.success = false;
    pickup_as_.setAborted(
        pickup_result_, "Pickup action failed due to dangerous joint effector");
    return;
  }
  pickup_feedback_.progress = 0.33f;
  pickup_as_.publishFeedback(pickup_feedback_);
  // stage 2
  left_arm_target_pose.setIdentity();
  right_arm_target_pose.setIdentity();
  left_arm_target_pose.rotate(
      Eigen::AngleAxisd(-M_PI / 8, Eigen::Vector3d::UnitZ()));
  right_arm_target_pose.rotate(
      Eigen::AngleAxisd(M_PI / 8, Eigen::Vector3d::UnitZ()));
  prev_q = goal_q;
  left_arm_target_pose.translation() << 0.27, 0.15, 0.69;
  right_arm_target_pose.translation() = left_arm_target_pose.translation();
  right_arm_target_pose.translation().y() =
      -right_arm_target_pose.translation().y();
  left_find_ik = arm_model_->inverseKinematic(g1_dual_arm::G1ArmEnum::LeftArm,
                                              left_arm_target_pose,
                                              low_state_.getQ(), goal_q);
  right_find_ik = arm_model_->inverseKinematic(g1_dual_arm::G1ArmEnum::RightArm,
                                               right_arm_target_pose,
                                               low_state_.getQ(), goal_q);
  std::cout << "find_ik: " << (left_find_ik && right_find_ik)
            << "\ngoal q: " << goal_q.transpose() << std::endl;
  if (left_find_ik && right_find_ik &&
      !simplePlanAndMove(5.4, prev_q, goal_q)) {
    pickup_result_.success = false;
    pickup_as_.setAborted(
        pickup_result_, "Pickup action failed due to dangerous joint effector");
    return;
  }
  pickup_feedback_.progress = 0.66f;
  pickup_as_.publishFeedback(pickup_feedback_);
  // stage 3
  left_arm_target_pose.setIdentity();
  right_arm_target_pose.setIdentity();
  left_arm_target_pose.rotate(
      Eigen::AngleAxisd(-M_PI / 6, Eigen::Vector3d::UnitY()));
  right_arm_target_pose.rotate(
      Eigen::AngleAxisd(-M_PI / 6, Eigen::Vector3d::UnitY()));
  left_arm_target_pose.rotate(
      Eigen::AngleAxisd(-M_PI / 8, Eigen::Vector3d::UnitZ()));
  right_arm_target_pose.rotate(
      Eigen::AngleAxisd(M_PI / 8, Eigen::Vector3d::UnitZ()));
  prev_q = goal_q;
  left_arm_target_pose.translation() << 0.27, 0.15, 0.9;
  right_arm_target_pose.translation() = left_arm_target_pose.translation();
  right_arm_target_pose.translation().y() =
      -right_arm_target_pose.translation().y();
  left_find_ik = arm_model_->inverseKinematic(g1_dual_arm::G1ArmEnum::LeftArm,
                                              left_arm_target_pose,
                                              low_state_.getQ(), goal_q);
  right_find_ik = arm_model_->inverseKinematic(g1_dual_arm::G1ArmEnum::RightArm,
                                               right_arm_target_pose,
                                               low_state_.getQ(), goal_q);
  std::cout << "find_ik: " << (left_find_ik && right_find_ik)
            << "\ngoal q: " << goal_q.transpose() << std::endl;
  if (left_find_ik && right_find_ik &&
      !simplePlanAndMove(4.2, prev_q, goal_q)) {
    pickup_result_.success = false;
    pickup_as_.setAborted(
        pickup_result_, "Pickup action failed due to dangerous joint effector");
    return;
  }
  pickup_feedback_.progress = 1.f;
  pickup_as_.publishFeedback(pickup_feedback_);
  pickup_result_.success = true;
  pickup_as_.setSucceeded(pickup_result_);
}

void G1Controller::actionPlaceBox(
    const actionlib::SimpleActionServer<dual_arm_as::PlaceAction>::GoalConstPtr
        &goal) {
  std::lock_guard<std::mutex> lock(mtx_);
  if (!goal->place) {
    place_result_.success = false;
    place_as_.setSucceeded(place_result_, "Place action not performed");
    return;
  }
  place_feedback_.progress = 0.f;
  Eigen::Isometry3d left_arm_target_pose = Eigen::Isometry3d::Identity(),
                    right_arm_target_pose = Eigen::Isometry3d::Identity();
  Eigen::VectorXf goal_q(14), prev_q(14);
  bool left_find_ik, right_find_ik;
  // stage 1
  left_arm_target_pose.setIdentity();
  right_arm_target_pose.setIdentity();
  left_arm_target_pose.rotate(
      Eigen::AngleAxisd(-M_PI / 8, Eigen::Vector3d::UnitZ()));
  right_arm_target_pose.rotate(
      Eigen::AngleAxisd(M_PI / 8, Eigen::Vector3d::UnitZ()));
  left_arm_target_pose.translation() << 0.27, 0.15, 0.68;
  right_arm_target_pose.translation() = left_arm_target_pose.translation();
  right_arm_target_pose.translation().y() =
      -right_arm_target_pose.translation().y();
  left_find_ik = arm_model_->inverseKinematic(g1_dual_arm::G1ArmEnum::LeftArm,
                                              left_arm_target_pose,
                                              low_state_.getQ(), goal_q);
  right_find_ik = arm_model_->inverseKinematic(g1_dual_arm::G1ArmEnum::RightArm,
                                               right_arm_target_pose,
                                               low_state_.getQ(), goal_q);
  std::cout << "find_ik: " << (left_find_ik && right_find_ik)
            << "\ngoal q: " << goal_q.transpose() << std::endl;
  if (is_init_) {
    prev_q = goal_q_hist_;
  } else {
    low_state_.getQ(prev_q);
    is_init_ = true;
  }
  if (left_find_ik && right_find_ik &&
      !simplePlanAndMove(5.2, prev_q, goal_q)) {
    place_result_.success = false;
    place_as_.setAborted(
        place_result_, "Pickup action failed due to dangerous joint effector");
    return;
  }
  place_feedback_.progress = 0.5f;
  place_as_.publishFeedback(place_feedback_);
  // stage 2
  left_arm_target_pose.setIdentity();
  right_arm_target_pose.setIdentity();
  prev_q = goal_q;
  left_arm_target_pose.translation() << 0.10, 0.32, 0.63;
  right_arm_target_pose.translation() = left_arm_target_pose.translation();
  right_arm_target_pose.translation().y() =
      -right_arm_target_pose.translation().y();
  left_find_ik = arm_model_->inverseKinematic(g1_dual_arm::G1ArmEnum::LeftArm,
                                              left_arm_target_pose,
                                              low_state_.getQ(), goal_q);
  right_find_ik = arm_model_->inverseKinematic(g1_dual_arm::G1ArmEnum::RightArm,
                                               right_arm_target_pose,
                                               low_state_.getQ(), goal_q);
  std::cout << "find_ik: " << (left_find_ik && right_find_ik)
            << "\ngoal q: " << goal_q.transpose() << std::endl;
  if (left_find_ik && right_find_ik && !simplePlanAndMove(2., prev_q, goal_q)) {
    place_result_.success = false;
    place_as_.setAborted(
        place_result_, "Pickup action failed due to dangerous joint effector");
    return;
  }
  place_feedback_.progress = 1.f;
  place_as_.publishFeedback(place_feedback_);
  place_result_.success = true;
  place_as_.setSucceeded(place_result_);
}

void G1Controller::actionLiftRightArm() {
  std::lock_guard<std::mutex> lock(mtx_);
  Eigen::Isometry3d right_arm_target_pose = Eigen::Isometry3d::Identity();
  Eigen::VectorXf goal_q(14), prev_q(14);
  low_state_.getQ(prev_q);
  low_state_.getQ(goal_q);
  bool left_find_ik;
  // stage 1
  right_arm_target_pose.rotate(Eigen::AngleAxisd(0, Eigen::Vector3d::UnitY()));
  right_arm_target_pose.translation() << 0.25, -0.14, 0.9;
  left_find_ik = arm_model_->inverseKinematic(g1_dual_arm::G1ArmEnum::RightArm,
                                              right_arm_target_pose,
                                              low_state_.getQ(), goal_q);
  if (is_init_) {
    prev_q = goal_q_hist_;
  } else {
    low_state_.getQ(prev_q);
    is_init_ = true;
  }
  std::cout << "find_ik:" << left_find_ik << "\ngoal_q: " << goal_q.transpose()
            << std::endl;
  if (left_find_ik && !simplePlanAndMove(4.0, prev_q, goal_q)) {
    return;
  }
  // stage 2
  prev_q = goal_q;
  right_arm_target_pose.setIdentity();
  right_arm_target_pose.rotate(
      Eigen::AngleAxisd(M_PI / 8, Eigen::Vector3d::UnitY()));
  right_arm_target_pose.translation() << 0.4, -0.14, 0.82;
  left_find_ik = arm_model_->inverseKinematic(g1_dual_arm::G1ArmEnum::RightArm,
                                              right_arm_target_pose,
                                              low_state_.getQ(), goal_q);
  std::cout << "find_ik:" << left_find_ik << "\ngoal_q: " << goal_q.transpose()
            << std::endl;
  if (left_find_ik && !simplePlanAndMove(4.0, prev_q, goal_q)) {
    return;
  }
}

bool G1Controller::simplePlanAndMove(
    double period, const Eigen::Ref<const Eigen::VectorXf> &start_q,
    const Eigen::Ref<const Eigen::VectorXf> &end_q) {
  math_utils::QuinticInterpolationFn<Eigen::VectorXf> interpolation_fn;
  interpolation_fn.setPolyInterpolationKernel(period, start_q, end_q);
  ros::Time start_time = ros::Time::now();
  ros::Time prev_time = start_time;
  // Get theoretical torque
  while ((ros::Time::now() - start_time).toSec() < period && ros::ok()) {
    double t = (ros::Time::now() - start_time).toSec();
    double dt = (ros::Time::now() - prev_time).toSec();
    prev_time = ros::Time::now();
    Eigen::VectorXf cmd_q = interpolation_fn(t);
    // std::cout << "cmd_q: " << cmd_q.transpose() << std::endl;
    if (arm_model_->constrainJointPos(cmd_q)) {
      ROS_WARN("The command joint q is out of limit!!");
    }
    if (!checkSafety(low_state_.getTau(), dt)) {
      low_cmd_.setControlGain(0.f, 2.f);
      return false;
    };
    low_cmd_.setControlGain(40.f, 1.f);
    Eigen::VectorXf desired_tau =
        low_cmd_.getControlGainKp().cwiseProduct(cmd_q - low_state_.getQ()) -
        low_cmd_.getControlGainKd().cwiseProduct(low_state_.getDq());
    Eigen::VectorXf real_tau = low_state_.getTau();
    // if (desired_tau.cwiseAbs().mean() > 20.0) {
    //   break;
    // } else {
    // std::cout << "desired_tau: " << desired_tau.transpose()
    //           << "\nreal_tau: " << real_tau.transpose()
    //           << "\ncmd_q: " << cmd_q.transpose()
    //           << "\nreal_q: " << low_state_.getQ().transpose()
    //           << "\nreal_dq: " << low_state_.getDq().transpose() <<
    //           std::endl;
    low_cmd_.setQ(cmd_q);
    // }
  }
  goal_q_hist_ = end_q;
  return true;
}

}  // namespace g1_controller
