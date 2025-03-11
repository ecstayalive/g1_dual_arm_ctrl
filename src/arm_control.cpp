#include "droid_arm_controller/arm_control.h"

#include <Eigen/Dense>

#include "droid_arm_controller/math_utils.h"

namespace g1_controller {

G1Controller::G1Controller(const ros::NodeHandle &handle) : handle_(handle) {
  api_ptr_ = std::make_unique<sdk::UnitreeG1DualArmGazeboAPI>(handle);
  arm_model_ = std::make_unique<g1_dual_arm::G1DualArmModel>(handle);
  if (communication_.joinable()) {
    communication_.join();
  }
  communication_ = std::thread([this]() {
    ros::Rate rate(100);
    while (true) {
      api_ptr_->setCmd(low_cmd_);
      api_ptr_->send();
      api_ptr_->recv();
      api_ptr_->getState(low_state_);
      rate.sleep();
    }
  });
}

G1Controller::~G1Controller() {
  if (communication_.joinable()) {
    communication_.join();
  }
}

void G1Controller::actionPickUPAndPlaceBox() {
  Eigen::Isometry3d left_arm_target_pose = Eigen::Isometry3d::Identity(),
                    right_arm_target_pose = Eigen::Isometry3d::Identity();
  Eigen::VectorXd left_goal_q(7), right_goal_q(7);
  Eigen::VectorXd left_prev_q(7), right_prev_q(7);
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
  left_find_ik = arm_model_->inverseKinematic(
      g1_dual_arm::G1ArmEnum::LeftArm, left_arm_target_pose, left_goal_q);
  right_find_ik = arm_model_->inverseKinematic(
      g1_dual_arm::G1ArmEnum::RightArm, right_arm_target_pose, right_goal_q);
  low_state_.left_arm.getQ(left_prev_q);
  low_state_.right_arm.getQ(right_prev_q);
  dualArmJointPlanAndMove(3.0, left_prev_q, left_goal_q, right_prev_q,
                          right_goal_q);
  // stage 2
  left_arm_target_pose.setIdentity();
  right_arm_target_pose.setIdentity();
  left_arm_target_pose.rotate(
      Eigen::AngleAxisd(-M_PI / 8, Eigen::Vector3d::UnitZ()));
  right_arm_target_pose.rotate(
      Eigen::AngleAxisd(M_PI / 8, Eigen::Vector3d::UnitZ()));
  left_prev_q = left_goal_q;
  right_prev_q = right_goal_q;
  left_arm_target_pose.translation() << 0.33, 0.14, 0.69;
  right_arm_target_pose.translation() = left_arm_target_pose.translation();
  right_arm_target_pose.translation().y() =
      -right_arm_target_pose.translation().y();
  left_find_ik = arm_model_->inverseKinematic(
      g1_dual_arm::G1ArmEnum::LeftArm, left_arm_target_pose, left_goal_q);
  right_find_ik = arm_model_->inverseKinematic(
      g1_dual_arm::G1ArmEnum::RightArm, right_arm_target_pose, right_goal_q);
  dualArmJointPlanAndMove(5.4, left_prev_q, left_goal_q, right_prev_q,
                          right_goal_q);
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
  left_prev_q = left_goal_q;
  right_prev_q = right_goal_q;
  left_arm_target_pose.translation() << 0.38, 0.14, 0.9;
  right_arm_target_pose.translation() = left_arm_target_pose.translation();
  right_arm_target_pose.translation().y() =
      -right_arm_target_pose.translation().y();
  left_find_ik = arm_model_->inverseKinematic(
      g1_dual_arm::G1ArmEnum::LeftArm, left_arm_target_pose, left_goal_q);
  right_find_ik = arm_model_->inverseKinematic(
      g1_dual_arm::G1ArmEnum::RightArm, right_arm_target_pose, right_goal_q);
  dualArmJointPlanAndMove(4.2, left_prev_q, left_goal_q, right_prev_q,
                          right_goal_q);
  // stage 4
  left_arm_target_pose.setIdentity();
  right_arm_target_pose.setIdentity();
  left_prev_q = left_goal_q;
  right_prev_q = right_goal_q;
  left_arm_target_pose.translation() << 0.32, 0.15, 0.68;
  right_arm_target_pose.translation() = left_arm_target_pose.translation();
  right_arm_target_pose.translation().y() =
      -right_arm_target_pose.translation().y();
  left_find_ik = arm_model_->inverseKinematic(
      g1_dual_arm::G1ArmEnum::LeftArm, left_arm_target_pose, left_goal_q);
  right_find_ik = arm_model_->inverseKinematic(
      g1_dual_arm::G1ArmEnum::RightArm, right_arm_target_pose, right_goal_q);
  dualArmJointPlanAndMove(5.2, left_prev_q, left_goal_q, right_prev_q,
                          right_goal_q);
  // stage 2
  left_arm_target_pose.setIdentity();
  right_arm_target_pose.setIdentity();
  left_prev_q = left_goal_q;
  right_prev_q = right_goal_q;
  left_arm_target_pose.translation() << 0.12, 0.32, 0.63;
  right_arm_target_pose.translation() = left_arm_target_pose.translation();
  right_arm_target_pose.translation().y() =
      -right_arm_target_pose.translation().y();
  left_find_ik = arm_model_->inverseKinematic(
      g1_dual_arm::G1ArmEnum::LeftArm, left_arm_target_pose, left_goal_q);
  right_find_ik = arm_model_->inverseKinematic(
      g1_dual_arm::G1ArmEnum::RightArm, right_arm_target_pose, right_goal_q);
  dualArmJointPlanAndMove(2., left_prev_q, left_goal_q, right_prev_q,
                          right_goal_q);
}

void G1Controller::actionPickUpBox() {
  Eigen::Isometry3d left_arm_target_pose = Eigen::Isometry3d::Identity(),
                    right_arm_target_pose = Eigen::Isometry3d::Identity();
  Eigen::VectorXd left_goal_q(7), right_goal_q(7);
  Eigen::VectorXd left_prev_q(7), right_prev_q(7);
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
  left_find_ik = arm_model_->inverseKinematic(
      g1_dual_arm::G1ArmEnum::LeftArm, left_arm_target_pose, left_goal_q);
  right_find_ik = arm_model_->inverseKinematic(
      g1_dual_arm::G1ArmEnum::RightArm, right_arm_target_pose, right_goal_q);
  low_state_.left_arm.getQ(left_prev_q);
  low_state_.right_arm.getQ(right_prev_q);
  dualArmJointPlanAndMove(3.0, left_prev_q, left_goal_q, right_prev_q,
                          right_goal_q);
  // stage 2
  left_arm_target_pose.setIdentity();
  right_arm_target_pose.setIdentity();
  left_arm_target_pose.rotate(
      Eigen::AngleAxisd(-M_PI / 8, Eigen::Vector3d::UnitZ()));
  right_arm_target_pose.rotate(
      Eigen::AngleAxisd(M_PI / 8, Eigen::Vector3d::UnitZ()));
  left_prev_q = left_goal_q;
  right_prev_q = right_goal_q;
  left_arm_target_pose.translation() << 0.33, 0.14, 0.69;
  right_arm_target_pose.translation() = left_arm_target_pose.translation();
  right_arm_target_pose.translation().y() =
      -right_arm_target_pose.translation().y();
  left_find_ik = arm_model_->inverseKinematic(
      g1_dual_arm::G1ArmEnum::LeftArm, left_arm_target_pose, left_goal_q);
  right_find_ik = arm_model_->inverseKinematic(
      g1_dual_arm::G1ArmEnum::RightArm, right_arm_target_pose, right_goal_q);
  std::cout << "left_goal_q: " << left_goal_q.transpose() << std::endl;
  std::cout << "right_goal_q: " << right_goal_q.transpose() << std::endl;
  dualArmJointPlanAndMove(5.4, left_prev_q, left_goal_q, right_prev_q,
                          right_goal_q);
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
  left_prev_q = left_goal_q;
  right_prev_q = right_goal_q;
  left_arm_target_pose.translation() << 0.38, 0.14, 0.9;
  right_arm_target_pose.translation() = left_arm_target_pose.translation();
  right_arm_target_pose.translation().y() =
      -right_arm_target_pose.translation().y();
  left_find_ik = arm_model_->inverseKinematic(
      g1_dual_arm::G1ArmEnum::LeftArm, left_arm_target_pose, left_goal_q);
  right_find_ik = arm_model_->inverseKinematic(
      g1_dual_arm::G1ArmEnum::RightArm, right_arm_target_pose, right_goal_q);
  dualArmJointPlanAndMove(4.2, left_prev_q, left_goal_q, right_prev_q,
                          right_goal_q);
}

void G1Controller::actionPlaceBox() {
  // stage 1
  Eigen::Isometry3d left_arm_target_pose = Eigen::Isometry3d::Identity(),
                    right_arm_target_pose = Eigen::Isometry3d::Identity();
  Eigen::VectorXd left_goal_q(7), right_goal_q(7);
  Eigen::VectorXd left_prev_q(7), right_prev_q(7);
  bool left_find_ik, right_find_ik;
  low_state_.left_arm.getQ(left_prev_q);
  low_state_.right_arm.getQ(right_prev_q);
  left_arm_target_pose.translation() << 0.32, 0.15, 0.68;
  right_arm_target_pose.translation() = left_arm_target_pose.translation();
  right_arm_target_pose.translation().y() =
      -right_arm_target_pose.translation().y();
  left_find_ik = arm_model_->inverseKinematic(
      g1_dual_arm::G1ArmEnum::LeftArm, left_arm_target_pose, left_goal_q);
  right_find_ik = arm_model_->inverseKinematic(
      g1_dual_arm::G1ArmEnum::RightArm, right_arm_target_pose, right_goal_q);
  dualArmJointPlanAndMove(5.2, left_prev_q, left_goal_q, right_prev_q,
                          right_goal_q);
  // stage 2
  left_arm_target_pose.setIdentity();
  right_arm_target_pose.setIdentity();
  left_prev_q = left_goal_q;
  right_prev_q = right_goal_q;
  left_arm_target_pose.translation() << 0.12, 0.32, 0.63;
  right_arm_target_pose.translation() = left_arm_target_pose.translation();
  right_arm_target_pose.translation().y() =
      -right_arm_target_pose.translation().y();
  left_find_ik = arm_model_->inverseKinematic(
      g1_dual_arm::G1ArmEnum::LeftArm, left_arm_target_pose, left_goal_q);
  right_find_ik = arm_model_->inverseKinematic(
      g1_dual_arm::G1ArmEnum::RightArm, right_arm_target_pose, right_goal_q);
  dualArmJointPlanAndMove(2., left_prev_q, left_goal_q, right_prev_q,
                          right_goal_q);
}

void G1Controller::dualArmJointPlanAndMove(
    double period, const Eigen::Ref<const Eigen::VectorXd> &left_arm_start_q,
    const Eigen::Ref<const Eigen::VectorXd> &left_arm_end_q,
    const Eigen::Ref<const Eigen::VectorXd> &right_arm_start_q,
    const Eigen::Ref<const Eigen::VectorXd> &right_arm_end_q) {
  math_utils::QuinticInterpolationFn<Eigen::VectorXd> left_interpolation_fn,
      right_interpolation_fn;
  left_interpolation_fn.setPolyInterpolationKernel(period, left_arm_start_q,
                                                   left_arm_end_q);
  right_interpolation_fn.setPolyInterpolationKernel(period, right_arm_start_q,
                                                    right_arm_end_q);
  ros::Time start_time = ros::Time::now();
  while ((ros::Time::now() - start_time).toSec() < period) {
    low_cmd_.left_arm.setControlGain(100, 0.8);
    low_cmd_.right_arm.setControlGain(100, 0.8);
    low_cmd_.left_arm.setQ(
        left_interpolation_fn((ros::Time::now() - start_time).toSec()));
    low_cmd_.right_arm.setQ(
        right_interpolation_fn((ros::Time::now() - start_time).toSec()));
  }
}

}  // namespace g1_controller
