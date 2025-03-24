#pragma once
// clang-format off
#include <pinocchio/fwd.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/algorithm/rnea.hpp>
#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
// clang-format on
#include <ros/ros.h>

namespace g1_dual_arm {

enum class G1ArmEnum { LeftArm, RightArm };

class G1DualArmModel {
 public:
  explicit G1DualArmModel(const ros::NodeHandle &handle) : handle_(handle) {
    std::string urdf;
    handle_.getParam("robot_description", urdf);
    pinocchio::urdf::buildModelFromXML(urdf, model_);
    data_ptr_ = std::make_unique<pinocchio::Data>(model_);
    base_frame_id_ = model_.getFrameId(std::string{base_frame_name_},
                                       pinocchio::FIXED_JOINT);
    left_ee_id_ =
        model_.getFrameId(std::string{left_ee_name_}, pinocchio::FIXED_JOINT);
    right_ee_id_ =
        model_.getFrameId(std::string{right_ee_name_}, pinocchio::FIXED_JOINT);
    single_arm_dof = model_.njoints / 2;
  };

  bool inverseKinematic(G1ArmEnum arm_id, const Eigen::Isometry3d &target_pose,
                        const Eigen::Ref<const Eigen::VectorXf> &guess_q,
                        Eigen::Ref<Eigen::VectorXf> q, double timeout = 0.1) {
    pinocchio::SE3 desired_pose(target_pose.rotation(),
                                target_pose.translation());
    Eigen::VectorXf desired_q(14);
    bool success{false};
    switch (arm_id) {
      case G1ArmEnum::LeftArm:
        success = ikImpl(desired_pose, guess_q, desired_q, left_ee_id_,
                         base_frame_id_);
        q.head<7>() = desired_q.head<7>();
        break;
      case G1ArmEnum::RightArm:
        success =
            ikImpl(desired_pose, guess_q, q, right_ee_id_, base_frame_id_);
        q.tail<7>() = desired_q.tail<7>();
        break;
    }
    return success;
  }

  bool inverseKinematic(G1ArmEnum arm_id,
                        const Eigen::Ref<const Eigen::Matrix3d> &target_rot,
                        const Eigen::Ref<const Eigen::Vector3d> &target_pos,
                        const Eigen::Ref<const Eigen::VectorXf> &guess_q,
                        Eigen::Ref<Eigen::VectorXf> q, double timeout = 0.1) {
    pinocchio::SE3 target_pose(target_rot, target_pos);

    switch (arm_id) {
      case G1ArmEnum::LeftArm:
        break;
      case G1ArmEnum::RightArm:
        break;
    }
    return false;
  }

  void setEeFrameName(std::string_view left_ee_frame_name,
                      std::string_view right_ee_frame_name) {}

 protected:
  bool ikImpl(const pinocchio::SE3 &target_pose,
              const Eigen::Ref<const Eigen::VectorXf> &guess_q,
              Eigen::Ref<Eigen::VectorXf> q, const int &frame_id,
              const int &ref_id, const unsigned int &it_max = 100) {
    bool success{false};
    Eigen::VectorXd desired_q = guess_q.cast<double>();
    Eigen::VectorXd v(model_.nv);
    pinocchio::Data::Matrix6x J(6, model_.nv);
    for (int i = 0; i < it_max; ++i) {
      pinocchio::framesForwardKinematics(model_, *data_ptr_, desired_q);
      const pinocchio::SE3 oMf_e = data_ptr_->oMf[frame_id].actInv(target_pose);
      Eigen::Matrix<double, 6, 1> err = pinocchio::log6(oMf_e).toVector();
      if (err.norm() < ik_eps_) {
        success = true;
        break;
      }
      pinocchio::computeFrameJacobian(model_, *data_ptr_, desired_q, frame_id,
                                      J);  // J in local frame
      pinocchio::Data::Matrix6 Jlog;
      pinocchio::Jlog6(oMf_e.inverse(), Jlog);
      J = -Jlog * J;
      pinocchio::Data::Matrix6 JJt;
      JJt.noalias() = J * J.transpose();
      JJt.diagonal().array() += ik_damp_;
      v.noalias() = -J.transpose() * JJt.ldlt().solve(err);
      desired_q = pinocchio::integrate(model_, desired_q, v * ik_dt_);
    }
    q = desired_q.cast<float>();
    return success;
  }

 public:
  pinocchio::Model model_;
  std::unique_ptr<pinocchio::Data> data_ptr_;

  // Inverse Kinematics
  double ik_dt_{1e-1}, ik_damp_{1e-6}, ik_eps_{1e-3};
  int single_arm_dof{7};

 private:
  ros::NodeHandle handle_;
  std::string_view left_ee_name_{"left_hand_palm_joint"},
      right_ee_name_{"right_hand_palm_joint"};
  std::string_view base_frame_name_{"fixed_joint"};
  int left_ee_id_, right_ee_id_, base_frame_id_;
};

}  // namespace g1_dual_arm
