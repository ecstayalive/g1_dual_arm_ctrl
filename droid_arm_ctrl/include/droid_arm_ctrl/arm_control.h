#pragma once
#include <thread>
// clang-format off
#include "droid_arm_ctrl/arm_model.h"
#include "droid_arm_ctrl/arm_api.h"
#include "droid_arm_ctrl/gazebo_arm_api.h"
#include "droid_arm_ctrl/arm_planner.h"
#include "droid_arm_ctrl/utils.h"
// clang-format on
#include <actionlib/server/simple_action_server.h>

#include "dual_arm_as/PickupAction.h"
#include "dual_arm_as/PlaceAction.h"

namespace g1_controller {

enum class ActionLibs { Pickup, Place, LiftRightArm };

class G1Controller {
 public:
  G1Controller(const ros::NodeHandle &handle);

  ~G1Controller();

  void setSimArmApi() {
    api_ptr_ = std::make_unique<sdk::G1DualArmGazeboAPI>(handle_);
    setCommunication();
  }

  void setArmApi(const std::string &name = "eth0") {
    api_ptr_ = std::make_unique<sdk::G1DualArmAPI>(name);
    setCommunication();
  }

  /**
   * @brief This api is used by other user to implement the integrated system.
   * @param name
   */
  void setIntCommApi(const std::string &name = "eth0") {
    api_ptr_ = std::make_unique<sdk::G1DualCommAPI>(handle_, name);
    setCommunication();
  }

  void actionPickupAndPlaceBox();
  void actionPickupBox(const actionlib::SimpleActionServer<
                       dual_arm_as::PickupAction>::GoalConstPtr &goal);
  void actionPlaceBox(const actionlib::SimpleActionServer<
                      dual_arm_as::PlaceAction>::GoalConstPtr &goal);
  void actionLiftRightArm();

  bool simplePlanAndMove(double period,
                         const Eigen::Ref<const Eigen::VectorXf> &start_q,
                         const Eigen::Ref<const Eigen::VectorXf> &end_q);
  bool checkSafety(const Eigen::Ref<const Eigen::VectorXf> &tau,
                   const double &dt) {
    int n = ((tau.cwiseAbs() - joint_tau_limit_).array() > 0.).count();
    if (n > 0) {
      max_tau_time += dt;
    } else {
      max_tau_time = std::max(0., max_tau_time - dt);
    }
    // std::cout << "max_tau_time: " << max_tau_time << std::endl;
    return max_tau_time <= kTauTimeLimit;
  }

 protected:
  void setCommunication() {
    if (communication_.joinable()) {
      communication_.join();
    }
    communication_ = std::thread([this]() {
      utils::Rate rate(200);
      while (communication_enabled_) {
        api_ptr_->setCmd(low_cmd_);
        api_ptr_->send();
        api_ptr_->recv();
        api_ptr_->getState(low_state_);
        rate.sleep();
      }
    });
    while (low_state_.tick == 0) {
      std::this_thread::sleep_for(std::chrono::milliseconds(2));
    }
    low_cmd_.setControlGain(0.f, 1.f);
    low_cmd_.setQ(low_state_.getQ());
  };

  // void dualArmPosePlanAndMove(
  //     double period,
  //     const Eigen::Ref<const Eigen::VectorXd> &left_arm_start_pose,
  //     const Eigen::Ref<const Eigen::VectorXd> &left_arm_end_pose,
  //     const Eigen::Ref<const Eigen::VectorXd> &right_arm_start_pose,
  //     const Eigen::Ref<const Eigen::VectorXd> &right_arm_end_pose);
 public:
  sdk::G1DualArmLowCmd low_cmd_;
  sdk::G1DualArmLowState low_state_;

 protected:
  ros::NodeHandle handle_;
  std::thread communication_;
  bool communication_enabled_{true};
  std::unique_ptr<sdk::DualArmAPI> api_ptr_;
  std::unique_ptr<g1_dual_arm::G1DualArmPlanner> arm_model_;
  std::mutex mtx_;
  // std::unique_ptr<g1_dual_arm::G1DualArmModel> arm_model_;

  // action
  actionlib::SimpleActionServer<dual_arm_as::PickupAction> pickup_as_;
  dual_arm_as::PickupResult pickup_result_;
  dual_arm_as::PickupFeedback pickup_feedback_;
  actionlib::SimpleActionServer<dual_arm_as::PlaceAction> place_as_;
  dual_arm_as::PlaceResult place_result_;
  dual_arm_as::PlaceFeedback place_feedback_;

  Eigen::VectorXf goal_q_hist_;
  bool is_init_{false};

  Eigen::Matrix<float, 14, 1> joint_tau_limit_;
  const double kTauTimeLimit{1.0};
  double max_tau_time{0.0};
};
}  // namespace g1_controller
