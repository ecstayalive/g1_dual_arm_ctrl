#pragma once
#include <thread>
// clang-format off
#include "droid_arm_ctrl/arm_model.h"
#include "droid_arm_ctrl/arm_api.h"
#include "droid_arm_ctrl/gazebo_arm_api.h"
#include "droid_arm_ctrl/arm_planner.h"
// clang-format on
#include <actionlib/server/simple_action_server.h>

#include "dual_arm_as/PickupAction.h"
#include "dual_arm_as/PlaceAction.h"

namespace g1_controller {

class G1Controller {
 public:
  G1Controller(const ros::NodeHandle &handle);

  ~G1Controller();

  void actionPickupAndPlaceBox();
  void actionPickupBox(const actionlib::SimpleActionServer<
                       dual_arm_as::PickupAction>::GoalConstPtr &goal);
  void actionPlaceBox(const actionlib::SimpleActionServer<
                      dual_arm_as::PlaceAction>::GoalConstPtr &goal);

  void actionLiftRightArm();

  void simplePlanAndMove(double period,
                         const Eigen::Ref<const Eigen::VectorXf> &start_q,
                         const Eigen::Ref<const Eigen::VectorXf> &end_q);

  void setSimArmApi() {
    api_ptr_ = std::make_unique<sdk::G1DualArmGazeboAPI>(handle_);
    if (communication_.joinable()) {
      communication_.join();
    }
    communication_ = std::thread([this]() {
      ros::Rate rate(100);
      while (communication_enabled_) {
        api_ptr_->setCmd(low_cmd_);
        api_ptr_->send();
        api_ptr_->recv();
        api_ptr_->getState(low_state_);
        rate.sleep();
      }
    });
  }

  void setArmApi(const std::string &name = "eth0") {
    api_ptr_ = std::make_unique<sdk::G1DualArmAPI>(name);
    if (communication_.joinable()) {
      communication_.join();
    }
    communication_ = std::thread([this]() {
      ros::Rate rate(100);
      while (communication_enabled_) {
        api_ptr_->setCmd(low_cmd_);
        api_ptr_->send();
        api_ptr_->recv();
        api_ptr_->getState(low_state_);
        rate.sleep();
      }
    });
  }

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
  // std::unique_ptr<g1_dual_arm::G1DualArmModel> arm_model_;

  // action
  actionlib::SimpleActionServer<dual_arm_as::PickupAction> pickup_as_;
  dual_arm_as::PickupResult pickup_result_;
  dual_arm_as::PickupFeedback pickup_feedback_;
  actionlib::SimpleActionServer<dual_arm_as::PlaceAction> place_as_;
  dual_arm_as::PlaceResult place_result_;
  dual_arm_as::PlaceFeedback place_feedback_;

  Eigen::VectorXf prev_goal_q_;
  bool is_init_{false};
};
}  // namespace g1_controller
