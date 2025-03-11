#include "droid_arm_controller/arm_api.h"

namespace sdk {

UnitreeG1DualArmGazeboAPI::UnitreeG1DualArmGazeboAPI(
    const ros::NodeHandle& handle)
    : handle_(handle) {
  for (unsigned int arm_id = 0; arm_id < kArmPrefix_.size(); ++arm_id) {
    std::string prefix = kArmPrefix_[arm_id];
    for (unsigned int joint_id = 0; joint_id < kArmJointNames_.size();
         ++joint_id) {
      std::string joint_name = kArmJointNames_[joint_id];
      std::string channel_name =
          "/" + kTopicPrefix_ + "/" + prefix + "_" + joint_name + "_controller";
      std::string pub_name = channel_name + "/command";
      std::string sub_name = channel_name + "/state";
      pubs_.push_back(
          handle_.advertise<unitree_legged_msgs::MotorCmd>(pub_name, 1));
      subs_.push_back(handle_.subscribe<unitree_legged_msgs::MotorState>(
          sub_name, 1,
          [this, arm_id,
           joint_id](const unitree_legged_msgs::MotorStateConstPtr& state_msg) {
            this->jointCallback(arm_id, joint_id, state_msg);
          }));
    }
  }
}

void UnitreeG1DualArmGazeboAPI::send() {
  for (unsigned int i = 0; i < kArmJointNames_.size(); ++i) {
    unitree_legged_msgs::MotorCmd left_cmd, right_cmd;
    left_cmd.mode = 0x0A;
    left_cmd.q = low_cmd_.left_arm.q[i];
    left_cmd.dq = low_cmd_.left_arm.dq[i];
    left_cmd.tau = low_cmd_.left_arm.tau[i];
    left_cmd.Kp = low_cmd_.left_arm.kp[i];
    left_cmd.Kd = low_cmd_.left_arm.kd[i];
    right_cmd.mode = 0x0A;
    right_cmd.q = low_cmd_.right_arm.q[i];
    right_cmd.dq = low_cmd_.right_arm.dq[i];
    right_cmd.tau = low_cmd_.right_arm.tau[i];
    right_cmd.Kp = low_cmd_.right_arm.kp[i];
    right_cmd.Kd = low_cmd_.right_arm.kd[i];
    pubs_[i].publish(left_cmd);
    pubs_[i + 7].publish(right_cmd);
  }
}

void UnitreeG1DualArmGazeboAPI::recv() {}

void UnitreeG1DualArmGazeboAPI::setCmd(const G1LowCmd& g1_low_cmd) {
  low_cmd_ = g1_low_cmd;
}

void UnitreeG1DualArmGazeboAPI::getState(G1LowState& g1_low_state) {
  g1_low_state = low_state_;
}

void UnitreeG1DualArmGazeboAPI::jointCallback(
    unsigned int arm_id, unsigned int joint_id,
    const unitree_legged_msgs::MotorStateConstPtr& motor_state) {
  ArmLowState* arm_state;
  switch (arm_id) {
    case 0: {  // for left arm
      arm_state = &low_state_.left_arm;
      break;
    }
    case 1: {  // for right arm
      arm_state = &low_state_.right_arm;
      break;
    }
    default:
      throw std::runtime_error("Invalid arm_id");
  }
  arm_state->q[joint_id] = motor_state->q;
  arm_state->dq[joint_id] = motor_state->dq;
  arm_state->ddq[joint_id] = motor_state->ddq;
  arm_state->tau[joint_id] = motor_state->tauEst;
  // TODO: Integrate the kinematic chain into this api
  // arm_state->end_posture.setZero();
}

}  // namespace sdk
