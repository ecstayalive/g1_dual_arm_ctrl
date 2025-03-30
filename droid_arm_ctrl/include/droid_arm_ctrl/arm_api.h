#pragma once
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/JointState.h>
#include <unitree_legged_msgs/MotorCmd.h>
#include <unitree_legged_msgs/MotorState.h>

#include <chrono>
#include <thread>
#include <unitree/idl/hg/LowCmd_.hpp>
#include <unitree/idl/hg/LowState_.hpp>
#include <unitree/robot/channel/channel_publisher.hpp>
#include <unitree/robot/channel/channel_subscriber.hpp>

#include "droid_arm_ctrl/states_and_cmd.h"

namespace sdk {

class DualArmAPI {
 public:
  virtual ~DualArmAPI() {};
  virtual void send() = 0;
  virtual void recv() = 0;
  void sendRecv() {
    send();
    recv();
  };
  virtual void setCmd(const G1DualArmLowCmd& cmd) = 0;
  virtual void getState(G1DualArmLowState& state) = 0;
};

class G1DualArmAPI : public DualArmAPI {
 public:
  G1DualArmAPI(const std::string& interface = "eth0") {
    unitree::robot::ChannelFactory::Instance()->Init(0, interface);
    lowcmd_publisher_.reset(
        new unitree::robot::ChannelPublisher<unitree_hg::msg::dds_::LowCmd_>(
            kArmCmdTopic_));
    lowcmd_publisher_->InitChannel();
    lowstate_subscriber_.reset(
        new unitree::robot::ChannelSubscriber<unitree_hg::msg::dds_::LowState_>(
            kStatesTopic_));
    lowstate_subscriber_->InitChannel(
        std::bind(&G1DualArmAPI::lowStateCallback, this, std::placeholders::_1),
        1);
    while (low_state_.tick() == 0) {
      std::this_thread::sleep_for(std::chrono::milliseconds(2));
    }
  }
  void send() override { lowcmd_publisher_->Write(low_cmd_); }
  void recv() override {}
  void setCmd(const G1DualArmLowCmd& cmd) override {
    low_cmd_.mode_pr() = mode_;
    low_cmd_.mode_machine() = mode_machine_;
    for (int i{0}; i < kG1ArmIdxStart_; ++i) {
      low_cmd_.motor_cmd()[i].mode(0);
      low_cmd_.motor_cmd()[i].kp(0.f);
      low_cmd_.motor_cmd()[i].kd(0.f);
      low_cmd_.motor_cmd()[i].q(low_state_.motor_state()[i].q());
      low_cmd_.motor_cmd()[i].dq(low_state_.motor_state()[i].dq());
      low_cmd_.motor_cmd()[i].tau(0.f);
    }
    for (int i{kG1ArmIdxStart_}; i < kG1ArmIdxStart_ + kG1ArmDof_; ++i) {
      low_cmd_.motor_cmd()[i].mode(1);
      low_cmd_.motor_cmd()[i + kG1ArmDof_].mode(1);
      low_cmd_.motor_cmd()[i].kp(cmd.left_arm.kp[i - kG1ArmIdxStart_]);
      low_cmd_.motor_cmd()[i + kG1ArmDof_].kp(
          cmd.right_arm.kp[i - kG1ArmIdxStart_]);
      low_cmd_.motor_cmd()[i].kd(cmd.left_arm.kd[i - kG1ArmIdxStart_]);
      low_cmd_.motor_cmd()[i + kG1ArmDof_].kd(
          cmd.right_arm.kd[i - kG1ArmIdxStart_]);
      low_cmd_.motor_cmd()[i].q(cmd.left_arm.q[i - kG1ArmIdxStart_]);
      low_cmd_.motor_cmd()[i + kG1ArmDof_].q(
          cmd.right_arm.q[i - kG1ArmIdxStart_]);
      low_cmd_.motor_cmd()[i].dq(cmd.left_arm.dq[i - kG1ArmIdxStart_]);
      low_cmd_.motor_cmd()[i].dq(cmd.right_arm.dq[i - kG1ArmIdxStart_]);
      low_cmd_.motor_cmd()[i].tau(cmd.left_arm.tau[i - kG1ArmIdxStart_]);
      low_cmd_.motor_cmd()[i + kG1ArmDof_].tau(
          cmd.right_arm.tau[i - kG1ArmIdxStart_]);
    }
    low_cmd_.crc() =
        crc32Core((uint32_t*)&low_cmd_, (sizeof(low_cmd_) >> 2) - 1);
  }
  void getState(G1DualArmLowState& state) override {
    for (int i{kG1ArmIdxStart_}; i < kG1ArmIdxStart_ + kG1ArmDof_; ++i) {
      state.left_arm.q[i - kG1ArmIdxStart_] = low_state_.motor_state()[i].q();
      state.left_arm.dq[i - kG1ArmIdxStart_] = low_state_.motor_state()[i].dq();
      state.left_arm.ddq[i - kG1ArmIdxStart_] =
          low_state_.motor_state()[i].ddq();
      state.left_arm.tau[i - kG1ArmIdxStart_] =
          low_state_.motor_state()[i].tau_est();
      state.right_arm.q[i - kG1ArmIdxStart_] =
          low_state_.motor_state()[i + kG1ArmDof_].q();
      state.right_arm.dq[i - kG1ArmIdxStart_] =
          low_state_.motor_state()[i + kG1ArmDof_].dq();
      state.right_arm.ddq[i - kG1ArmIdxStart_] =
          low_state_.motor_state()[i + kG1ArmDof_].ddq();
      state.right_arm.tau[i - kG1ArmIdxStart_] =
          low_state_.motor_state()[i + kG1ArmDof_].tau_est();
    }
    state.tick = low_state_.tick();
  }

 private:
  uint32_t crc32Core(const uint32_t* ptr, uint32_t len) {
    uint32_t xbit = 0;
    uint32_t data = 0;
    uint32_t CRC32 = 0xFFFFFFFF;
    const uint32_t dwPolynomial = 0x04c11db7;
    for (uint32_t i = 0; i < len; i++) {
      xbit = 1 << 31;
      data = ptr[i];
      for (uint32_t bits = 0; bits < 32; bits++) {
        if (CRC32 & 0x80000000) {
          CRC32 <<= 1;
          CRC32 ^= dwPolynomial;
        } else
          CRC32 <<= 1;
        if (data & xbit) CRC32 ^= dwPolynomial;
        xbit >>= 1;
      }
    }
    return CRC32;
  };

  void lowStateCallback(const void* msg) {
    const unitree_hg::msg::dds_::LowState_* low_state =
        static_cast<const unitree_hg::msg::dds_::LowState_*>(msg);
    if (low_state->crc() !=
        crc32Core((uint32_t*)low_state,
                  (sizeof(unitree_hg::msg::dds_::LowState_) >> 2) - 1)) {
      std::cout << "low_state CRC Error" << std::endl;
      return;
    }
    memcpy(&low_state_, low_state, sizeof(unitree_hg::msg::dds_::LowState_));
    if (mode_machine_ != low_state->mode_machine()) {
      if (mode_machine_ == 0)
        std::cout << "G1 type: " << unsigned(low_state->mode_machine())
                  << std::endl;
      mode_machine_ = low_state->mode_machine();
    }
  }

 private:
  unitree::robot::ChannelPublisherPtr<unitree_hg::msg::dds_::LowCmd_>
      lowcmd_publisher_;
  unitree::robot::ChannelSubscriberPtr<unitree_hg::msg::dds_::LowState_>
      lowstate_subscriber_;
  unitree_hg::msg::dds_::LowState_ low_state_;
  unitree_hg::msg::dds_::LowCmd_ low_cmd_;
  uint8_t mode_machine_{0};
  uint8_t mode_{0};  // PR:0, AB:1

  const std::string kArmCmdTopic_{"rt/lowcmd"}, kStatesTopic_{"rt/lowstate"};
  const int kG1NumMotor_{29};
  const int kG1ArmIdxStart_{15};
  const unsigned int kG1ArmDof_{7};
};

/**
 * @brief
 * @details Comm is the abbreviation for communication.
 */
class G1DualCommAPI : public DualArmAPI {
 public:
  G1DualCommAPI(const ros::NodeHandle& handle,
                const std::string& interface = "eth0")
      : handle_(handle) {
    unitree::robot::ChannelFactory::Instance()->Init(0, interface);
    lowcmd_publisher_ =
        handle_.advertise<sensor_msgs::JointState>(kCmdTopic_, 1);
    lowstate_subscriber_.reset(
        new unitree::robot::ChannelSubscriber<unitree_hg::msg::dds_::LowState_>(
            kStateTopic_));
    lowstate_subscriber_->InitChannel(
        std::bind(&G1DualCommAPI::lowStateCallback, this,
                  std::placeholders::_1),
        1);
    while (low_state_.tick() == 0) {
      std::this_thread::sleep_for(std::chrono::milliseconds(2));
    }
    // Initialize the command message
    msg_.name = kJointNames_;
    msg_.position.assign(kJointNames_.size(), 0);
    msg_.velocity.assign(kJointNames_.size(), 0);
    msg_.effort.assign(kJointNames_.size(), 0);
    for (int i{0}; i < 14; ++i) {
      msg_.position[2 * i] = low_state_.motor_state()[i + kG1ArmIdxStart_].q();
    }
  }
  void send() override { lowcmd_publisher_.publish(msg_); }
  void recv() override {}
  void setCmd(const G1DualArmLowCmd& cmd) override {
    msg_.header.stamp = ros::Time::now();
    msg_.name = kJointNames_;
    for (int i{0}; i < kG1ArmDof_; ++i) {
      msg_.position[2 * i] = cmd.left_arm.q[i];
      msg_.velocity[2 * i] = cmd.left_arm.dq[i];
      msg_.position[2 * i + 1] = cmd.left_arm.kp[i];
      msg_.velocity[2 * i + 1] = cmd.left_arm.kd[i];
      msg_.position[2 * (i + kG1ArmDof_)] = cmd.right_arm.q[i];
      msg_.velocity[2 * (i + kG1ArmDof_)] = cmd.right_arm.dq[i];
      msg_.position[2 * (i + kG1ArmDof_) + 1] = cmd.right_arm.kd[i];
      msg_.velocity[2 * (i + kG1ArmDof_) + 1] = cmd.right_arm.kd[i];
    }
  }
  void getState(G1DualArmLowState& state) override {
    for (int i{kG1ArmIdxStart_}; i < kG1ArmIdxStart_ + kG1ArmDof_; ++i) {
      state.left_arm.q[i - kG1ArmIdxStart_] = low_state_.motor_state()[i].q();
      state.left_arm.dq[i - kG1ArmIdxStart_] = low_state_.motor_state()[i].dq();
      state.left_arm.ddq[i - kG1ArmIdxStart_] =
          low_state_.motor_state()[i].ddq();
      state.left_arm.tau[i - kG1ArmIdxStart_] =
          low_state_.motor_state()[i].tau_est();
      state.right_arm.q[i - kG1ArmIdxStart_] =
          low_state_.motor_state()[i + kG1ArmDof_].q();
      state.right_arm.dq[i - kG1ArmIdxStart_] =
          low_state_.motor_state()[i + kG1ArmDof_].dq();
      state.right_arm.ddq[i - kG1ArmIdxStart_] =
          low_state_.motor_state()[i + kG1ArmDof_].ddq();
      state.right_arm.tau[i - kG1ArmIdxStart_] =
          low_state_.motor_state()[i + kG1ArmDof_].tau_est();
    }
    state.tick = low_state_.tick();
  }

 protected:
  uint32_t crc32Core(const uint32_t* ptr, uint32_t len) {
    uint32_t xbit = 0;
    uint32_t data = 0;
    uint32_t CRC32 = 0xFFFFFFFF;
    const uint32_t dwPolynomial = 0x04c11db7;
    for (uint32_t i = 0; i < len; i++) {
      xbit = 1 << 31;
      data = ptr[i];
      for (uint32_t bits = 0; bits < 32; bits++) {
        if (CRC32 & 0x80000000) {
          CRC32 <<= 1;
          CRC32 ^= dwPolynomial;
        } else
          CRC32 <<= 1;
        if (data & xbit) CRC32 ^= dwPolynomial;
        xbit >>= 1;
      }
    }
    return CRC32;
  };

  void lowStateCallback(const void* msg) {
    const unitree_hg::msg::dds_::LowState_* low_state =
        static_cast<const unitree_hg::msg::dds_::LowState_*>(msg);
    if (low_state->crc() !=
        crc32Core((uint32_t*)low_state,
                  (sizeof(unitree_hg::msg::dds_::LowState_) >> 2) - 1)) {
      std::cout << "low_state CRC Error" << std::endl;
      return;
    }
    memcpy(&low_state_, low_state, sizeof(unitree_hg::msg::dds_::LowState_));
    if (mode_machine_ != low_state->mode_machine()) {
      if (mode_machine_ == 0)
        std::cout << "G1 type: " << unsigned(low_state->mode_machine())
                  << std::endl;
      mode_machine_ = low_state->mode_machine();
    }
  }

 protected:
  ros::NodeHandle handle_;
  uint8_t mode_machine_{0};
  unitree_hg::msg::dds_::LowState_ low_state_;
  unitree::robot::ChannelSubscriberPtr<unitree_hg::msg::dds_::LowState_>
      lowstate_subscriber_;
  ros::Publisher lowcmd_publisher_;

  const std::string kCmdTopic_{"/g1_dual_arm/arm_cmd"};
  const std::string kStateTopic_{"rt/lowstate"};
  const std::vector<std::string> kJointNames_{
      "l_shoulder_pitch", "l_shoulder_pitch_gain",
      "l_shoulder_roll",  "l_shoulder_roll_gain",
      "l_shoulder_yaw",   "l_shoulder_yaw_gain",
      "l_elbow",          "l_elbow_gain",
      "l_wrist_roll",     "l_wrist_roll_gain",
      "l_wrist_pitch",    "l_wrist_pitch_gain",
      "l_wrist_yaw",      "l_wrist_yaw_gain",
      "r_shoulder_pitch", "r_shoulder_pitch_gain",
      "r_shoulder_roll",  "r_shoulder_roll_gain",
      "r_shoulder_yaw",   "r_shoulder_yaw_gain",
      "r_elbow",          "r_elbow_gain",
      "r_wrist_roll",     "r_wrist_roll_gain",
      "r_wrist_pitch",    "r_wrist_pitch_gain",
      "r_wrist_yaw",      "r_wrist_yaw_gain"};
  sensor_msgs::JointState msg_;
  const int kG1NumMotor_{29};
  const int kG1ArmIdxStart_{15};
  const unsigned int kG1ArmDof_{7};
};

}  // namespace sdk
