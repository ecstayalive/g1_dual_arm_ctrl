#include <droid_arm_ctrl/arm_api.h>

#include <chrono>
#include <cmath>
#include <thread>

int main(int argc, char **argv) {
  sdk::G1DualArmAPI arm_api;
  sdk::G1DualArmLowCmd low_cmd;
  sdk::G1DualArmLowState low_state;

  float control_dt{0.02f};
  float max_joint_vel = 0.5f;
  float max_dq = max_joint_vel * control_dt;
  std::chrono::milliseconds sleep_time =
      std::chrono::milliseconds(static_cast<int>(control_dt / 0.001f));
  Eigen::VectorXf init_pos(Eigen::VectorXf::Zero(14));
  Eigen::VectorXf target_pos(Eigen::VectorXf::Zero(14));
  target_pos << 0.f, M_PI_2f32, 0.f, M_PI_2f32, 0.f, 0.f, 0.f, 0.f, -M_PI_2f32,
      0.f, M_PI_2f32, 0.f, 0.f, 0.f;

  // Initialize arm
  Eigen::VectorXf cur_q, cmd_q(init_pos);
  arm_api.recv();
  arm_api.getState(low_state);
  low_state.getQ(cur_q);
  float period = 2.f;
  int num_time_steps = static_cast<int>(period / control_dt);
  for (int i{0}; i < num_time_steps; ++i) {
    float phase = static_cast<float>(i) / num_time_steps;
    low_cmd.setQ(cur_q * (1 - phase) + phase * init_pos);
    low_cmd.setDq(Eigen::VectorXf::Zero(14));
    low_cmd.setTau(Eigen::VectorXf::Zero(14));
    low_cmd.setControlGain(40.f, 1.0f);
    arm_api.setCmd(low_cmd);
    arm_api.send();
    arm_api.recv();
    arm_api.getState(low_state);
    std::this_thread::sleep_for(sleep_time);
  }

  // Lift arm
  period = 5.0f;
  num_time_steps = static_cast<int>(period / control_dt);
  for (int i{0}; i < num_time_steps; ++i) {
    arm_api.recv();
    arm_api.getState(low_state);
    cmd_q += (target_pos - cmd_q).cwiseMax(-max_dq).cwiseMin(max_dq);
    low_cmd.setQ(cmd_q);
    low_cmd.setDq(Eigen::VectorXf::Zero(14));
    low_cmd.setTau(Eigen::VectorXf::Zero(14));
    low_cmd.setControlGain(40.f, 1.0f);
    arm_api.setCmd(low_cmd);
    arm_api.send();
    std::this_thread::sleep_for(sleep_time);
  }

  // Lift arm
  period = 5.0f;
  num_time_steps = static_cast<int>(period / control_dt);
  for (int i{0}; i < num_time_steps; ++i) {
    arm_api.recv();
    arm_api.getState(low_state);
    cmd_q += (init_pos - cmd_q).cwiseMax(-max_dq).cwiseMin(max_dq);
    low_cmd.setQ(cmd_q);
    low_cmd.setDq(Eigen::VectorXf::Zero(14));
    low_cmd.setTau(Eigen::VectorXf::Zero(14));
    low_cmd.setControlGain(40.f, 1.f);
    arm_api.setCmd(low_cmd);
    arm_api.send();
    std::this_thread::sleep_for(sleep_time);
  }

  return 0;
}
