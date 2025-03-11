#include <algorithm>
#include <eigen3/Eigen/Core>
#include <vector>

namespace sdk {

struct ArmLowCmd {
  ArmLowCmd(unsigned int dof = 7) {
    this->dof = dof;
    q.assign(dof, 0.);
    dq.assign(dof, 0.);
    tau.assign(dof, 0.);
    kp.assign(dof, 0.);
    kd.assign(dof, 0.);
  };
  void setControlGain(double kp, double kd) {
    this->kp.assign(dof, kp);
    this->kd.assign(dof, kd);
  };
  void setControlGain(std::vector<double> kp, std::vector<double> kd) {
    assert(this->kp.size() == kp.size());
    assert(this->kd.size() == kd.size());
    this->kp = kp;
    this->kd = kd;
  };
  void setControlGain(const Eigen::Ref<const Eigen::VectorXd>& kp,
                      const Eigen::Ref<const Eigen::VectorXd>& kd) {
    assert(this->kp.size() == kp.size());
    assert(this->kd.size() == kd.size());
    std::copy(kp.data(), kp.data() + kp.size(), this->kp.data());
    std::copy(kd.data(), kd.data() + kd.size(), this->kd.data());
  }
  void setQ(const Eigen::Ref<const Eigen::VectorXd>& q) {
    assert(this->q.size() == q.size());
    std::copy(q.data(), q.data() + q.size(), this->q.data());
  };
  void setDq(const Eigen::Ref<const Eigen::VectorXd>& dq) {
    assert(this->dq.size() == dq.size());
    std::copy(dq.data(), dq.data() + dq.size(), this->dq.data());
  };
  void setTau(const Eigen::Ref<const Eigen::VectorXd>& tau) {
    assert(this->tau.size() == tau.size());
    std::copy(tau.data(), tau.data() + tau.size(), this->tau.data());
  };
  std::vector<double> q;
  std::vector<double> dq;
  std::vector<double> tau;
  std::vector<double> kp;
  std::vector<double> kd;
  size_t dof;
};

struct G1LowCmd {
  ArmLowCmd left_arm, right_arm;
};

struct ArmLowState {
  ArmLowState(unsigned int dof = 7) {
    this->dof = dof;
    q.assign(dof, 0.);
    dq.assign(dof, 0.);
    ddq.assign(dof, 0.);
    tau.assign(dof, 0.);
    end_posture.setZero();
  };
  void getQ(Eigen::Ref<Eigen::VectorXd> q) {
    q = Eigen::Map<Eigen::VectorXd>(this->q.data(), this->q.size());
  }
  [[nodiscard]] const Eigen::VectorXd getQ() {
    return Eigen::Map<Eigen::VectorXd>(this->q.data(), this->q.size());
  }
  void getDq(Eigen::Ref<Eigen::VectorXd> dq) {
    dq = Eigen::Map<Eigen::VectorXd>(this->dq.data(), this->dq.size());
  }
  [[nodiscard]] const Eigen::VectorXd getDq() {
    return Eigen::Map<Eigen::VectorXd>(this->dq.data(), this->dq.size());
  }
  void getDdq(Eigen::Ref<Eigen::VectorXd> ddq) {
    ddq = Eigen::Map<Eigen::VectorXd>(this->ddq.data(), this->ddq.size());
  }
  [[nodiscard]] const Eigen::VectorXd getDdq() {
    return Eigen::Map<Eigen::VectorXd>(this->ddq.data(), this->ddq.size());
  }
  void getTau(Eigen::Ref<Eigen::VectorXd> tau) {
    tau = Eigen::Map<Eigen::VectorXd>(this->tau.data(), this->tau.size());
  }
  [[nodiscard]] const Eigen::VectorXd getTau() {
    return Eigen::Map<Eigen::VectorXd>(this->tau.data(), this->tau.size());
  }
  std::vector<double> q;
  std::vector<double> dq;
  std::vector<double> ddq;
  std::vector<double> tau;
  Eigen::Matrix<double, 6, 1> end_posture;
  size_t dof;
};

struct G1LowState {
  ArmLowState left_arm, right_arm;
};

}  // namespace sdk
