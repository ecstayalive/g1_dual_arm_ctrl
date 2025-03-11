#include <thread>

#include "droid_arm_controller/arm_api.h"
#include "droid_arm_controller/arm_model.h"

namespace g1_controller {

class G1Controller {
 public:
  G1Controller(const ros::NodeHandle &handle);

  ~G1Controller();
  void actionPickUPAndPlaceBox();
  void actionPickUpBox();
  void actionPlaceBox();

  void dualArmJointPlanAndMove(
      double period, const Eigen::Ref<const Eigen::VectorXd> &left_arm_start_q,
      const Eigen::Ref<const Eigen::VectorXd> &left_arm_end_q,
      const Eigen::Ref<const Eigen::VectorXd> &right_arm_start_q,
      const Eigen::Ref<const Eigen::VectorXd> &right_arm_end_q);

  // void dualArmPosePlanAndMove(
  //     double period,
  //     const Eigen::Ref<const Eigen::VectorXd> &left_arm_start_pose,
  //     const Eigen::Ref<const Eigen::VectorXd> &left_arm_end_pose,
  //     const Eigen::Ref<const Eigen::VectorXd> &right_arm_start_pose,
  //     const Eigen::Ref<const Eigen::VectorXd> &right_arm_end_pose);

 public:
  sdk::G1LowCmd low_cmd_;
  sdk::G1LowState low_state_;

 protected:
  ros::NodeHandle handle_;
  std::thread communication_;
  std::unique_ptr<sdk::UnitreeG1DualArmGazeboAPI> api_ptr_;
  std::unique_ptr<g1_dual_arm::G1DualArmModel> arm_model_;
};

}  // namespace g1_controller
