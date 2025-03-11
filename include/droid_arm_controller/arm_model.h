#include <moveit/kinematic_constraints/utils.h>
#include <moveit/planning_interface/planning_interface.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_state/robot_state.h>
#include <pluginlib/class_loader.h>
#include <ros/ros.h>

#include <boost/scoped_ptr.hpp>

// NB: remove the below sentence
#include <moveit_visual_tools/moveit_visual_tools.h>

namespace g1_dual_arm {

enum class G1ArmEnum { LeftArm, RightArm };

class G1DualArmModel {
 public:
  explicit G1DualArmModel(const ros::NodeHandle &handle);
  bool inverseKinematic(G1ArmEnum arm_id, const Eigen::Isometry3d &target_pose,
                        Eigen::Ref<Eigen::VectorXd> joint_pos,
                        double timeout = 0.1);
  void planning(const Eigen::Ref<const Eigen::VectorXd> &left_arm_target_q,
                const Eigen::Ref<const Eigen::VectorXd> &right_arm_target_q);
  // void planning(G1ArmEnum arm_id, const Eigen::Isometry3d &target_pose);

 public:
  const moveit::core::RobotModelPtr &getKinematicModel() const {
    return robot_model_;
  }
  const moveit::core::RobotStatePtr &getKinematicState() const {
    return robot_state_;
  }
  const moveit::core::JointModelGroup *getArmModelGroup(
      G1ArmEnum arm_id) const {
    switch (arm_id) {
      case (G1ArmEnum::LeftArm):
        return left_arm_model_group_;
      case (G1ArmEnum::RightArm):
        return right_arm_model_group_;
    }
  }
  const moveit::core::JointModelGroup *getLeftArmModelGroup(
      G1ArmEnum arm_id) const {
    return left_arm_model_group_;
  }
  const moveit::core::JointModelGroup *getRightArmModelGroup(
      G1ArmEnum arm_id) const {
    return right_arm_model_group_;
  }

 protected:
  ros::NodeHandle handle_;
  moveit::core::RobotModelPtr robot_model_;
  moveit::core::RobotStatePtr robot_state_;
  moveit::core::JointModelGroup *left_arm_model_group_, *right_arm_model_group_;
  planning_scene::PlanningScenePtr planning_scene_;
  planning_interface::PlannerManagerPtr planner_manager_;
};

}  // namespace g1_dual_arm
