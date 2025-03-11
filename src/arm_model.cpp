#include "droid_arm_controller/arm_model.h"

namespace g1_dual_arm {

G1DualArmModel::G1DualArmModel(const ros::NodeHandle& handle)
    : handle_(handle) {
  robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
  robot_model_ = robot_model_loader.getModel();
  robot_state_ = std::make_shared<moveit::core::RobotState>(robot_model_);
  left_arm_model_group_ = robot_model_->getJointModelGroup("g1_left_arm");
  right_arm_model_group_ = robot_model_->getJointModelGroup("g1_right_arm");
  planning_scene_ =
      std::make_shared<planning_scene::PlanningScene>(robot_model_);
  planning_scene_->getCurrentStateNonConst().setToDefaultValues(
      left_arm_model_group_, "g1_left_arm_home");
  planning_scene_->getCurrentStateNonConst().setToDefaultValues(
      right_arm_model_group_, "g1_right_arm_home");
  // Trajectory planning algorithm
  boost::scoped_ptr<pluginlib::ClassLoader<planning_interface::PlannerManager>>
      planner_plugin_loader;
  try {
    planner_plugin_loader.reset(
        new pluginlib::ClassLoader<planning_interface::PlannerManager>(
            "moveit_core", "planning_interface::PlannerManager"));
  } catch (pluginlib::PluginlibException& ex) {
    ROS_FATAL_STREAM("Exception while creating planning plugin loader "
                     << ex.what());
  }
  planner_manager_.reset(planner_plugin_loader->createUnmanagedInstance(
      "ompl_interface/OMPLPlanner"));
  if (!planner_manager_->initialize(robot_model_, handle_.getNamespace()))
    ROS_FATAL_STREAM("Could not initialize planner instance");
  ROS_INFO_STREAM("Using planning interface '"
                  << planner_manager_->getDescription() << "'");
}

bool G1DualArmModel::inverseKinematic(G1ArmEnum arm_id,
                                      const Eigen::Isometry3d& target_pose,
                                      Eigen::Ref<Eigen::VectorXd> joint_pos,
                                      double time_out) {
  bool find_ik;
  std::vector<double> joint_pos_values;
  switch (arm_id) {
    case G1ArmEnum::LeftArm: {
      find_ik =
          robot_state_->setFromIK(left_arm_model_group_, target_pose, time_out);
      robot_state_->copyJointGroupPositions(left_arm_model_group_,
                                            joint_pos_values);
      break;
    }
    case G1ArmEnum::RightArm: {
      find_ik = robot_state_->setFromIK(right_arm_model_group_, target_pose,
                                        time_out);
      robot_state_->copyJointGroupPositions(right_arm_model_group_,
                                            joint_pos_values);
      break;
    }
  }
  robot_state_->setToDefaultValues();
  if (find_ik) {
    joint_pos = Eigen::Map<Eigen::VectorXd>(joint_pos_values.data(),
                                            joint_pos_values.size());
  } else {
    joint_pos.setZero();
  }
  return find_ik;
}

void G1DualArmModel::planning(
    const Eigen::Ref<const Eigen::VectorXd>& left_arm_target_q,
    const Eigen::Ref<const Eigen::VectorXd>& right_arm_target_q) {
  const std::vector<double> left_arm_target_joint_pos(
      left_arm_target_q.data(),
      left_arm_target_q.data() + left_arm_target_q.size()),
      right_arm_target_joint_pos(
          right_arm_target_q.data(),
          right_arm_target_q.data() + right_arm_target_q.size());
  moveit::core::RobotState goal_state = planning_scene_->getCurrentState();
  goal_state.setJointGroupPositions(left_arm_model_group_,
                                    left_arm_target_joint_pos);
  goal_state.setJointGroupPositions(right_arm_model_group_,
                                    right_arm_target_joint_pos);
  planning_interface::MotionPlanRequest left_arm_req, right_arm_req;
  planning_interface::MotionPlanResponse left_arm_res, right_arm_res;
  moveit_msgs::Constraints left_arm_goal, right_arm_goal;
  planning_interface::PlanningContextPtr left_context, right_context;
  left_arm_req.group_name = "g1_left_arm";
  right_arm_req.group_name = "g1_right_arm";
  left_arm_goal = kinematic_constraints::constructGoalConstraints(
      goal_state, left_arm_model_group_);
  right_arm_goal = kinematic_constraints::constructGoalConstraints(
      goal_state, right_arm_model_group_);
  left_arm_req.goal_constraints.push_back(left_arm_goal);
  right_arm_req.goal_constraints.push_back(right_arm_goal);
  left_context = planner_manager_->getPlanningContext(
      planning_scene_, left_arm_req, left_arm_res.error_code_);
  right_context = planner_manager_->getPlanningContext(
      planning_scene_, right_arm_req, right_arm_res.error_code_);
  left_context->solve(left_arm_res);
  right_context->solve(right_arm_res);
  if (left_arm_res.error_code_.val != left_arm_res.error_code_.SUCCESS ||
      right_arm_res.error_code_.val != right_arm_res.error_code_.SUCCESS) {
    ROS_ERROR("Could not compute plan successfully");
    return;
  }
  moveit_msgs::MotionPlanResponse left_arm_planning_msg, right_arm_planning_msg;
  left_arm_res.getMessage(left_arm_planning_msg);
  right_arm_res.getMessage(right_arm_planning_msg);
  // Visualization
  moveit_msgs::DisplayTrajectory display_trajectory;
  // ros::Publisher display_publisher =
  //     handle_.advertise<moveit_msgs::DisplayTrajectory>(
  //         "/move_group/display_planned_path", 1, true);
  moveit_visual_tools::MoveItVisualTools visual_tools("world");
  visual_tools.loadRobotStatePub("/display_robot_state");
  visual_tools.enableBatchPublishing();
  visual_tools.deleteAllMarkers();  // clear all old markers
  visual_tools.trigger();
  display_trajectory.trajectory.push_back(left_arm_planning_msg.trajectory);
  display_trajectory.trajectory.push_back(right_arm_planning_msg.trajectory);
  visual_tools.publishTrajectoryLine(display_trajectory.trajectory.front(),
                                     left_arm_model_group_);
  visual_tools.publishTrajectoryLine(display_trajectory.trajectory.back(),
                                     right_arm_model_group_);
  // Update the robot states
  robot_state_->setJointGroupPositions(
      left_arm_model_group_,
      left_arm_planning_msg.trajectory.joint_trajectory.points.back()
          .positions);
  robot_state_->setJointGroupPositions(
      right_arm_model_group_,
      right_arm_planning_msg.trajectory.joint_trajectory.points.back()
          .positions);
  planning_scene_->setCurrentState(*robot_state_.get());
  // Visualization
  visual_tools.publishRobotState(planning_scene_->getCurrentStateNonConst(),
                                 rviz_visual_tools::GREEN);
  visual_tools.prompt(
      "Press 'next' in the RvizVisualToolsGui window to continue");
}

}  // namespace g1_dual_arm
