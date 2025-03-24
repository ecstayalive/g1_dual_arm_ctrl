#include <Eigen/Dense>
#include <cmath>

#include "droid_arm_ctrl/arm_control.h"

int main(int argc, char** argv) {
  ros::init(argc, argv, "g1_arm_controller");
  ros::NodeHandle handle;
  ros::AsyncSpinner spinner(4);
  spinner.start();
  g1_controller::G1Controller g1_arm_controller(handle);
  g1_arm_controller.setArmApi();
  // prepare
  ros::Time start_time = ros::Time::now();
  while ((ros::Time::now() - start_time).toSec() < 1.0) {
    g1_arm_controller.low_cmd_.left_arm.setControlGain(0., 2.0);
  }
  g1_arm_controller.actionLiftRightArm();
  ros::waitForShutdown();
  return 0;
}
