#include <Eigen/Dense>
#include <cmath>

#include "droid_arm_ctrl/arm_control.h"

int main(int argc, char** argv) {
  ros::init(argc, argv, "g1_arm_controller");
  ros::NodeHandle handle;
  ros::AsyncSpinner spinner(4);
  spinner.start();
  g1_controller::G1Controller g1_arm_controller(handle);
  g1_arm_controller.setSimArmApi();
  // prepare
  ros::Time start_time = ros::Time::now();

  // g1_arm_controller.actionPickupAndPlaceBox();
  ros::waitForShutdown();
  return 0;
}
