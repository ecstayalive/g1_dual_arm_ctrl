#include "droid_arm_ctrl/arm_control.h"

int main(int argc, char** argv) {
  ros::init(argc, argv, "g1_arm_ctrl");
  ros::NodeHandle handle;
  ros::AsyncSpinner spinner(4);
  spinner.start();
  g1_controller::G1Controller g1_arm_controller(handle);
  g1_arm_controller.setArmApi();
  // prepare
  // g1_arm_controller.actionPickupAndPlaceBox();
  ros::waitForShutdown();
  return 0;
}
