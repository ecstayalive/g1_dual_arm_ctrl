#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <unitree_legged_msgs/MotorCmd.h>
#include <unitree_legged_msgs/MotorState.h>

#include "droid_arm_controller/states_and_cmd.h"

namespace sdk {

class UnitreeG1DualArmGazeboAPI {
 public:
  UnitreeG1DualArmGazeboAPI(const ros::NodeHandle& handle);

  void send();
  void recv();
  void setCmd(const G1LowCmd& cmd);
  void getState(G1LowState& state);

 protected:
  void jointCallback(
      unsigned int arm_id, unsigned int joint_id,
      const unitree_legged_msgs::MotorStateConstPtr& motor_state);

 protected:
  ros::NodeHandle handle_;
  const std::string kTopicPrefix_{"g1_gazebo"};
  const std::vector<std::string> kArmPrefix_{"left", "right"};
  const std::vector<std::string> kArmJointNames_{
      "shoulder_pitch", "shoulder_roll", "shoulder_yaw", "elbow",
      "wrist_roll",     "wrist_pitch",   "wrist_yaw"};
  std::vector<ros::Publisher> pubs_;
  std::vector<ros::Subscriber> subs_;
  G1LowCmd low_cmd_;
  G1LowState low_state_;
};

}  // namespace sdk
