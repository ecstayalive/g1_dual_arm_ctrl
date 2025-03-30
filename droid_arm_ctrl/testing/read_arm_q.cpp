#include <chrono>
#include <cmath>
#include <thread>

#include "droid_arm_ctrl/arm_api.h"
#include "droid_arm_ctrl/utils.h"

class TestApi {
 public:
  TestApi() {
    api_ptr_ = std::make_unique<sdk::G1DualArmAPI>();
    if (communication_.joinable()) {
      communication_.join();
    }
    communication_ = std::thread([this]() {
      utils::Rate rate(200);
      while (communication_enabled_) {
        api_ptr_->setCmd(low_cmd_);
        api_ptr_->send();
        api_ptr_->recv();
        api_ptr_->getState(low_state_);
        rate.sleep();
      }
    });
    while (low_state_.tick == 0) {
      std::this_thread::sleep_for(std::chrono::milliseconds(2));
    }
  }
  ~TestApi() {
    communication_enabled_ = false;
    if (communication_.joinable()) {
      communication_.join();
    }
  }

 public:
  std::unique_ptr<sdk::G1DualArmAPI> api_ptr_;
  std::thread communication_;
  bool communication_enabled_{true};

  sdk::G1DualArmLowCmd low_cmd_;
  sdk::G1DualArmLowState low_state_;
};

int main(int argc, char **argv) {
  TestApi test_api;
  while (true) {
    std::cout << "joint postion: " << test_api.low_state_.getQ().transpose()
              << std::endl;
  }
}
