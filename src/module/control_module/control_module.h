#pragma once

#include "aimrt_module_cpp_interface/module_base.h"
#include "arm_manager.h"
#include "joint_command.pb.h"
#include "joint_state.pb.h"
#include "service.h"
#include <vector>
#include <string>

using JointStateArray = aimrt::protocols::sensor::JointStateArray;
using JointCommandArray = aimrt::protocols::sensor::JointCommandArray;

namespace robotic_arm::control_module {

class ControlModule : public aimrt::ModuleBase {
 public:
  ControlModule() = default;
  ~ControlModule() override = default;

  aimrt::ModuleInfo Info() const override {
    return aimrt::ModuleInfo{.name = "ControlModule"};
  }

  bool Initialize(aimrt::CoreRef core) override;

  bool Start() override;

  void Shutdown() override;

 private:
  auto GetLogger() { return core_.GetLogger(); }
  void ControlLoop();
  void JointStateCallback(const std::shared_ptr<const JointStateArray>& msg);

 private:
  aimrt::CoreRef core_;
  aimrt::channel::PublisherRef pub_joint_cmd_;
  aimrt::channel::SubscriberRef sub_joint_state_;
  // 周期控制执行器（单独）
  aimrt::executor::ExecutorRef cycle_control_executor_;

  std::atomic_bool is_running_{false};
  std::promise<void> stop_sig_;

  std::string topic_pub_joint_cmd_;
  std::string topic_sub_joint_state_;
  std::string control_executor_name_;
  double control_frequency_;
  std::vector<std::string> joint_names_;

  std::shared_ptr<ArmManager> arm_manager_;
  std::shared_ptr<ArmController> arm_controller_;
  std::shared_ptr<GripperController> gripper_controller_;

  std::shared_ptr<MoveControlServiceImpl> move_control_service_;

  // 记录上一帧有效指令，用于轨迹间保持
  ArmMotion::JointCommand last_arm_cmd_;
  GripperController::GripperCommand last_gripper_cmd_;
};

}  // namespace robotic_arm::control_module
