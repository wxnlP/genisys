#pragma once

#include <string>
#include <vector>
#include "actuator_manager.h"
#include "aimrt_module_cpp_interface/module_base.h"
#include "joint_command.pb.h"
#include "joint_state.pb.h"

using JointStateArray = aimrt::protocols::sensor::JointStateArray;
using JointCommandArray = aimrt::protocols::sensor::JointCommandArray;

namespace robotic_arm::actuator_module {

class ActuatorModule : public aimrt::ModuleBase {
 public:
  ActuatorModule() = default;
  ~ActuatorModule() override = default;

  aimrt::ModuleInfo Info() const override {
    return aimrt::ModuleInfo{.name = "ActuatorModule"};
  }

  bool Initialize(aimrt::CoreRef core) override;

  bool Start() override;

  void Shutdown() override;

 private:
  auto GetLogger() { return core_.GetLogger(); }
  void PublishLoop();
  void JointCmdCallback(const std::shared_ptr<const JointCommandArray>& msg);

 private:
  aimrt::CoreRef core_;
  aimrt::channel::SubscriberRef sub_joint_cmd_;
  aimrt::channel::PublisherRef pub_joint_state_;
  aimrt::executor::ExecutorRef pub_executor_;

  std::atomic_bool is_running_{false};
  std::atomic_bool is_shutting_down_{false};
  std::promise<void> stop_sig_;

  std::string topic_sub_joint_cmd_;
  std::string topic_pub_joint_state_;
  std::string executor_name_;
  double publish_frequecy_;
  std::vector<std::string> joint_names_;

  /* SDK Interfaces */
  std::shared_ptr<ActuatorManager> actuator_manager_;
  std::shared_ptr<ArmCtrlBase> arm_actuator_;
  std::shared_ptr<GripperCtrlBase> gripper_actuator_;
};

}  // namespace robotic_arm::actuator_module
