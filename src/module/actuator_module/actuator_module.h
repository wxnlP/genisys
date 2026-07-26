#pragma once

#include <mutex>
#include <semaphore>
#include <string>
#include <vector>
#include "actuator_manager.h"
#include "aimrt_module_cpp_interface/context/context.h"
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
  void WriteLoop();
  void JointCmdCallback(const std::shared_ptr<const JointCommandArray>& msg);

 private:
  aimrt::CoreRef core_;
  std::shared_ptr<aimrt::context::Context> ctx_ptr_;
  aimrt::context::res::Subscriber<JointCommandArray> sub_joint_cmd_;
  aimrt::context::res::Publisher<JointStateArray> pub_joint_state_;
  aimrt::executor::ExecutorRef pub_executor_;
  aimrt::executor::ExecutorRef write_executor_;

  std::promise<void> pub_stop_sig_;
  std::promise<void> write_stop_sig_;
  std::binary_semaphore write_sem_{0};
  std::mutex cached_cmd_lock;

  std::string topic_sub_joint_cmd_;
  std::string topic_pub_joint_state_;
  std::string pub_executor_name_;
  std::string write_executor_name_;
  double publish_frequecy_;
  std::vector<std::string> joint_names_;

  /* SDK Interfaces */
  std::shared_ptr<ActuatorManager> actuator_manager_;
  std::shared_ptr<ArmCtrlBase> arm_actuator_;
  std::shared_ptr<GripperCtrlBase> gripper_actuator_;

  ArmCtrlBase::JointCommand cached_joint_cmd_;
  GripperCtrlBase::GripperCommand cached_gripper_cmd_;
};

}  // namespace robotic_arm::actuator_module
