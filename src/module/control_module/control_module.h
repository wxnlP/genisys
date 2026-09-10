#pragma once

#include <atomic>
#include <cstdint>
#include <mutex>
#include <string>
#include <thread>
#include <vector>
#include "aimrt_module_cpp_interface/context/context.h"
#include "aimrt_module_cpp_interface/module_base.h"
#include "arm_manager.h"
#include "joint_command.pb.h"
#include "joint_state.pb.h"
#include "move_control.aimrt_rpc.pb.h"
#include "servo.pb.h"

using JointStateArray = aimrt::protocols::sensor::JointStateArray;
using JointCommandArray = aimrt::protocols::sensor::JointCommandArray;
using Pose = robotic_arm::protocols::PoseStamped;
using ServoJCommand = robotic_arm::protocols::ServoJCommand;
using ServoJPoseCommand = robotic_arm::protocols::ServoJPoseCommand;
using ServoLCommand = robotic_arm::protocols::ServoLCommand;

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
  void PublishLoop();
  void JointStateCallback(const std::shared_ptr<const JointStateArray>& msg);
  void ServoJCommandCallback(const std::shared_ptr<const ServoJCommand>& msg);
  void ServoJPoseCommandCallback(const std::shared_ptr<const ServoJPoseCommand>& msg);
  void ServoLCommandCallback(const std::shared_ptr<const ServoLCommand>& msg);

 private:
  aimrt::CoreRef core_;
  std::shared_ptr<aimrt::context::Context> ctx_ptr_;
  aimrt::context::res::Publisher<JointCommandArray> pub_joint_cmd_;
  aimrt::context::res::Publisher<Pose> pub_cart_expected_;
  aimrt::context::res::Publisher<Pose> pub_cart_current_;
  aimrt::context::res::Subscriber<JointStateArray> sub_joint_state_;
  aimrt::context::res::Subscriber<ServoJCommand> sub_servoj_cmd_;
  aimrt::context::res::Subscriber<ServoJPoseCommand> sub_servoj_pose_cmd_;
  aimrt::context::res::Subscriber<ServoLCommand> sub_servol_cmd_;
  // 周期控制执行器（单独）
  aimrt::executor::ExecutorRef cycle_control_executor_;

  std::promise<void> pub_stop_sig_;
  std::promise<void> control_stop_sig_;
  std::mutex last_arm_cmd_mutex_;
  std::jthread publish_thread_;

  std::string topic_pub_joint_cmd_;
  std::string topic_sub_joint_state_;
  std::string topic_pub_cart_expected_;
  std::string topic_pub_cart_current_;
  std::string topic_sub_servoj_cmd_;
  std::string topic_sub_servoj_pose_cmd_;
  std::string topic_sub_servol_cmd_;
  std::string control_executor_name_;
  double control_frequency_;
  double publish_frequency_;
  std::vector<std::string> joint_names_;

  std::shared_ptr<ArmManager> arm_manager_;
  std::shared_ptr<ArmController> arm_controller_;
  std::shared_ptr<GripperController> gripper_controller_;

  // 记录上一帧有效指令，用于轨迹间保持
  ArmMotion::JointCommand last_arm_cmd_;
  GripperController::GripperCommand last_gripper_cmd_;
  std::atomic<bool> joint_state_received_{false};
};

}  // namespace robotic_arm::control_module
