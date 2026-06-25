#pragma once

#include <memory>

#include "arm_controller.h"
#include "gripper_controller.h"
#include "move_control.aimrt_rpc.pb.h"
#include "util/log_util.h"

namespace robotic_arm::control_module {

class MoveControlServiceImpl : public robotic_arm::protocols::MoveControlServiceAsyncService {
 public:
  MoveControlServiceImpl(std::shared_ptr<ArmController> arm_controller,
                         std::shared_ptr<GripperController> gripper_controller)
      : arm_controller_(std::move(arm_controller)),
        gripper_controller_(std::move(gripper_controller)) {}
  ~MoveControlServiceImpl() override = default;

  void MoveL(
      aimrt::rpc::ContextRef ctx_ref,
      const ::robotic_arm::protocols::MoveLRequest& req,
      ::robotic_arm::protocols::MoveResponse& rsp,
      std::function<void(aimrt::rpc::Status)>&& callback) override;
  void MoveJ(
      aimrt::rpc::ContextRef ctx_ref,
      const ::robotic_arm::protocols::MoveJRequest& req,
      ::robotic_arm::protocols::MoveResponse& rsp,
      std::function<void(aimrt::rpc::Status)>&& callback) override;
  void MoveLMuti(
      aimrt::rpc::ContextRef ctx_ref,
      const ::robotic_arm::protocols::MoveLMutiRequest& req,
      ::robotic_arm::protocols::MoveResponse& rsp,
      std::function<void(aimrt::rpc::Status)>&& callback) override;
  void MoveJMuti(
      aimrt::rpc::ContextRef ctx_ref,
      const ::robotic_arm::protocols::MoveJMutiRequest& req,
      ::robotic_arm::protocols::MoveResponse& rsp,
      std::function<void(aimrt::rpc::Status)>&& callback) override;

 private:
  // 独立日志组件
  auto GetLogger() { return aimrt::common::util::SimpleLogger(); }

 private:
  std::shared_ptr<ArmController> arm_controller_;
  std::shared_ptr<GripperController> gripper_controller_;
};
}  // namespace robotic_arm::control_module