#include "service.h"

#include "arm_type.h"

namespace robotic_arm::control_module {

void MoveControlServiceImpl::MoveL(
    aimrt::rpc::ContextRef ctx_ref,
    const ::robotic_arm::protocols::MoveLRequest& req,
    ::robotic_arm::protocols::MoveResponse& rsp,
    std::function<void(aimrt::rpc::Status)>&& callback) {
  AIMRT_INFO("\n---\n收到MoveL请求\n---");
  ArmMotion::Pose6Dof target_pose(
      req.pose().position().x(), req.pose().position().y(), req.pose().position().z(),
      req.pose().orientation().w(), req.pose().orientation().x(),
      req.pose().orientation().y(), req.pose().orientation().z());

  int ret = arm_controller_->MoveL(target_pose, req.v(), req.a());
  if (ret == 0) {
    arm_controller_->WaitForCompletion();
    rsp.mutable_error()->set_code(0);
    rsp.mutable_error()->set_msg("MoveL succeeded");
  } else {
    rsp.mutable_error()->set_code(ret);
    rsp.mutable_error()->set_msg("MoveL failed");
  }

  callback(aimrt::rpc::Status());
}

void MoveControlServiceImpl::MoveJ(
    aimrt::rpc::ContextRef ctx_ref,
    const ::robotic_arm::protocols::MoveJRequest& req,
    ::robotic_arm::protocols::MoveResponse& rsp,
    std::function<void(aimrt::rpc::Status)>&& callback) {
  AIMRT_INFO("\n---\n收到MoveJ请求\n---");
  ArmMotion::Joint6Dof target_joint;
  for (int i = 0; i < 6 && i < req.position().position_size(); ++i) {
    target_joint[i] = req.position().position(i);
  }

  int ret = arm_controller_->MoveJ(target_joint, req.v(), req.a());
  if (ret == 0) {
    arm_controller_->WaitForCompletion();
    rsp.mutable_error()->set_code(0);
    rsp.mutable_error()->set_msg("MoveJ succeeded");
  } else {
    rsp.mutable_error()->set_code(ret);
    rsp.mutable_error()->set_msg("MoveJ failed");
  }

  callback(aimrt::rpc::Status());
}

void MoveControlServiceImpl::MoveLMuti(
    aimrt::rpc::ContextRef ctx_ref,
    const ::robotic_arm::protocols::MoveLMutiRequest& req,
    ::robotic_arm::protocols::MoveResponse& rsp,
    std::function<void(aimrt::rpc::Status)>&& callback) {
  AIMRT_INFO("\n---\n收到MoveLMuti请求\n---");
  std::vector<ArmMotion::Pose6Dof> points;
  points.reserve(req.pose_size());
  for (int i = 0; i < req.pose_size(); ++i) {
    const auto& p = req.pose(i);
    points.emplace_back(
        p.position().x(), p.position().y(), p.position().z(),
        p.orientation().w(), p.orientation().x(),
        p.orientation().y(), p.orientation().z());
  }

  int ret = arm_controller_->MoveLMulti(points, req.r(), req.v(), req.a());
  if (ret == 0) {
    arm_controller_->WaitForCompletion();
    rsp.mutable_error()->set_code(0);
    rsp.mutable_error()->set_msg("MoveLMuti succeeded");
  } else {
    rsp.mutable_error()->set_code(ret);
    rsp.mutable_error()->set_msg("MoveLMuti failed");
  }

  callback(aimrt::rpc::Status());
}

void MoveControlServiceImpl::MoveJMuti(
    aimrt::rpc::ContextRef ctx_ref,
    const ::robotic_arm::protocols::MoveJMutiRequest& req,
    ::robotic_arm::protocols::MoveResponse& rsp,
    std::function<void(aimrt::rpc::Status)>&& callback) {
  AIMRT_INFO("\n---\n收到MoveJMuti请求\n---");
  std::vector<ArmMotion::Joint6Dof> points;
  points.reserve(req.position_size());
  for (int i = 0; i < req.position_size(); ++i) {
    ArmMotion::Joint6Dof joint;
    const auto& j = req.position(i);
    for (int k = 0; k < 6 && k < j.position_size(); ++k) {
      joint[k] = j.position(k);
    }
    points.push_back(joint);
  }

  int ret = arm_controller_->MoveJMulti(points, req.r(), req.v(), req.a());
  if (ret == 0) {
    arm_controller_->WaitForCompletion();
    rsp.mutable_error()->set_code(0);
    rsp.mutable_error()->set_msg("MoveJMuti succeeded");
  } else {
    rsp.mutable_error()->set_code(ret);
    rsp.mutable_error()->set_msg("MoveJMuti failed");
  }

  callback(aimrt::rpc::Status());
}

}  // namespace robotic_arm::control_module