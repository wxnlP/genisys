#pragma once

#include <atomic>
#include <cstdint>
#include <future>
#include <mutex>
#include <optional>
#include <string>
#include <vector>
#include "aimrt_module_cpp_interface/context/context.h"
#include "aimrt_module_cpp_interface/module_base.h"
#include "arm_manager.h"
#include "joint_command.pb.h"
#include "joint_state.pb.h"
#include "pose.pb.h"
#include "GripperForce.aimrt_rpc.srv.h"
#include "GripperPos.aimrt_rpc.srv.h"
#include "IKCheck.aimrt_rpc.srv.h"
#include "MoveJ.aimrt_rpc.srv.h"
#include "MoveJPose.aimrt_rpc.srv.h"
#include "MoveJPoseMulti.aimrt_rpc.srv.h"
#include "MoveJMulti.aimrt_rpc.srv.h"
#include "MoveL.aimrt_rpc.srv.h"
#include "MoveLMulti.aimrt_rpc.srv.h"
#include "genisys_msgs/msg/servo_j_command.hpp"
#include "genisys_msgs/msg/servo_j_pose_command.hpp"
#include "genisys_msgs/msg/servo_l_command.hpp"
#include "genisys_msgs/msg/task_context.hpp"

using JointStateArray = aimrt::protocols::sensor::JointStateArray;
using JointCommandArray = aimrt::protocols::sensor::JointCommandArray;
using Pose = genisys::protocols::PoseStamped;

using ServoJCommand = genisys_msgs::msg::ServoJCommand;
using ServoJPoseCommand = genisys_msgs::msg::ServoJPoseCommand;
using ServoLCommand = genisys_msgs::msg::ServoLCommand;
using TaskContextMsg = genisys_msgs::msg::TaskContext;

namespace genisys::control_module {

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
  void PosePublishLoop();
  void TaskPublishLoop();
  void JointStateCallback(const std::shared_ptr<const JointStateArray>& msg);
  void ServoJCommandCallback(const std::shared_ptr<const ServoJCommand>& msg);
  void ServoJPoseCommandCallback(const std::shared_ptr<const ServoJPoseCommand>& msg);
  void ServoLCommandCallback(const std::shared_ptr<const ServoLCommand>& msg);
  void IKCheckHandler(aimrt::rpc::ContextRef ctx_ref,
                      const genisys_msgs::srv::IKCheck_Request& req,
                      genisys_msgs::srv::IKCheck_Response& rsp);

 private:
  aimrt::CoreRef core_;
  std::shared_ptr<aimrt::context::Context> ctx_ptr_;
  aimrt::context::res::Publisher<JointCommandArray> pub_joint_cmd_;
  aimrt::context::res::Publisher<Pose> pub_tcp_pose_expected_;
  aimrt::context::res::Publisher<Pose> pub_tcp_pose_;
  aimrt::context::res::Publisher<Pose> pub_flange_pose_;
  aimrt::context::res::Subscriber<JointStateArray> sub_joint_state_;
  aimrt::context::res::Subscriber<ServoJCommand> sub_servoj_cmd_;
  aimrt::context::res::Subscriber<ServoJPoseCommand> sub_servoj_pose_cmd_;
  aimrt::context::res::Subscriber<ServoLCommand> sub_servol_cmd_;
  aimrt::context::res::Publisher<TaskContextMsg> pub_task_context_;
  // 周期控制执行器（单独）
  aimrt::executor::ExecutorRef cycle_control_executor_;
  // 笛卡尔位姿发布执行器
  aimrt::executor::ExecutorRef pose_publish_executor_;
  // 任务状态发布执行器
  aimrt::executor::ExecutorRef task_publish_executor_;

  std::promise<void> pose_pub_stop_sig_;
  std::promise<void> control_stop_sig_;
  std::promise<void> task_pub_stop_sig_;
  std::mutex last_arm_cmd_mutex_;

  std::string topic_pub_joint_cmd_;
  std::string topic_sub_joint_state_;
  std::string topic_pub_tcp_pose_expected_;
  std::string topic_pub_tcp_pose_;
  std::string topic_pub_flange_pose_;
  std::string topic_sub_servoj_cmd_;
  std::string topic_sub_servoj_pose_cmd_;
  std::string topic_sub_servol_cmd_;
  std::string topic_pub_task_context_;
  std::string control_executor_name_;
  std::string pose_publish_executor_name_;
  std::string task_publish_executor_name_;
  double control_frequency_;
  double pose_publish_frequency_;
  double task_publish_frequency_;
  std::vector<std::string> joint_names_;

  uint64_t last_running_task_id_{0};
  uint64_t last_finished_task_id_{0};

  std::shared_ptr<ArmManager> arm_manager_;
  std::shared_ptr<ArmController> arm_controller_;
  std::shared_ptr<GripperController> gripper_controller_;

  // 记录上一帧有效指令，用于轨迹间保持
  std::optional<ArmMotion::JointCommand> last_arm_cmd_;
  std::optional<GripperController::GripperCommand> last_gripper_cmd_;
};

}  // namespace genisys::control_module
