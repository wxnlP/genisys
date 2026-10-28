#include "control_module/control_module.h"
#include <fmt/ranges.h>  // 容器元素格式化
#include <chrono>
#include <cmath>
#include <ranges>
#include <thread>
#include "aimrt_module_protobuf_interface/channel/protobuf_channel.h"
#include "aimrt_module_ros2_interface/channel/ros2_channel.h"
#include "common/warn_throttle.h"
#include "util/time_util.h"
#include "yaml-cpp/yaml.h"

namespace genisys::control_module {

// 状态读取失败的告警节流周期
constexpr uint32_t kWarnInterval = 500;

bool ControlModule::Initialize(aimrt::CoreRef core) {
  // Save aimrt framework handle
  core_ = core;
  ctx_ptr_ = std::make_shared<aimrt::context::Context>(core_);
  ctx_ptr_->LetMe();
  try {
    // Read cfg
    auto file_path = ctx_ptr_->GetConfigFilePath();

    if (!file_path.empty()) {
      YAML::Node cfg_node = YAML::LoadFile(std::string(file_path));
      control_executor_name_ = cfg_node["control_executor_name"].as<std::string>();
      pose_publish_executor_name_ = cfg_node["pose_publish_executor_name"].as<std::string>();
      topic_pub_joint_cmd_ = cfg_node["topic_pub_joint_cmd"].as<std::string>();
      topic_sub_joint_state_ = cfg_node["topic_sub_joint_state"].as<std::string>();
      topic_pub_tcp_pose_expected_ = cfg_node["topic_pub_tcp_pose_expected"].as<std::string>();
      topic_pub_tcp_pose_ = cfg_node["topic_pub_tcp_pose"].as<std::string>();
      topic_pub_flange_pose_ = cfg_node["topic_pub_flange_pose"].as<std::string>();
      control_frequency_ = cfg_node["control_frequency"].as<double>();
      pose_publish_frequency_ = cfg_node["pose_publish_frequency"].as<double>();
      task_publish_frequency_ = cfg_node["task_publish_frequency"].as<double>();
      topic_pub_task_context_ = cfg_node["topic_pub_task_context"].as<std::string>();
      task_publish_executor_name_ = cfg_node["task_publish_executor_name"].as<std::string>();
      topic_sub_servoj_cmd_ = cfg_node["topic_sub_servoj_cmd"].as<std::string>();
      topic_sub_servoj_pose_cmd_ = cfg_node["topic_sub_servoj_pose_cmd"].as<std::string>();
      topic_sub_servol_cmd_ = cfg_node["topic_sub_servol_cmd"].as<std::string>();
      auto robot_cfg_file = cfg_node["config_file"].as<std::string>();

      arm_manager_ = std::make_shared<ArmManager>(robot_cfg_file);
      AIMRT_CHECK_ERROR_THROW(arm_manager_, "Failed to create ArmManager");

      arm_controller_ = arm_manager_->GetArmController();
      AIMRT_CHECK_ERROR_THROW(arm_controller_, "Failed to get ArmController");

      gripper_controller_ = arm_manager_->GetGripperController();

      arm_manager_->GetJointNames(joint_names_);
      AIMRT_INFO("\n---\n控制器器关节名称：{}\n---", joint_names_);
    }
    cycle_control_executor_ = ctx_ptr_->CreateExecutor(control_executor_name_);

    pose_publish_executor_ = ctx_ptr_->CreateExecutor(pose_publish_executor_name_);

    task_publish_executor_ = ctx_ptr_->CreateExecutor(task_publish_executor_name_);

    pub_joint_cmd_ = ctx_ptr_->CreatePublisher<JointCommandArray>(topic_pub_joint_cmd_);

    pub_tcp_pose_expected_ = ctx_ptr_->CreatePublisher<Pose>(topic_pub_tcp_pose_expected_);

    pub_tcp_pose_ = ctx_ptr_->CreatePublisher<Pose>(topic_pub_tcp_pose_);

    pub_flange_pose_ = ctx_ptr_->CreatePublisher<Pose>(topic_pub_flange_pose_);

    pub_task_context_ = ctx_ptr_->CreatePublisher<TaskContextMsg>(topic_pub_task_context_);

    sub_joint_state_ = ctx_ptr_->CreateSubscriber<JointStateArray>(topic_sub_joint_state_, std::bind(&ControlModule::JointStateCallback, this, std::placeholders::_1));

    // 伺服指令通道（Channel）
    sub_servoj_cmd_ = ctx_ptr_->CreateSubscriber<ServoJCommand>(
        topic_sub_servoj_cmd_, std::bind(&ControlModule::ServoJCommandCallback, this, std::placeholders::_1));
    sub_servoj_pose_cmd_ = ctx_ptr_->CreateSubscriber<ServoJPoseCommand>(
        topic_sub_servoj_pose_cmd_, std::bind(&ControlModule::ServoJPoseCommandCallback, this, std::placeholders::_1));
    sub_servol_cmd_ = ctx_ptr_->CreateSubscriber<ServoLCommand>(
        topic_sub_servol_cmd_, std::bind(&ControlModule::ServoLCommandCallback, this, std::placeholders::_1));

    // 创建 RPC 服务
    auto movej_handle = ctx_ptr_->CreateServer<genisys_msgs::srv::MoveJCoServer>();
    auto movej_pose_handle = ctx_ptr_->CreateServer<genisys_msgs::srv::MoveJPoseCoServer>();
    auto movej_multi_handle = ctx_ptr_->CreateServer<genisys_msgs::srv::MoveJMultiCoServer>();
    auto movej_pose_multi_handle = ctx_ptr_->CreateServer<genisys_msgs::srv::MoveJPoseMultiCoServer>();
    auto movel_handle = ctx_ptr_->CreateServer<genisys_msgs::srv::MoveLCoServer>();
    auto movel_multi_handle = ctx_ptr_->CreateServer<genisys_msgs::srv::MoveLMultiCoServer>();
    auto gripper_pos_handle = ctx_ptr_->CreateServer<genisys_msgs::srv::GripperPosCoServer>();
    auto gripper_force_handle = ctx_ptr_->CreateServer<genisys_msgs::srv::GripperForceCoServer>();

    // MoveJ 服务处理
    movej_handle->MoveJ.ServeInline([this](aimrt::rpc::ContextRef ctx_ref,
                                           const genisys_msgs::srv::MoveJ_Request& req,
                                           genisys_msgs::srv::MoveJ_Response& rsp) {
      AIMRT_INFO("\n---\n收到MoveJ请求\n---");
      ArmMotion::Joint6Dof target_joint;
      for (int i = 0; i < 6; ++i) {
        target_joint[i] = req.position.position[i];
      }

      int ret = arm_controller_->MoveJ(target_joint, req.v, req.a);
      if (ret != 0) {
        rsp.error.code = ret;
        rsp.error.msg = "MoveJ failed";
        return aimrt::rpc::Status();
      }
      ArmController::TaskContext task;
      if (arm_controller_->GetCurrentTaskContext(task)) {
        rsp.task_id = task.id;
        rsp.duration_ms = task.duration_ms;
      }
      rsp.error.code = 0;
      rsp.error.msg = "MoveJ accepted";

      return aimrt::rpc::Status();
    });

    // MoveJPose 服务处理（MoveJ 的位姿输入重载）
    movej_pose_handle->MoveJPose.ServeInline([this](aimrt::rpc::ContextRef ctx_ref,
                                                    const genisys_msgs::srv::MoveJPose_Request& req,
                                                    genisys_msgs::srv::MoveJPose_Response& rsp) {
      AIMRT_INFO("\n---\n收到MoveJPose请求\n---");
      ArmMotion::Pose6Dof target_tcp(
          req.pose.position.x, req.pose.position.y, req.pose.position.z,
          req.pose.orientation.w, req.pose.orientation.x,
          req.pose.orientation.y, req.pose.orientation.z);

      int ret = arm_controller_->MoveJ(target_tcp, req.v, req.a);
      if (ret != 0) {
        rsp.error.code = ret;
        rsp.error.msg = "MoveJPose failed";
        return aimrt::rpc::Status();
      }
      ArmController::TaskContext task;
      if (arm_controller_->GetCurrentTaskContext(task)) {
        rsp.task_id = task.id;
        rsp.duration_ms = task.duration_ms;
      }
      rsp.error.code = 0;
      rsp.error.msg = "MoveJPose accepted";

      return aimrt::rpc::Status();
    });

    // MoveJMulti 服务处理
    movej_multi_handle->MoveJMulti.ServeInline([this](aimrt::rpc::ContextRef ctx_ref,
                                                      const genisys_msgs::srv::MoveJMulti_Request& req,
                                                      genisys_msgs::srv::MoveJMulti_Response& rsp) {
      AIMRT_INFO("\n---\n收到MoveJMulti请求\n---");
      std::vector<ArmMotion::Joint6Dof> points;
      points.reserve(req.position.size());
      for (const auto& j : req.position) {
        points.emplace_back(j.position[0], j.position[1], j.position[2],
                            j.position[3], j.position[4], j.position[5]);
      }

      int ret = arm_controller_->MoveJMulti(points, req.r, req.v, req.a);
      if (ret != 0) {
        rsp.error.code = ret;
        rsp.error.msg = "MoveJMulti failed";
        return aimrt::rpc::Status();
      }
      ArmController::TaskContext task;
      if (arm_controller_->GetCurrentTaskContext(task)) {
        rsp.task_id = task.id;
        rsp.duration_ms = task.duration_ms;
      }
      rsp.error.code = 0;
      rsp.error.msg = "MoveJMulti accepted";

      return aimrt::rpc::Status();
    });

    // MoveJPoseMulti 服务处理（MoveJMulti 的位姿输入重载）
    movej_pose_multi_handle->MoveJPoseMulti.ServeInline([this](aimrt::rpc::ContextRef ctx_ref,
                                                               const genisys_msgs::srv::MoveJPoseMulti_Request& req,
                                                               genisys_msgs::srv::MoveJPoseMulti_Response& rsp) {
      AIMRT_INFO("\n---\n收到MoveJPoseMulti请求\n---");
      std::vector<ArmMotion::Pose6Dof> tcp_points;
      tcp_points.reserve(req.pose.size());
      for (const auto& p : req.pose) {
        tcp_points.emplace_back(
            p.position.x, p.position.y, p.position.z,
            p.orientation.w, p.orientation.x,
            p.orientation.y, p.orientation.z);
      }

      int ret = arm_controller_->MoveJMulti(tcp_points, req.r, req.v, req.a);
      if (ret != 0) {
        rsp.error.code = ret;
        rsp.error.msg = "MoveJPoseMulti failed";
        return aimrt::rpc::Status();
      }
      ArmController::TaskContext task;
      if (arm_controller_->GetCurrentTaskContext(task)) {
        rsp.task_id = task.id;
        rsp.duration_ms = task.duration_ms;
      }
      rsp.error.code = 0;
      rsp.error.msg = "MoveJPoseMulti accepted";

      return aimrt::rpc::Status();
    });

    // MoveL 服务处理
    movel_handle->MoveL.ServeInline([this](aimrt::rpc::ContextRef ctx_ref,
                                           const genisys_msgs::srv::MoveL_Request& req,
                                           genisys_msgs::srv::MoveL_Response& rsp) {
      AIMRT_INFO("\n---\n收到MoveL请求\n---");
      ArmMotion::Pose6Dof target_tcp(
          req.pose.position.x, req.pose.position.y, req.pose.position.z,
          req.pose.orientation.w, req.pose.orientation.x,
          req.pose.orientation.y, req.pose.orientation.z);

      int ret = arm_controller_->MoveL(target_tcp, req.v, req.a);
      if (ret != 0) {
        rsp.error.code = ret;
        rsp.error.msg = "MoveL failed";
        return aimrt::rpc::Status();
      }
      ArmController::TaskContext task;
      if (arm_controller_->GetCurrentTaskContext(task)) {
        rsp.task_id = task.id;
        rsp.duration_ms = task.duration_ms;
      }
      rsp.error.code = 0;
      rsp.error.msg = "MoveL accepted";

      return aimrt::rpc::Status();
    });

    // MoveLMulti 服务处理
    movel_multi_handle->MoveLMulti.ServeInline([this](aimrt::rpc::ContextRef ctx_ref,
                                                      const genisys_msgs::srv::MoveLMulti_Request& req,
                                                      genisys_msgs::srv::MoveLMulti_Response& rsp) {
      AIMRT_INFO("\n---\n收到MoveLMulti请求\n---");
      std::vector<ArmMotion::Pose6Dof> tcp_points;
      tcp_points.reserve(req.pose.size());
      for (const auto& p : req.pose) {
        tcp_points.emplace_back(
            p.position.x, p.position.y, p.position.z,
            p.orientation.w, p.orientation.x,
            p.orientation.y, p.orientation.z);
      }

      int ret = arm_controller_->MoveLMulti(tcp_points, req.r, req.v, req.a);
      if (ret != 0) {
        rsp.error.code = ret;
        rsp.error.msg = "MoveLMulti failed";
        return aimrt::rpc::Status();
      }
      ArmController::TaskContext task;
      if (arm_controller_->GetCurrentTaskContext(task)) {
        rsp.task_id = task.id;
        rsp.duration_ms = task.duration_ms;
      }
      rsp.error.code = 0;
      rsp.error.msg = "MoveLMulti accepted";

      return aimrt::rpc::Status();
    });

    // GripperPos 服务处理：位置控制
    gripper_pos_handle->GripperPos.ServeInline([this](aimrt::rpc::ContextRef ctx_ref,
                                                      const genisys_msgs::srv::GripperPos_Request& req,
                                                      genisys_msgs::srv::GripperPos_Response& rsp) {
      if (!gripper_controller_) {
        AIMRT_ERROR("收到 GripperPos 请求，但当前配置未启用夹爪");
        rsp.error.code = -1;
        rsp.error.msg = "Gripper is not configured in body cfg";
        return aimrt::rpc::Status();
      }
      AIMRT_INFO("\n---\n收到GripperPos请求\n---");
      bool ret = gripper_controller_->PosControl(
          req.position, req.velocity, req.acceleration, req.effort);
      if (ret) {
        gripper_controller_->WaitForCompletion();
        rsp.error.code = 0;
        rsp.error.msg = "GripperPos succeeded";
      } else {
        rsp.error.code = -1;
        rsp.error.msg = "GripperPos failed";
      }

      return aimrt::rpc::Status();
    });

    // GripperForce 服务处理：力控抓取
    gripper_force_handle->GripperForce.ServeInline(
        [this](aimrt::rpc::ContextRef ctx_ref,
               const genisys_msgs::srv::GripperForce_Request& req,
               genisys_msgs::srv::GripperForce_Response& rsp) {
          if (!gripper_controller_) {
            AIMRT_ERROR("收到 GripperForce 请求，但当前配置未启用夹爪");
            rsp.error.code = -1;
            rsp.error.msg = "Gripper is not configured in body cfg";
            return aimrt::rpc::Status();
          }
          AIMRT_INFO("\n---\n收到GripperForce请求\n---");

          GripperController::ForceControlParams params;
          params.approach_velocity = req.approach_velocity;
          params.approach_accel = req.approach_accel;
          params.switch_velocity = req.switch_velocity;
          params.target_torque = req.target_torque;
          params.closed_release = req.closed_release;

          bool ret = gripper_controller_->ForceControl(params);
          if (ret) {
            rsp.error.code = 0;
            rsp.error.msg = "GripperForce succeeded";
          } else {
            rsp.error.code = -1;
            rsp.error.msg = "GripperForce failed";
          }

          return aimrt::rpc::Status();
        });

    // IKCheck 服务处理：批量逆解查询（只读，不产生运动任务）
    auto ik_check_handle = ctx_ptr_->CreateServer<genisys_msgs::srv::IKCheckCoServer>();
    ik_check_handle->IKCheck.ServeInline(
        [this](aimrt::rpc::ContextRef ctx_ref,
               const genisys_msgs::srv::IKCheck_Request& req,
               genisys_msgs::srv::IKCheck_Response& rsp) {
          IKCheckHandler(ctx_ref, req, rsp);
          return aimrt::rpc::Status();
        });

  } catch (const std::exception& e) {
    AIMRT_ERROR("Init failed, {}", e.what());
    return false;
  }

  AIMRT_INFO("Init succeeded.");

  return true;
}

bool ControlModule::Start() {
  try {
    cycle_control_executor_.Execute([this]() {
      ctx_ptr_->LetMe();
      ControlLoop();
    });
    pose_publish_executor_.Execute([this]() {
      ctx_ptr_->LetMe();
      PosePublishLoop();
    });
    task_publish_executor_.Execute([this]() {
      ctx_ptr_->LetMe();
      TaskPublishLoop();
    });

  } catch (const std::exception& e) {
    AIMRT_ERROR("control_module Start failed, {}", e.what());
    return false;
  }
  AIMRT_INFO("control_module Started succeeded.");
  return true;
}

void ControlModule::Shutdown() {
  try {
    ctx_ptr_->StopRunning();

    if (arm_controller_) {
      arm_controller_->StopSampleThread();
    }
  } catch (const std::exception& e) {
    AIMRT_ERROR("control_module Shutdown failed, {}", e.what());
  }
  AIMRT_INFO("control_module Shutdown succeeded.");
}

void ControlModule::TaskPublishLoop() {
  try {
    auto period = std::chrono::nanoseconds(static_cast<uint64_t>(1.0 / task_publish_frequency_ * 1000000000));
    auto next_loop_time = std::chrono::steady_clock::now();

    std::vector<ArmController::TaskContext> history;
    std::vector<ArmController::TaskContext> pending;

    while (ctx_ptr_->Running()) {
      pending.clear();

      if (arm_controller_->GetHistoryTaskContext(history)) {
        std::ranges::sort(history, {}, &ArmController::TaskContext::id);
        for (const auto& ctx : history) {
          if (ctx.id <= last_finished_task_id_) {
            continue;
          }
          pending.push_back(ctx);
          last_finished_task_id_ = ctx.id;
        }
      }

      ArmController::TaskContext current;
      if (arm_controller_->GetCurrentTaskContext(current) && current.id != last_running_task_id_) {
        pending.push_back(current);
        last_running_task_id_ = current.id;
      }

      for (const auto& ctx : pending) {
        auto time_ns = aimrt::common::util::GetCurTimestampNs();

        TaskContextMsg msg;
        msg.header.stamp.sec = static_cast<int32_t>(time_ns / 1000000000);
        msg.header.stamp.nanosec = static_cast<uint32_t>(time_ns % 1000000000);
        msg.header.frame_id = "base_link";
        msg.id = ctx.id;
        msg.duration_ms = ctx.duration_ms;
        const int64_t end_ns = ctx.end_timestamp.time_since_epoch().count();
        msg.end_timestamp.sec = static_cast<int32_t>(end_ns / 1000000000);
        msg.end_timestamp.nanosec = static_cast<uint32_t>(end_ns % 1000000000);
        msg.state = static_cast<int8_t>(ctx.state);
        msg.code = ctx.code;

        ctx_ptr_->LetMe();
        pub_task_context_.Publish(msg);
      }

      next_loop_time += period;
      std::this_thread::sleep_until(next_loop_time);
    }
  } catch (const std::exception& e) {
    AIMRT_ERROR("ControlModule::TaskPublishLoop crashed: {}", e.what());
  }
  AIMRT_INFO("成功退出 ControlModule::TaskPublishLoop");
}

void ControlModule::ControlLoop() {
  try {
    // 计算周期
    auto period = std::chrono::nanoseconds(static_cast<uint64_t>(1.0 / control_frequency_ * 1000000000));
    // 记录下一次循环时间
    auto next_loop_time = std::chrono::steady_clock::now();
    // 序列
    uint64_t seq_count = 0;

    JointCommandArray joint_cmd_msg;
    joint_cmd_msg.mutable_joints()->Reserve(7);
    ArmMotion::JointCommand joint_cmd;
    GripperController::GripperCommand gripper_cmd;
    // 告警节流
    WarnThrottle lag_warn;

    // 周期计时统计
    uint64_t dbg_cycle = 0;
    int64_t cycle_max_us = 0;
    int64_t get_cmd_sum_us = 0, build_msg_sum_us = 0, publish_sum_us = 0;

    while (ctx_ptr_->Running()) {
      auto t_start = std::chrono::steady_clock::now();

      joint_cmd_msg.Clear();
      // header 时间戳统一为 system_clock
      auto time_ns = aimrt::common::util::GetCurTimestampNs();

      auto* header = joint_cmd_msg.mutable_header();
      header->set_time_stamp(time_ns);
      header->set_frame_id("base_link");
      header->set_sequence_num(seq_count++);

      {
        std::lock_guard<std::mutex> lock(last_arm_cmd_mutex_);
        if (!arm_controller_->GetJointCommand(joint_cmd)) {
          // 无规划轨迹 — 复用上一帧指令保持
          if (last_arm_cmd_.has_value()) {
            joint_cmd = *last_arm_cmd_;
          } else {
            ArmMotion::Joint6Dof current_joint;
            if (arm_controller_->GetCurrentJoint(current_joint) != 0) {
              next_loop_time += period;
              std::this_thread::sleep_until(next_loop_time);
              continue;
            }
            joint_cmd.position = current_joint;
            joint_cmd.velocity = ArmMotion::Joint6Dof{};
            joint_cmd.effort = ArmMotion::Joint6Dof{};
            last_arm_cmd_ = joint_cmd;
          }
        } else {
          // 记录上一次指令
          last_arm_cmd_ = joint_cmd;
        }
      }
      auto t_get_cmd = std::chrono::steady_clock::now();

      arm_controller_->MotionStateHandle(std::chrono::system_clock::now());

      for (int i = 0; i < 6; ++i) {
        auto* joint = joint_cmd_msg.add_joints();
        joint->set_name(joint_names_[i]);
        joint->set_position(joint_cmd.position[i]);
        joint->set_velocity(joint_cmd.velocity[i]);
        joint->set_effort(joint_cmd.effort[i]);
        joint->set_stiffness(joint_cmd.stiffness[i]);
        joint->set_damping(joint_cmd.damping[i]);
      }
      if (gripper_controller_) {
        if (gripper_controller_->GetGripperCommand(gripper_cmd) == -1) {
          // 无规划轨迹 — 复用上一帧指令保持
          if (last_gripper_cmd_.has_value()) {
            gripper_cmd = *last_gripper_cmd_;
          } else {
            double current_position = 0.0;
            if (gripper_controller_->GetCurrentPosition(current_position) != 0) {
              next_loop_time += period;
              std::this_thread::sleep_until(next_loop_time);
              continue;
            }
            gripper_cmd.position = current_position;
            gripper_cmd.velocity = 0.0;
            gripper_cmd.effort = 0.0;
            last_gripper_cmd_ = gripper_cmd;
          }
        } else {
          last_gripper_cmd_ = gripper_cmd;
        }
        auto* joint = joint_cmd_msg.add_joints();
        joint->set_name(joint_names_.back());
        joint->set_position(gripper_cmd.position);
        joint->set_velocity(gripper_cmd.velocity);
        joint->set_effort(gripper_cmd.effort);
        joint->set_stiffness(gripper_cmd.stiffness);
        joint->set_damping(gripper_cmd.damping);
      }
      auto t_build_msg = std::chrono::steady_clock::now();

      ctx_ptr_->LetMe();
      pub_joint_cmd_.Publish(joint_cmd_msg);

      auto t_publish = std::chrono::steady_clock::now();

      // 每 200 周期打印一次详细计时
      auto get_cmd_us = std::chrono::duration_cast<std::chrono::microseconds>(t_get_cmd - t_start).count();
      auto build_msg_us = std::chrono::duration_cast<std::chrono::microseconds>(t_build_msg - t_get_cmd).count();
      auto publish_us = std::chrono::duration_cast<std::chrono::microseconds>(t_publish - t_build_msg).count();
      auto total_us = std::chrono::duration_cast<std::chrono::microseconds>(t_publish - t_start).count();

      dbg_cycle++;

      if (dbg_cycle % 200 == 0) {
        AIMRT_DEBUG(
            "[CycleTiming] Slow cycle: get_cmd={}μs, build_msg={}μs, publish={}μs, total={}μs",
            get_cmd_us, build_msg_us, publish_us, total_us);
      }

      next_loop_time += period;
      auto now = std::chrono::steady_clock::now();
      if (next_loop_time < now - period) {  // 落后超过 1 个周期
        if (lag_warn.ShouldLog(kWarnInterval)) {
          AIMRT_WARN("Control loop is lagging behind, skip to next cycle.");
          AIMRT_WARN(
              "\n---\nNow cycle: get_cmd={}μs, build_msg={}μs, publish={}μs, total={}μs\n---",
              get_cmd_us, build_msg_us, publish_us, total_us);
        }
        next_loop_time = now + period;  // 放弃追赶，重新对齐
      }
      std::this_thread::sleep_until(next_loop_time);
    }
  } catch (const std::exception& e) {
    AIMRT_ERROR("ControlModule::ControlLoop crashed: {}", e.what());
  }
  AIMRT_INFO("成功退出 ControlModule::ControlLoop");
}

void ControlModule::PosePublishLoop() {
  try {
    // 计算周期
    auto period = std::chrono::nanoseconds(static_cast<uint64_t>(1.0 / pose_publish_frequency_ * 1000000000));
    // 记录下一次循环时间
    auto next_loop_time = std::chrono::steady_clock::now();
    // 序列
    uint64_t seq_count = 0;

    Pose tcp_pose_expected_msg;
    Pose tcp_pose_msg;
    Pose flange_pose_msg;
    EndEffectorPoses end_effector_poses;
    ArmMotion::Joint6Dof joint_pos_cmd;
    ArmMotion::Pose6Dof tcp_pose_expected;
    // 告警节流
    WarnThrottle pose_warn;
    WarnThrottle fk_warn;

    auto fill_pose_msg = [this](Pose& msg, const ArmMotion::Pose6Dof& source_pose, uint64_t& seq, const auto& time_ns) {
      auto* header = msg.mutable_header();
      header->set_time_stamp(time_ns);
      header->set_frame_id("base_link");
      header->set_sequence_num(seq++);

      auto* pose = msg.mutable_pose();

      auto* quat = pose->mutable_orientation();
      quat->set_x(source_pose.quat.x);
      quat->set_y(source_pose.quat.y);
      quat->set_z(source_pose.quat.z);
      quat->set_w(source_pose.quat.w);

      auto* pos = pose->mutable_position();
      pos->set_x(source_pose.x);
      pos->set_y(source_pose.y);
      pos->set_z(source_pose.z);
    };

    while (ctx_ptr_->Running()) {
      // 获取期望和当前笛卡尔位姿
      if (arm_controller_->GetEndEffectorPoses(end_effector_poses) != 0) {
        if (pose_warn.ShouldLog(kWarnInterval)) {
          AIMRT_WARN("读取机械臂位姿失败（累计 {} 次）", pose_warn.count());
        }
        next_loop_time += period;
        std::this_thread::sleep_until(next_loop_time);
        continue;
      }
      pose_warn.Reset();
      {
        std::lock_guard<std::mutex> lock(last_arm_cmd_mutex_);
        if (last_arm_cmd_.has_value()) {
          joint_pos_cmd = last_arm_cmd_->position;
        }
      }
      if (arm_controller_->JointToTCPPose(joint_pos_cmd, tcp_pose_expected) != 0) {
        if (fk_warn.ShouldLog(kWarnInterval)) {
          AIMRT_WARN("期望位姿正解失败（累计 {} 次）", fk_warn.count());
        }
        next_loop_time += period;
        std::this_thread::sleep_until(next_loop_time);
        continue;
      }
      fk_warn.Reset();

      // 打包msg
      tcp_pose_expected_msg.Clear();
      tcp_pose_msg.Clear();
      flange_pose_msg.Clear();

      // header 时间戳统一为 system_clock
      auto time_ns = aimrt::common::util::GetCurTimestampNs();

      fill_pose_msg(tcp_pose_expected_msg, tcp_pose_expected, seq_count, time_ns);
      fill_pose_msg(tcp_pose_msg, end_effector_poses.tcp, seq_count, time_ns);
      fill_pose_msg(flange_pose_msg, end_effector_poses.flange, seq_count, time_ns);

      ctx_ptr_->LetMe();
      pub_tcp_pose_expected_.Publish(tcp_pose_expected_msg);
      pub_tcp_pose_.Publish(tcp_pose_msg);
      pub_flange_pose_.Publish(flange_pose_msg);

      next_loop_time += period;
      std::this_thread::sleep_until(next_loop_time);
    }
  } catch (const std::exception& e) {
    AIMRT_ERROR("ControlModule::PosePublishLoop crashed: {}", e.what());
  }
  AIMRT_INFO("成功退出 ControlModule::PosePublishLoop");
}

void ControlModule::JointStateCallback(const std::shared_ptr<const JointStateArray>& msg) {
  if (!ctx_ptr_->Running()) {
    return;
  }

  ArmMotion::JointState joint_state;
  int n = msg->joints_size();

  if (n < 6) {
    AIMRT_WARN("joint_state 关节数 {} < 6，丢弃", n);
    return;
  }

  for (int i = 0; i < 6; ++i) {
    joint_state.position[i] = msg->joints(i).position();
    joint_state.velocity[i] = msg->joints(i).velocity();
    joint_state.effort[i] = msg->joints(i).effort();
  }

  // 更新机械臂控制的状态
  arm_controller_->SetJointState(joint_state);
  // 更新夹爪控制器的状态
  if (gripper_controller_ && n >= 7) {
    auto joint = msg->joints(6);
    gripper_controller_->SetGripperState(joint.position(), joint.velocity(), joint.effort());
  }
}

void ControlModule::ServoJCommandCallback(const std::shared_ptr<const ServoJCommand>& msg) {
  if (!ctx_ptr_->Running() || !msg) {
    return;
  }

  if (msg->target.size() < 6) {
    AIMRT_ERROR("[ServoJ] invalid target size: {}", msg->target.size());
    return;
  }

  const double dt = msg->dt;

  ArmMotion::Joint6Dof target_joint;
  for (int i = 0; i < 6; ++i) {
    target_joint[i] = msg->target[i];
  }

  if (arm_controller_->ServoJ(target_joint, dt) != 0) {
    AIMRT_ERROR("[ServoJ] ServoJ failed");
    return;
  }
  // 最后一条指令显式停止
  if (msg->stop) {
    arm_controller_->ServoStop();
  }
}

void ControlModule::ServoJPoseCommandCallback(const std::shared_ptr<const ServoJPoseCommand>& msg) {
  if (!ctx_ptr_->Running() || !msg) {
    return;
  }

  const double dt = msg->dt;

  const auto& p = msg->target;
  ArmMotion::Pose6Dof target_tcp(
      p.position.x, p.position.y, p.position.z,
      p.orientation.w, p.orientation.x,
      p.orientation.y, p.orientation.z);

  if (arm_controller_->ServoJ(target_tcp, dt) != 0) {
    AIMRT_ERROR("[ServoJPose] ServoJ(pose) failed");
    return;
  }

  // 最后一条指令显式停止
  if (msg->stop) {
    arm_controller_->ServoStop();
  }
}

void ControlModule::ServoLCommandCallback(const std::shared_ptr<const ServoLCommand>& msg) {
  if (!ctx_ptr_->Running() || !msg) {
    return;
  }

  const double dt = msg->dt;

  const auto& p = msg->target;
  ArmMotion::Pose6Dof target_tcp(
      p.position.x, p.position.y, p.position.z,
      p.orientation.w, p.orientation.x,
      p.orientation.y, p.orientation.z);

  if (arm_controller_->ServoL(target_tcp, dt) != 0) {
    AIMRT_ERROR("[ServoL] ServoL failed");
    return;
  }

  // 最后一条指令显式停止
  if (msg->stop) {
    arm_controller_->ServoStop();
  }
}

void ControlModule::IKCheckHandler(aimrt::rpc::ContextRef ctx_ref,
                                   const genisys_msgs::srv::IKCheck_Request& req,
                                   genisys_msgs::srv::IKCheck_Response& rsp) {
  (void)ctx_ref;
  const auto t_start = std::chrono::steady_clock::now();
  const auto& targets = req.request;

  rsp.results.clear();
  rsp.results.reserve(targets.size());

  if (targets.size() > 64) {
    AIMRT_WARN("IKCheck: 大批量请求 n={}，会占用 RPC 线程较久", targets.size());
  }

  for (const auto& target : targets) {
    auto& result = rsp.results.emplace_back();
    result.id = target.id;
    result.joint.position.fill(0.0);

    const auto& p = target.pose;
    const double qw = p.orientation.w;
    const double qx = p.orientation.x;
    const double qy = p.orientation.y;
    const double qz = p.orientation.z;
    const double quat_norm = std::sqrt(qw * qw + qx * qx + qy * qy + qz * qz);

    const bool frame_ok = (target.frame == genisys_msgs::msg::IKRequest::FRAME_TCP) ||
                          (target.frame == genisys_msgs::msg::IKRequest::FRAME_FLANGE);
    if (!frame_ok) {
      result.status = genisys_msgs::msg::IKResult::INVALID_INPUT;
      result.solved = false;
      continue;
    }

    ArmMotion::Pose6Dof pose(p.position.x, p.position.y, p.position.z, qw, qx, qy, qz);
    const bool is_flange = (target.frame == genisys_msgs::msg::IKRequest::FRAME_FLANGE);

    ArmMotion::Joint6Dof joint;
    const int ret = arm_controller_->SolveIK(pose, is_flange, target.do_fk, joint);

    if (ret == 0) {
      result.status = genisys_msgs::msg::IKResult::OK;
      result.solved = true;
      for (int i = 0; i < 6; ++i) {
        result.joint.position[i] = joint[i];
      }
    } else if (ret == -2) {
      // -2: 没有通过限位的分支；
      result.status = genisys_msgs::msg::IKResult::NO_SOLUTION;
      result.solved = false;
    } else if (ret == -3 ){
      // -3: 选中的解过不了 FK 反代
      result.status = genisys_msgs::msg::IKResult::FK_VERIFY_WARN;
      result.solved = false;
    } else {
      result.status = genisys_msgs::msg::IKResult::OTHER_ERRORS;
      result.solved = false;
      AIMRT_WARN("IK Check failed with OTHER_ERRORS");
    }
  }
}

}  // namespace genisys::control_module
