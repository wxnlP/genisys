#include "control_module/control_module.h"
#include <fmt/ranges.h> // 容器元素格式化
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
      publish_executor_name_ = cfg_node["publish_executor_name"].as<std::string>();
      topic_pub_joint_cmd_ = cfg_node["topic_pub_joint_cmd"].as<std::string>();
      topic_sub_joint_state_ = cfg_node["topic_sub_joint_state"].as<std::string>();
      topic_pub_cart_expected_ = cfg_node["topic_pub_cart_expected"].as<std::string>();
      topic_pub_cart_current_ = cfg_node["topic_pub_cart_current"].as<std::string>();
      control_frequency_ = cfg_node["control_frequency"].as<double>();
      publish_frequency_ = cfg_node["publish_frequency"].as<double>();
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

    publish_executor_ = ctx_ptr_->CreateExecutor(publish_executor_name_);

    pub_joint_cmd_ = ctx_ptr_->CreatePublisher<JointCommandArray>(topic_pub_joint_cmd_);

    pub_cart_expected_ = ctx_ptr_->CreatePublisher<Pose>(topic_pub_cart_expected_);

    pub_cart_current_ = ctx_ptr_->CreatePublisher<Pose>(topic_pub_cart_current_);

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
      if (ret == 0) {
        arm_controller_->WaitForCompletion();
        rsp.error.code = 0;
        rsp.error.msg = "MoveJ succeeded";
      } else {
        rsp.error.code = ret;
        rsp.error.msg = "MoveJ failed";
      }

      return aimrt::rpc::Status();
    });

    // MoveJPose 服务处理（MoveJ 的位姿输入重载）
    movej_pose_handle->MoveJPose.ServeInline([this](aimrt::rpc::ContextRef ctx_ref,
                                                    const genisys_msgs::srv::MoveJPose_Request& req,
                                                    genisys_msgs::srv::MoveJPose_Response& rsp) {
      AIMRT_INFO("\n---\n收到MoveJPose请求\n---");
      ArmMotion::Pose6Dof target_pose(
          req.pose.position.x, req.pose.position.y, req.pose.position.z,
          req.pose.orientation.w, req.pose.orientation.x,
          req.pose.orientation.y, req.pose.orientation.z);

      int ret = arm_controller_->MoveJ(target_pose, req.v, req.a);
      if (ret == 0) {
        arm_controller_->WaitForCompletion();
        rsp.error.code = 0;
        rsp.error.msg = "MoveJPose succeeded";
      } else {
        rsp.error.code = ret;
        rsp.error.msg = "MoveJPose failed";
      }

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
      if (ret == 0) {
        arm_controller_->WaitForCompletion();
        rsp.error.code = 0;
        rsp.error.msg = "MoveJMulti succeeded";
      } else {
        rsp.error.code = ret;
        rsp.error.msg = "MoveJMulti failed";
      }

      return aimrt::rpc::Status();
    });

    // MoveJPoseMulti 服务处理（MoveJMulti 的位姿输入重载）
    movej_pose_multi_handle->MoveJPoseMulti.ServeInline([this](aimrt::rpc::ContextRef ctx_ref,
                                                               const genisys_msgs::srv::MoveJPoseMulti_Request& req,
                                                               genisys_msgs::srv::MoveJPoseMulti_Response& rsp) {
      AIMRT_INFO("\n---\n收到MoveJPoseMulti请求\n---");
      std::vector<ArmMotion::Pose6Dof> points;
      points.reserve(req.pose.size());
      for (const auto& p : req.pose) {
        points.emplace_back(
            p.position.x, p.position.y, p.position.z,
            p.orientation.w, p.orientation.x,
            p.orientation.y, p.orientation.z);
      }

      int ret = arm_controller_->MoveJMulti(points, req.r, req.v, req.a);
      if (ret == 0) {
        arm_controller_->WaitForCompletion();
        rsp.error.code = 0;
        rsp.error.msg = "MoveJPoseMulti succeeded";
      } else {
        rsp.error.code = ret;
        rsp.error.msg = "MoveJPoseMulti failed";
      }

      return aimrt::rpc::Status();
    });

    // MoveL 服务处理
    movel_handle->MoveL.ServeInline([this](aimrt::rpc::ContextRef ctx_ref,
                                           const genisys_msgs::srv::MoveL_Request& req,
                                           genisys_msgs::srv::MoveL_Response& rsp) {
      AIMRT_INFO("\n---\n收到MoveL请求\n---");
      ArmMotion::Pose6Dof target_pose(
          req.pose.position.x, req.pose.position.y, req.pose.position.z,
          req.pose.orientation.w, req.pose.orientation.x,
          req.pose.orientation.y, req.pose.orientation.z);

      int ret = arm_controller_->MoveL(target_pose, req.v, req.a);
      if (ret == 0) {
        arm_controller_->WaitForCompletion();
        rsp.error.code = 0;
        rsp.error.msg = "MoveL succeeded";
      } else {
        rsp.error.code = ret;
        rsp.error.msg = "MoveL failed";
      }

      return aimrt::rpc::Status();
    });

    // MoveLMulti 服务处理
    movel_multi_handle->MoveLMulti.ServeInline([this](aimrt::rpc::ContextRef ctx_ref,
                                                      const genisys_msgs::srv::MoveLMulti_Request& req,
                                                      genisys_msgs::srv::MoveLMulti_Response& rsp) {
      AIMRT_INFO("\n---\n收到MoveLMulti请求\n---");
      std::vector<ArmMotion::Pose6Dof> points;
      points.reserve(req.pose.size());
      for (const auto& p : req.pose) {
        points.emplace_back(
            p.position.x, p.position.y, p.position.z,
            p.orientation.w, p.orientation.x,
            p.orientation.y, p.orientation.z);
      }

      int ret = arm_controller_->MoveLMulti(points, req.r, req.v, req.a);
      if (ret == 0) {
        arm_controller_->WaitForCompletion();
        rsp.error.code = 0;
        rsp.error.msg = "MoveLMulti succeeded";
      } else {
        rsp.error.code = ret;
        rsp.error.msg = "MoveLMulti failed";
      }

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
    publish_executor_.Execute([this]() {
      ctx_ptr_->LetMe();
      PublishLoop();
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

    pub_stop_sig_.get_future().wait();
    control_stop_sig_.get_future().wait();

    if (arm_controller_) {
      arm_controller_->StopSampleThread();
    }
  } catch (const std::exception& e) {
    AIMRT_ERROR("control_module Shutdown failed, {}", e.what());
  }
  AIMRT_INFO("control_module Shutdown succeeded.");
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
  control_stop_sig_.set_value();
  AIMRT_INFO("成功退出 ControlModule::ControlLoop");
}

void ControlModule::PublishLoop() {
  try {
    // 计算周期
    auto period = std::chrono::nanoseconds(static_cast<uint64_t>(1.0 / publish_frequency_ * 1000000000));
    // 记录下一次循环时间
    auto next_loop_time = std::chrono::steady_clock::now();
    // 序列
    uint64_t seq_count = 0;

    Pose expected_pose_msg;
    Pose current_pose_msg;
    ArmMotion::Pose6Dof current_pose;
    ArmMotion::Joint6Dof joint_pos_cmd;
    ArmMotion::Pose6Dof expected_pose;
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
      if (arm_controller_->GetCurrentPose(current_pose) != 0) {
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
      if (arm_controller_->JointToPose(joint_pos_cmd, expected_pose) != 0) {
        if (fk_warn.ShouldLog(kWarnInterval)) {
          AIMRT_WARN("期望位姿正解失败（累计 {} 次）", fk_warn.count());
        }
        next_loop_time += period;
        std::this_thread::sleep_until(next_loop_time);
        continue;
      }
      fk_warn.Reset();

      // 打包msg
      expected_pose_msg.Clear();
      current_pose_msg.Clear();

      // header 时间戳统一为 system_clock
      auto time_ns = aimrt::common::util::GetCurTimestampNs();

      fill_pose_msg(expected_pose_msg, expected_pose, seq_count, time_ns);
      fill_pose_msg(current_pose_msg, current_pose, seq_count, time_ns);

      ctx_ptr_->LetMe();
      pub_cart_expected_.Publish(expected_pose_msg);
      pub_cart_current_.Publish(current_pose_msg);

      next_loop_time += period;
      std::this_thread::sleep_until(next_loop_time);
    }
  } catch (const std::exception& e) {
    AIMRT_ERROR("ControlModule::PublishLoop crashed: {}", e.what());
  }
  pub_stop_sig_.set_value();
  AIMRT_INFO("成功退出 ControlModule::PublishLoop");
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
  ArmMotion::Pose6Dof target_pose(
      p.position.x, p.position.y, p.position.z,
      p.orientation.w, p.orientation.x,
      p.orientation.y, p.orientation.z);

  if (arm_controller_->ServoJ(target_pose, dt) != 0) {
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
  ArmMotion::Pose6Dof target_pose(
      p.position.x, p.position.y, p.position.z,
      p.orientation.w, p.orientation.x,
      p.orientation.y, p.orientation.z);

  if (arm_controller_->ServoL(target_pose, dt) != 0) {
    AIMRT_ERROR("[ServoL] ServoL failed");
    return;
  }

  // 最后一条指令显式停止
  if (msg->stop) {
    arm_controller_->ServoStop();
  }
}

}  // namespace genisys::control_module
