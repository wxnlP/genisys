#include "control_module/control_module.h"
#include <fmt/ranges.h>
#include <thread>
#include <pthread.h>
#include "aimrt_module_protobuf_interface/channel/protobuf_channel.h"
#include "yaml-cpp/yaml.h"

namespace robotic_arm::control_module {

bool ControlModule::Initialize(aimrt::CoreRef core) {
  // Save aimrt framework handle
  core_ = core;

  try {
    // Read cfg
    auto file_path = core_.GetConfigurator().GetConfigFilePath();
    AIMRT_CHECK_ERROR_THROW(!file_path.empty(), "Config file path is empty");

    YAML::Node cfg_node = YAML::LoadFile(std::string(file_path));
    control_executor_name_ = cfg_node["control_executor_name"].as<std::string>();
    topic_pub_joint_cmd_ = cfg_node["topic_pub_joint_cmd"].as<std::string>();
    topic_sub_joint_state_ = cfg_node["topic_sub_joint_state"].as<std::string>();
    topic_pub_cart_expected_ = cfg_node["topic_pub_cart_expected"].as<std::string>();
    topic_pub_cart_current_ = cfg_node["topic_pub_cart_current"].as<std::string>();
    control_frequency_ = cfg_node["control_frequency"].as<double>();
    publish_frequency_ = cfg_node["publish_frequency"].as<double>();
    auto robot_cfg_file = cfg_node["config_file"].as<std::string>();

    arm_manager_ = std::make_shared<ArmManager>(robot_cfg_file);
    AIMRT_CHECK_ERROR_THROW(arm_manager_, "Failed to create ArmManager");

    arm_controller_ = arm_manager_->GetArmController();
    AIMRT_CHECK_ERROR_THROW(arm_controller_, "Failed to get ArmController");

    gripper_controller_ = arm_manager_->GetGripperController();

    arm_manager_->GetJointNames(joint_names_);
    AIMRT_INFO("\n---\n控制器器关节名称：{}\n---", joint_names_);

    cycle_control_executor_ = core_.GetExecutorManager().GetExecutor(control_executor_name_);
    AIMRT_CHECK_ERROR_THROW(cycle_control_executor_, "Failed to get control executor '{}'", control_executor_name_);

    pub_joint_cmd_ = core_.GetChannelHandle().GetPublisher(topic_pub_joint_cmd_);
    AIMRT_CHECK_ERROR_THROW(pub_joint_cmd_, "Failed to get publisher for topic '{}'", topic_pub_joint_cmd_);
    aimrt::channel::RegisterPublishType<JointCommandArray>(pub_joint_cmd_);

    pub_cart_expected_ = core_.GetChannelHandle().GetPublisher(topic_pub_cart_expected_);
    AIMRT_CHECK_ERROR_THROW(pub_cart_expected_, "Failed to get publisher for topic '{}'", topic_pub_cart_expected_);
    aimrt::channel::RegisterPublishType<Pose>(pub_cart_expected_);

    pub_cart_current_ = core_.GetChannelHandle().GetPublisher(topic_pub_cart_current_);
    AIMRT_CHECK_ERROR_THROW(pub_cart_current_, "Failed to get publisher for topic '{}'", topic_pub_cart_current_);
    aimrt::channel::RegisterPublishType<Pose>(pub_cart_current_);

    sub_joint_state_ = core_.GetChannelHandle().GetSubscriber(topic_sub_joint_state_);
    AIMRT_CHECK_ERROR_THROW(sub_joint_state_, "Failed to get subscriber for topic '{}'", topic_sub_joint_state_);
    aimrt::channel::Subscribe<JointStateArray>(
        sub_joint_state_, std::bind(&ControlModule::JointStateCallback, this, std::placeholders::_1));

    // 创建 RPC 服务，传入控制器指针
    move_control_service_ = std::make_shared<MoveControlServiceImpl>(arm_controller_, gripper_controller_);
    core_.GetRpcHandle().RegisterService(move_control_service_.get());

  } catch (const std::exception& e) {
    AIMRT_ERROR("Init failed, {}", e.what());
    return false;
  }

  AIMRT_INFO("Init succeeded.");

  return true;
}

bool ControlModule::Start() {
  try {
    is_running_.store(true);
    cycle_control_executor_.Execute(std::bind(&ControlModule::ControlLoop, this));
    publish_thread_ = std::jthread(&ControlModule::PublishLoop, this);

  } catch (const std::exception& e) {
    AIMRT_ERROR("control_module Start failed, {}", e.what());
    return false;
  }
  AIMRT_INFO("control_module Started succeeded.");
  return true;
}

void ControlModule::Shutdown() {
  try {
    is_running_.store(false);
    stop_sig_.get_future().wait();
    pub_stop_sig_.get_future().wait();
  } catch (const std::exception& e) {
    AIMRT_ERROR("control_module Shutdown failed, {}", e.what());
  }
  AIMRT_INFO("control_module Shutdown succeeded.");
}

void ControlModule::ControlLoop() {
  // 计算周期
  auto period = std::chrono::nanoseconds(static_cast<uint64_t>(1 / control_frequency_ * 1000000000));
  // 记录下一次循环时间
  auto next_loop_time = std::chrono::steady_clock::now();
  // 序列
  uint64_t seq_count = 0;

  JointCommandArray joint_cmd_msg;
  joint_cmd_msg.mutable_joints()->Reserve(7);
  ArmMotion::JointCommand joint_cmd;
  GripperController::GripperCommand gripper_cmd;

  // 周期计时统计
  uint64_t dbg_cycle = 0;
  int64_t cycle_max_us = 0;
  int64_t get_cmd_sum_us = 0, build_msg_sum_us = 0, publish_sum_us = 0;

  while (is_running_.load()) {
    auto t_start = std::chrono::steady_clock::now();

    joint_cmd_msg.Clear();
    auto now = std::chrono::steady_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::nanoseconds>(now.time_since_epoch());
    auto time_ns = duration.count();

    auto* header = joint_cmd_msg.mutable_header();
    header->set_time_stamp(time_ns);
    header->set_frame_id("base_link");
    header->set_sequence_num(seq_count++);

    {
      std::lock_guard<std::mutex> lock(last_arm_cmd_mutex_);
      if (!arm_controller_->GetJointCommand(joint_cmd)) {
        // 无规划轨迹 — 复用上一帧指令保持
        if (last_arm_cmd_.stiffness[0] > 0.0) {
          joint_cmd = last_arm_cmd_;
        } else {
          // 尚无有效历史指令
          joint_cmd.position = ArmMotion::Joint6Dof{0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
          joint_cmd.velocity = ArmMotion::Joint6Dof{0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
          joint_cmd.effort = ArmMotion::Joint6Dof{0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
        }
      } else {
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
        if (last_gripper_cmd_.stiffness > 0.0) {
          gripper_cmd = last_gripper_cmd_;
        } else {
          // 尚无有效历史指令
          gripper_cmd.position = 0.0;
          gripper_cmd.velocity = 0.0;
          gripper_cmd.effort = 0.0;
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

    aimrt::channel::Publish(pub_joint_cmd_, joint_cmd_msg);

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
    now = std::chrono::steady_clock::now();
    if (next_loop_time < now - period) {  // 落后超过 1 个周期
      AIMRT_WARN("Control loop is lagging behind, skip to next cycle.");
      AIMRT_WARN(
          "\n---\nNow cycle: get_cmd={}μs, build_msg={}μs, publish={}μs, total={}μs\n---",
          get_cmd_us, build_msg_us, publish_us, total_us);
      next_loop_time = now + period;      // 放弃追赶，重新对齐
    }
    std::this_thread::sleep_until(next_loop_time);
  }
  stop_sig_.set_value();
}

void ControlModule::PublishLoop() {
  // 计算周期
  auto period = std::chrono::nanoseconds(static_cast<uint64_t>(1 / publish_frequency_ * 1000000000));
  // 记录下一次循环时间
  auto next_loop_time = std::chrono::steady_clock::now();
  // 序列
  uint64_t seq_count = 0;

  Pose expected_pose_msg;
  Pose current_pose_msg;
  ArmMotion::Pose6Dof current_pose;
  ArmMotion::Joint6Dof joint_pos_cmd;
  ArmMotion::Pose6Dof expected_pose;

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

  while (is_running_) {
    // 获取期望和当前笛卡尔位姿
    arm_controller_->GetCurrentPose(current_pose);
    {
      std::lock_guard<std::mutex> lock(last_arm_cmd_mutex_);
      joint_pos_cmd = last_arm_cmd_.position;
    }
    arm_controller_->JointToPose(joint_pos_cmd, expected_pose);

    // 打包msg
    expected_pose_msg.Clear();
    current_pose_msg.Clear();

    auto now = std::chrono::steady_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::nanoseconds>(now.time_since_epoch());
    auto time_ns = duration.count();

    fill_pose_msg(expected_pose_msg, expected_pose, seq_count, time_ns);
    fill_pose_msg(current_pose_msg, current_pose, seq_count, time_ns);

    aimrt::channel::Publish(pub_cart_expected_, expected_pose_msg);
    aimrt::channel::Publish(pub_cart_current_, current_pose_msg);

    next_loop_time += period;
    std::this_thread::sleep_until(next_loop_time);
  }
  pub_stop_sig_.set_value();
}

void ControlModule::JointStateCallback(const std::shared_ptr<const JointStateArray>& msg) {
  if (!is_running_.load()) return;

  ArmMotion::JointState joint_state;
  int n = msg->joints_size();
  // int expected = gripper_controller_ ? 7 : 6;
  // if (n < expected) {
  //   AIMRT_ERROR("关节数量 {} < {}, 检查 {} channel", n, expected, topic_sub_joint_state_);
  //   return;
  // }

  for (int i = 0; i < 6; ++i) {
    joint_state.position[i] = msg->joints(i).position();
    joint_state.velocity[i] = msg->joints(i).velocity();
    joint_state.effort[i] = msg->joints(i).effort();
  }

  // 更新机械臂控制的状态
  arm_controller_->SetJointState(joint_state);
  // 更新夹爪控制器的状态
  if (gripper_controller_) {
    auto joint = msg->joints(6);
    gripper_controller_->SetGripperState(joint.position(), joint.velocity(), joint.effort());
  }
}

}  // namespace robotic_arm::control_module
