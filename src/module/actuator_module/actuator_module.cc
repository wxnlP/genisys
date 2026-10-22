#include "actuator_module/actuator_module.h"
#include <fmt/ranges.h>  // 容器元素格式化
#include <thread>
#include "aimrt_module_protobuf_interface/channel/protobuf_channel.h"
#include "common/warn_throttle.h"
#include "util/time_util.h"
#include "yaml-cpp/yaml.h"

namespace genisys::actuator_module {

// 状态读取失败的告警节流周期
constexpr uint32_t kWarnInterval = 500;

bool ActuatorModule::Initialize(aimrt::CoreRef core) {
  // Save aimrt framework handle
  core_ = core;
  ctx_ptr_ = std::make_shared<aimrt::context::Context>(core_);
  ctx_ptr_->LetMe();

  try {
    // Read cfg
    auto file_path = ctx_ptr_->GetConfigFilePath();
    if (!file_path.empty()) {
      YAML::Node cfg_node = YAML::LoadFile(std::string(file_path));
      auto actuator_cfg_file = cfg_node["config_file"].as<std::string>();
      topic_sub_joint_cmd_ = cfg_node["topic_sub_joint_cmd"].as<std::string>();
      topic_pub_joint_state_ = cfg_node["topic_pub_joint_state"].as<std::string>();
      pub_executor_name_ = cfg_node["pub_executor_name"].as<std::string>();
      write_executor_name_ = cfg_node["write_executor_name"].as<std::string>();
      publish_frequecy_ = cfg_node["pub_frequency"].as<double>();

      actuator_manager_ = std::make_shared<ActuatorManager>(actuator_cfg_file);
      AIMRT_CHECK_ERROR_THROW(actuator_manager_, "Failed to create ActuatorManager");

      arm_actuator_ = actuator_manager_->GetArmActuator();
      AIMRT_CHECK_ERROR_THROW(arm_actuator_, "Failed to get ArmActuator");

      gripper_actuator_ = actuator_manager_->GetGripperActuator();

      actuator_manager_->GetJointNames(joint_names_);
      AIMRT_INFO("\n---\n执行器关节名称：{}\n---", joint_names_);
    }

    pub_executor_ = ctx_ptr_->CreateExecutor(pub_executor_name_);

    write_executor_ = ctx_ptr_->CreateExecutor(write_executor_name_);

    pub_joint_state_ = ctx_ptr_->CreatePublisher<JointStateArray>(topic_pub_joint_state_);

    sub_joint_cmd_ = ctx_ptr_->CreateSubscriber<JointCommandArray>(topic_sub_joint_cmd_, std::bind(&ActuatorModule::JointCmdCallback, this, std::placeholders::_1));

  } catch (const std::exception& e) {
    AIMRT_ERROR("Init failed, {}", e.what());
    return false;
  }

  AIMRT_INFO("Init succeeded.");

  return true;
}

bool ActuatorModule::Start() {
  try {
    // 使能各执行器
    if (arm_actuator_) {
      arm_actuator_->Enable();
    }
    if (gripper_actuator_) {
      gripper_actuator_->Enable();
    }
    
    pub_executor_.Execute([this] () {
      ctx_ptr_->LetMe();
      PublishLoop();
    });
    write_executor_.Execute([this] () {
      ctx_ptr_->LetMe();
      WriteLoop();
    });
  } catch (const std::exception& e) {
    AIMRT_ERROR("Start failed, {}", e.what());
    return false;
  }
  AIMRT_INFO("actuator_module Started succeeded.");
  return true;
}

void ActuatorModule::Shutdown() {
  try {
    ctx_ptr_->StopRunning();
    write_sem_.release();

    pub_stop_sig_.get_future().wait();
    write_stop_sig_.get_future().wait();

    // 失能各执行器
    if (arm_actuator_) {
      arm_actuator_->Disable();
    }
    if (gripper_actuator_) {
      gripper_actuator_->Disable();
    }

    // 关闭内部线程
    if (actuator_manager_) {
      actuator_manager_->Shutdown();
    }
  } catch (const std::exception& e) {
    AIMRT_ERROR("Disable actuators failed, {}", e.what());
  }
  AIMRT_INFO("actuator_module Shutdown succeeded.");
}

void ActuatorModule::PublishLoop() {
  try {
    // 计算周期
    auto period = std::chrono::nanoseconds(static_cast<uint64_t>(1 / publish_frequecy_ * 1000000000));
    // 记录下一次循环时间
    auto next_loop_time = std::chrono::steady_clock::now();
    // 序列
    uint64_t seq_count_ = 0;
    // 初始化
    JointStateArray joint_state_msg;
    joint_state_msg.mutable_joints()->Reserve(7);
    ArmCtrlBase::JointState joint_state;
    GripperCtrlBase::GripperState gripper_state;
    // 告警节流
    WarnThrottle arm_state_warn;
    WarnThrottle gripper_state_warn;
    
    while (ctx_ptr_->Running()) {
      joint_state_msg.Clear();
      if (!arm_actuator_->GetJointState(joint_state)) {
        if (arm_state_warn.ShouldLog(kWarnInterval)) {
          AIMRT_WARN("读取机械臂本体关节状态失败（累计 {} 次）", arm_state_warn.count());
        }
        next_loop_time += period;
        std::this_thread::sleep_until(next_loop_time);
        continue;
      }
      arm_state_warn.Reset();
      if (gripper_actuator_) {
        if (!gripper_actuator_->ReceiveState(gripper_state)) {
          if (gripper_state_warn.ShouldLog(kWarnInterval)) {
            AIMRT_WARN("读取夹爪状态失败（累计 {} 次）", gripper_state_warn.count());
          }
          next_loop_time += period;
          std::this_thread::sleep_until(next_loop_time);
          continue;
        }
        gripper_state_warn.Reset();
      }

      // header 时间戳统一为 system_clock
      auto time_ns = aimrt::common::util::GetCurTimestampNs();

      auto* header = joint_state_msg.mutable_header();
      header->set_time_stamp(time_ns);
      header->set_frame_id("base_link");
      header->set_sequence_num(seq_count_++);

      for (int i = 0; i < 6; ++i) {
        auto* joint = joint_state_msg.add_joints();
        joint->set_name(joint_names_[i]);
        joint->set_position(joint_state.position[i]);
        joint->set_velocity(joint_state.velocity[i]);
        joint->set_effort(joint_state.effort[i]);
      }

      if (gripper_actuator_) {
        auto* gripper_joint = joint_state_msg.add_joints();
        gripper_joint->set_name(joint_names_.back());
        gripper_joint->set_position(gripper_state.position);
        gripper_joint->set_velocity(gripper_state.velocity);
        gripper_joint->set_effort(gripper_state.effort);
      }

      ctx_ptr_->LetMe();
      pub_joint_state_.Publish(joint_state_msg);

      next_loop_time += period;

      std::this_thread::sleep_until(next_loop_time);
    }
  } catch (const std::exception& e) {
    AIMRT_ERROR("ActuatorModule::PublishLoop crashed: {}", e.what());
  }
  pub_stop_sig_.set_value();
  AIMRT_INFO("成功退出 ActuatorModule::PublishLoop");
}

void ActuatorModule::WriteLoop() {
  try {
    ArmCtrlBase::JointCommand joint_cmd;
    GripperCtrlBase::GripperCommand gripper_cmd;
    WarnThrottle arm_tx_warn;
    WarnThrottle gripper_tx_warn;
    while (ctx_ptr_->Running()) {
      if (!write_sem_.try_acquire_for(std::chrono::milliseconds(100))) {
        continue;
      }
      if (!ctx_ptr_->Running()) {
        break;
      }
      {
        std::lock_guard<std::mutex> lock(cached_cmd_lock);
        joint_cmd = cached_joint_cmd_;
        gripper_cmd = cached_gripper_cmd_;
      }
      if (!arm_actuator_->SetJointState(joint_cmd)) {
        if (arm_tx_warn.ShouldLog(kWarnInterval)) {
          AIMRT_WARN("关节指令下发失败（累计 {} 次）", arm_tx_warn.count());
        }
      } else {
        arm_tx_warn.Reset();
      }
      if (gripper_actuator_) {
        if (!gripper_actuator_->SendCommand(gripper_cmd)) {
          if (gripper_tx_warn.ShouldLog(kWarnInterval)) {
            AIMRT_WARN("夹爪指令下发失败（累计 {} 次）", gripper_tx_warn.count());
          }
        } else {
          gripper_tx_warn.Reset();
        }
      }
    }
  } catch (const std::exception& e) {
    AIMRT_ERROR("ActuatorModule::WriteLoop crashed: {}", e.what());
  }
  write_stop_sig_.set_value();
  AIMRT_INFO("成功退出 ActuatorModule::WriteLoop");
}

void ActuatorModule::JointCmdCallback(const std::shared_ptr<const JointCommandArray>& msg) {
  if (!ctx_ptr_->Running()) {
    return;
  }
  AIMRT_DEBUG("Received joint command message with {} joints.", msg->joints_size());
  int n = msg->joints_size();
  if (n < 6) {
    AIMRT_WARN("joint_cmd 关节数 {} < 6，丢弃", n);
    return;
  }

  {
    std::lock_guard<std::mutex> lock(cached_cmd_lock);

    for (int i = 0; i < 6; ++i) {
      cached_joint_cmd_.position[i] = msg->joints(i).position();
      cached_joint_cmd_.velocity[i] = msg->joints(i).velocity();
      cached_joint_cmd_.effort[i] = msg->joints(i).effort();
      cached_joint_cmd_.stiffness[i] = msg->joints(i).stiffness();
      cached_joint_cmd_.damping[i] = msg->joints(i).damping();
    }

    if (gripper_actuator_ && n > 6) {
      cached_gripper_cmd_.position = msg->joints(6).position();
      cached_gripper_cmd_.velocity = msg->joints(6).velocity();
      cached_gripper_cmd_.effort = msg->joints(6).effort();
      cached_gripper_cmd_.stiffness = msg->joints(6).stiffness();
      cached_gripper_cmd_.damping = msg->joints(6).damping();
    }
  }

  write_sem_.release();
}

}  // namespace genisys::actuator_module
