#include "actuator_module/actuator_module.h"
#include <fmt/ranges.h>
#include <thread>
#include "aimrt_module_protobuf_interface/channel/protobuf_channel.h"
#include "yaml-cpp/yaml.h"

namespace robotic_arm::actuator_module {

bool ActuatorModule::Initialize(aimrt::CoreRef core) {
  // Save aimrt framework handle
  core_ = core;

  try {
    // Read cfg
    auto file_path = core_.GetConfigurator().GetConfigFilePath();
    AIMRT_CHECK_ERROR_THROW(!file_path.empty(), "Config file path is empty");

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

    pub_executor_ = core_.GetExecutorManager().GetExecutor(pub_executor_name_);
    AIMRT_CHECK_ERROR_THROW(pub_executor_, "Failed to get pub executor '{}'", pub_executor_name_);

    write_executor_ = core.GetExecutorManager().GetExecutor(write_executor_name_);
    AIMRT_CHECK_ERROR_THROW(write_executor_, "Failed to get write executor '{}'", write_executor_name_);

    pub_joint_state_ = core_.GetChannelHandle().GetPublisher(topic_pub_joint_state_);
    AIMRT_CHECK_ERROR_THROW(pub_joint_state_, "Failed to get publisher for topic '{}'", topic_pub_joint_state_);
    aimrt::channel::RegisterPublishType<JointStateArray>(pub_joint_state_);

    sub_joint_cmd_ = core.GetChannelHandle().GetSubscriber(topic_sub_joint_cmd_);
    AIMRT_CHECK_ERROR_THROW(sub_joint_cmd_, "Failed to get subscriber for topic '{}'", topic_sub_joint_cmd_);
    aimrt::channel::Subscribe<JointCommandArray>(
        sub_joint_cmd_,
        std::bind(&ActuatorModule::JointCmdCallback, this, std::placeholders::_1));

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

    // 启动发布循环
    is_running_.store(true);
    pub_executor_.Execute(std::bind(&ActuatorModule::PublishLoop, this));
    write_executor_.Execute(std::bind(&ActuatorModule::WriteLoop, this));
  } catch (const std::exception& e) {
    AIMRT_ERROR("Start failed, {}", e.what());
    return false;
  }
  AIMRT_INFO("actuator_module Started succeeded.");
  return true;
}

void ActuatorModule::Shutdown() {
  try {
    // 停止发布循环和 channel 回调
    is_running_.store(false);
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
  // 计算周期
  auto period = std::chrono::nanoseconds(static_cast<uint64_t>(1 / publish_frequecy_ * 1000000000));
  // 记录下一次循环时间
  auto next_loop_time = std::chrono::steady_clock::now();
  // 序列
  uint64_t seq_count_ = 0;
  // 初始化
  JointStateArray joint_state_msg;
  ArmCtrlBase::JointState joint_state;
  GripperCtrlBase::GripperState gripper_state;

  while (is_running_.load()) {
    joint_state_msg.Clear();
    if (!arm_actuator_->GetJointState(joint_state)) {
      AIMRT_ERROR("读取机械臂本体关节状态失败！");
    }
    if (gripper_actuator_) {
      if (!gripper_actuator_->ReceiveState(gripper_state)) {
        AIMRT_ERROR("读取夹爪状态失败！");
      }
    }

    auto now = std::chrono::steady_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::nanoseconds>(now.time_since_epoch());
    auto time_ns = duration.count();

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

    aimrt::channel::Publish(pub_joint_state_, joint_state_msg);

    next_loop_time += period;

    std::this_thread::sleep_until(next_loop_time);
  }
  pub_stop_sig_.set_value();
  AIMRT_INFO("成功退出PublishLoop");
}

void ActuatorModule::WriteLoop() {
  ArmCtrlBase::JointCommand joint_cmd;
  GripperCtrlBase::GripperCommand gripper_cmd;
  while (is_running_.load()) {
    write_sem_.acquire();
    if (!is_running_.load()) {
      break;
    }
    {
      std::lock_guard<std::mutex> lock(cached_cmd_lock);
      joint_cmd = cached_joint_cmd_;
      gripper_cmd = cached_gripper_cmd_;
    }
    arm_actuator_->SetJointState(joint_cmd);
    if (gripper_actuator_) {
      gripper_actuator_->SendCommand(gripper_cmd);
    }
  }
  write_stop_sig_.set_value();
  AIMRT_INFO("成功退出WriteLoop");
}

void ActuatorModule::JointCmdCallback(const std::shared_ptr<const JointCommandArray>& msg) {
  if (!is_running_.load()) return;
  AIMRT_DEBUG("Received joint command message with {} joints.", msg->joints_size());
  int n = msg->joints_size();
  // int expected = gripper_actuator_ ? 7 : 6;
  // if (n < expected) {
  //   AIMRT_ERROR("关节数量 {} < {}, 检查 {} channel", n, expected, topic_sub_joint_cmd_);
  //   return;
  // }

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

}  // namespace robotic_arm::actuator_module
