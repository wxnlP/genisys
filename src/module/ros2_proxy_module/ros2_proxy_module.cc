#include "ros2_proxy_module/ros2_proxy_module.h"

#include <chrono>
#include <cstddef>
#include <cstdint>
#include <thread>
#include <utility>

#include "aimrt_module_protobuf_interface/channel/protobuf_channel.h"
#include "aimrt_module_ros2_interface/channel/ros2_channel.h"
#include "std_msgs/msg/header.hpp"
#include "yaml-cpp/yaml.h"

namespace genisys::ros2_proxy_module {
namespace {

constexpr uint64_t kNanoSecondsPerSecond = 1000000000ULL;

void FillHeader(const aimrt::protocols::common::Header& src,
                std_msgs::msg::Header& dst) {
  dst.stamp.sec = static_cast<int32_t>(src.time_stamp() / kNanoSecondsPerSecond);
  dst.stamp.nanosec = static_cast<uint32_t>(src.time_stamp() % kNanoSecondsPerSecond);
  dst.frame_id = src.frame_id();
}

sensor_msgs::msg::JointState ToRosJointState(const JointStateArray& src) {
  sensor_msgs::msg::JointState dst;
  FillHeader(src.header(), dst.header);

  const auto joint_num = static_cast<std::size_t>(src.joints_size());
  dst.name.reserve(joint_num);
  dst.position.reserve(joint_num);
  dst.velocity.reserve(joint_num);
  dst.effort.reserve(joint_num);

  for (const auto& joint : src.joints()) {
    dst.name.push_back(joint.name());
    dst.position.push_back(joint.position());
    dst.velocity.push_back(joint.velocity());
    dst.effort.push_back(joint.effort());
  }

  return dst;
}

geometry_msgs::msg::PoseStamped ToRosPoseStamped(const PbPoseStamped& src) {
  geometry_msgs::msg::PoseStamped dst;
  FillHeader(src.header(), dst.header);

  dst.pose.position.x = src.pose().position().x();
  dst.pose.position.y = src.pose().position().y();
  dst.pose.position.z = src.pose().position().z();

  dst.pose.orientation.x = src.pose().orientation().x();
  dst.pose.orientation.y = src.pose().orientation().y();
  dst.pose.orientation.z = src.pose().orientation().z();
  dst.pose.orientation.w = src.pose().orientation().w();

  return dst;
}

}  // namespace

bool Ros2ProxyModule::Initialize(aimrt::CoreRef core) {
  core_ = core;
  ctx_ptr_ = std::make_shared<aimrt::context::Context>(core_);
  ctx_ptr_->LetMe();

  try {
    const auto config_file_path = ctx_ptr_->GetConfigFilePath();
    AIMRT_CHECK_ERROR_THROW(!config_file_path.empty(), "ROS2ProxyModule config file path is empty");

    YAML::Node cfg_node = YAML::LoadFile(std::string(config_file_path));

    publish_executor_name_ = cfg_node["publish_executor_name"].as<std::string>();
    publish_frequency_ = cfg_node["publish_frequency"].as<double>();
    AIMRT_CHECK_ERROR_THROW(publish_frequency_ > 0.0, "publish_frequency must be positive");

    const auto topic_sub_joint_state = cfg_node["topic_sub_joint_state"].as<std::string>();
    const auto topic_sub_tcp_pose = cfg_node["topic_sub_tcp_pose"].as<std::string>();
    const auto topic_sub_tcp_pose_expected = cfg_node["topic_sub_tcp_pose_expected"].as<std::string>();
    const auto topic_sub_flange_pose = cfg_node["topic_sub_flange_pose"].as<std::string>();
    const auto topic_pub_joint_state = cfg_node["topic_pub_joint_state"].as<std::string>();
    const auto topic_pub_tcp_pose = cfg_node["topic_pub_tcp_pose"].as<std::string>();
    const auto topic_pub_tcp_pose_expected = cfg_node["topic_pub_tcp_pose_expected"].as<std::string>();
    const auto topic_pub_flange_pose = cfg_node["topic_pub_flange_pose"].as<std::string>();

    publish_executor_ = ctx_ptr_->CreateExecutor(publish_executor_name_);

    sub_joint_state_ = ctx_ptr_->CreateSubscriber<JointStateArray>(
        topic_sub_joint_state, [this](std::shared_ptr<const JointStateArray> msg) {
          std::lock_guard<std::mutex> lock(mutex_);
          latest_joint_state_ = std::move(msg);
          joint_state_updated_ = true;
        });

    sub_tcp_pose_ = ctx_ptr_->CreateSubscriber<PbPoseStamped>(
        topic_sub_tcp_pose, [this](std::shared_ptr<const PbPoseStamped> msg) {
          std::lock_guard<std::mutex> lock(mutex_);
          latest_tcp_pose_ = std::move(msg);
          tcp_pose_updated_ = true;
        });

    sub_tcp_pose_expected_ = ctx_ptr_->CreateSubscriber<PbPoseStamped>(
        topic_sub_tcp_pose_expected, [this](std::shared_ptr<const PbPoseStamped> msg) {
          std::lock_guard<std::mutex> lock(mutex_);
          latest_tcp_pose_expected_ = std::move(msg);
          tcp_pose_expected_updated_ = true;
        });

    sub_flange_pose_ = ctx_ptr_->CreateSubscriber<PbPoseStamped>(
        topic_sub_flange_pose, [this](std::shared_ptr<const PbPoseStamped> msg) {
          std::lock_guard<std::mutex> lock(mutex_);
          latest_flange_pose_ = std::move(msg);
          flange_pose_updated_ = true;
        });

    pub_joint_state_ = ctx_ptr_->CreatePublisher<sensor_msgs::msg::JointState>(topic_pub_joint_state);
    pub_tcp_pose_ = ctx_ptr_->CreatePublisher<geometry_msgs::msg::PoseStamped>(topic_pub_tcp_pose);
    pub_tcp_pose_expected_ = ctx_ptr_->CreatePublisher<geometry_msgs::msg::PoseStamped>(topic_pub_tcp_pose_expected);
    pub_flange_pose_ = ctx_ptr_->CreatePublisher<geometry_msgs::msg::PoseStamped>(topic_pub_flange_pose);

  } catch (const std::exception& e) {
    AIMRT_ERROR("Init failed, {}", e.what());
    return false;
  }

  AIMRT_INFO("Init succeeded.");

  return true;
}

bool Ros2ProxyModule::Start() {
  try {
    publish_executor_.Execute([this]() {
      ctx_ptr_->LetMe();
      PublishLoop();
    });
  } catch (const std::exception& e) {
    AIMRT_ERROR("Start failed, {}", e.what());
    return false;
  }

  AIMRT_INFO("Start succeeded.");

  return true;
}

void Ros2ProxyModule::Shutdown() {
  ctx_ptr_->StopRunning();
  AIMRT_INFO("Shutdown succeeded.");
}

void Ros2ProxyModule::PublishLoop() {
  const auto period = std::chrono::nanoseconds(
      static_cast<int64_t>(1.0 / publish_frequency_ * 1e9));
  auto next_loop_time = std::chrono::steady_clock::now();

  while (ctx_ptr_->Running()) {
    PublishJointState();
    PublishPose(pub_tcp_pose_, latest_tcp_pose_, tcp_pose_updated_);
    PublishPose(pub_tcp_pose_expected_, latest_tcp_pose_expected_, tcp_pose_expected_updated_);
    PublishPose(pub_flange_pose_, latest_flange_pose_, flange_pose_updated_);

    next_loop_time += period;
    std::this_thread::sleep_until(next_loop_time);
  }
}

void Ros2ProxyModule::PublishJointState() {
  std::shared_ptr<const JointStateArray> msg;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!joint_state_updated_) return;

    joint_state_updated_ = false;
    msg = latest_joint_state_;
  }

  if (!msg) return;

  pub_joint_state_.Publish(ToRosJointState(*msg));
}

void Ros2ProxyModule::PublishPose(
    aimrt::context::res::Publisher<geometry_msgs::msg::PoseStamped>& publisher,
    std::shared_ptr<const PbPoseStamped>& latest,
    bool& updated) {
  std::shared_ptr<const PbPoseStamped> msg;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!updated) return;

    updated = false;
    msg = latest;
  }

  if (!msg) return;

  publisher.Publish(ToRosPoseStamped(*msg));
}

}  // namespace genisys::ros2_proxy_module
