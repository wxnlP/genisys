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

void FillHeader(const aimrt::protocols::common::Header& src, std_msgs::msg::Header& dst) {
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

template <class TBridge, class TConvert>
void PublishLatest(TBridge& bridge, std::mutex& mutex, TConvert convert) {
  std::shared_ptr<const typename TBridge::SrcMsg> msg;
  {
    std::lock_guard<std::mutex> lock(mutex);
    if (!bridge.updated) return;

    bridge.updated = false;
    msg = bridge.latest;
  }

  if (!msg) return;

  bridge.pub.Publish(convert(*msg));
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

    publish_executor_ = ctx_ptr_->CreateExecutor(publish_executor_name_);

    const auto bridges_node = cfg_node["bridges"];
    AIMRT_CHECK_ERROR_THROW(bridges_node && bridges_node.IsSequence(),
                            "ROS2ProxyModule 'bridges' must be a sequence");

    for (const auto& bridge_node : bridges_node) {
      if (!bridge_node["enable"].as<bool>(true)) continue;

      const auto type = bridge_node["type"].as<std::string>();
      const auto sub_topic = bridge_node["sub_topic"].as<std::string>();
      const auto pub_topic = bridge_node["pub_topic"].as<std::string>();

      if (type == "joint_state") {
        AddJointStateBridge(sub_topic, pub_topic);
      } else if (type == "pose_stamped") {
        AddPoseBridge(sub_topic, pub_topic);
      } else {
        AIMRT_CHECK_ERROR_THROW(false, "Unsupported ROS2ProxyModule bridge type '{}'", type);
      }
    }

    AIMRT_CHECK_ERROR_THROW(!joint_state_bridges_.empty() || !pose_bridges_.empty(),
                            "No bridge is enabled in ROS2ProxyModule");

  } catch (const std::exception& e) {
    AIMRT_ERROR("Init failed, {}", e.what());
    return false;
  }

  AIMRT_INFO("Init succeeded, bridging {} joint_state and {} pose topic(s).",
             joint_state_bridges_.size(), pose_bridges_.size());

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

void Ros2ProxyModule::AddJointStateBridge(const std::string& sub_topic, const std::string& pub_topic) {
  auto bridge = std::make_unique<JointStateBridge>();
  auto* bridge_ptr = bridge.get();

  bridge->pub = ctx_ptr_->CreatePublisher<sensor_msgs::msg::JointState>(pub_topic);
  bridge->sub = ctx_ptr_->CreateSubscriber<JointStateArray>(
      sub_topic, [this, bridge_ptr](std::shared_ptr<const JointStateArray> msg) {
        std::lock_guard<std::mutex> lock(mutex_);
        bridge_ptr->latest = std::move(msg);
        bridge_ptr->updated = true;
      });

  joint_state_bridges_.push_back(std::move(bridge));
}

void Ros2ProxyModule::AddPoseBridge(const std::string& sub_topic, const std::string& pub_topic) {
  auto bridge = std::make_unique<PoseBridge>();
  auto* bridge_ptr = bridge.get();

  bridge->pub = ctx_ptr_->CreatePublisher<geometry_msgs::msg::PoseStamped>(pub_topic);
  bridge->sub = ctx_ptr_->CreateSubscriber<PbPoseStamped>(
      sub_topic, [this, bridge_ptr](std::shared_ptr<const PbPoseStamped> msg) {
        std::lock_guard<std::mutex> lock(mutex_);
        bridge_ptr->latest = std::move(msg);
        bridge_ptr->updated = true;
      });

  pose_bridges_.push_back(std::move(bridge));
}

void Ros2ProxyModule::PublishLoop() {
  const auto period = std::chrono::nanoseconds(
      static_cast<int64_t>(1.0 / publish_frequency_ * 1e9));
  auto next_loop_time = std::chrono::steady_clock::now();

  while (ctx_ptr_->Running()) {
    for (auto& bridge : joint_state_bridges_) {
      PublishLatest(*bridge, mutex_, ToRosJointState);
    }
    for (auto& bridge : pose_bridges_) {
      PublishLatest(*bridge, mutex_, ToRosPoseStamped);
    }

    next_loop_time += period;
    std::this_thread::sleep_until(next_loop_time);
  }
}

}  // namespace genisys::ros2_proxy_module
