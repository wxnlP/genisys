#pragma once

#include <memory>
#include <mutex>
#include <string>

#include "aimrt_module_cpp_interface/context/context.h"
#include "aimrt_module_cpp_interface/module_base.h"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "joint_state.pb.h"
#include "pose.pb.h"
#include "sensor_msgs/msg/joint_state.hpp"

namespace genisys::ros2_proxy_module {

using JointStateArray = aimrt::protocols::sensor::JointStateArray;
using PbPoseStamped = genisys::protocols::PoseStamped;

class Ros2ProxyModule : public aimrt::ModuleBase {
 public:
  Ros2ProxyModule() = default;
  ~Ros2ProxyModule() override = default;

  aimrt::ModuleInfo Info() const override {
    return aimrt::ModuleInfo{.name = "ROS2ProxyModule"};
  }

  bool Initialize(aimrt::CoreRef core) override;

  bool Start() override;

  void Shutdown() override;

 private:
  void PublishLoop();
  void PublishJointState();
  void PublishPose(aimrt::context::res::Publisher<geometry_msgs::msg::PoseStamped>& publisher,
                   std::shared_ptr<const PbPoseStamped>& latest,
                   bool& updated);

  aimrt::CoreRef core_;
  std::shared_ptr<aimrt::context::Context> ctx_ptr_;

  aimrt::context::res::Subscriber<JointStateArray> sub_joint_state_;
  aimrt::context::res::Subscriber<PbPoseStamped> sub_tcp_pose_;
  aimrt::context::res::Subscriber<PbPoseStamped> sub_tcp_pose_expected_;
  aimrt::context::res::Subscriber<PbPoseStamped> sub_flange_pose_;

  aimrt::context::res::Publisher<sensor_msgs::msg::JointState> pub_joint_state_;
  aimrt::context::res::Publisher<geometry_msgs::msg::PoseStamped> pub_tcp_pose_;
  aimrt::context::res::Publisher<geometry_msgs::msg::PoseStamped> pub_tcp_pose_expected_;
  aimrt::context::res::Publisher<geometry_msgs::msg::PoseStamped> pub_flange_pose_;

  aimrt::executor::ExecutorRef publish_executor_;

  std::string publish_executor_name_;
  double publish_frequency_{0.0};

  std::mutex mutex_;
  std::shared_ptr<const JointStateArray> latest_joint_state_;
  bool joint_state_updated_{false};
  std::shared_ptr<const PbPoseStamped> latest_tcp_pose_;
  bool tcp_pose_updated_{false};
  std::shared_ptr<const PbPoseStamped> latest_tcp_pose_expected_;
  bool tcp_pose_expected_updated_{false};
  std::shared_ptr<const PbPoseStamped> latest_flange_pose_;
  bool flange_pose_updated_{false};
};

}  // namespace genisys::ros2_proxy_module
