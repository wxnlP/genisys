#pragma once

#include <memory>
#include <mutex>
#include <string>
#include <vector>

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
  struct JointStateBridge {
    using SrcMsg = JointStateArray;

    aimrt::context::res::Subscriber<JointStateArray> sub;
    aimrt::context::res::Publisher<sensor_msgs::msg::JointState> pub;
    std::shared_ptr<const JointStateArray> latest;
    bool updated{false};
  };

  struct PoseBridge {
    using SrcMsg = PbPoseStamped;

    aimrt::context::res::Subscriber<PbPoseStamped> sub;
    aimrt::context::res::Publisher<geometry_msgs::msg::PoseStamped> pub;
    std::shared_ptr<const PbPoseStamped> latest;
    bool updated{false};
  };

  void AddJointStateBridge(const std::string& sub_topic, const std::string& pub_topic);
  void AddPoseBridge(const std::string& sub_topic, const std::string& pub_topic);

  void PublishLoop();

  aimrt::CoreRef core_;
  std::shared_ptr<aimrt::context::Context> ctx_ptr_;
  aimrt::executor::ExecutorRef publish_executor_;

  std::string publish_executor_name_;
  double publish_frequency_{0.0};

  std::mutex mutex_;
  std::vector<std::unique_ptr<JointStateBridge>> joint_state_bridges_;
  std::vector<std::unique_ptr<PoseBridge>> pose_bridges_;
};

}  // namespace genisys::ros2_proxy_module
