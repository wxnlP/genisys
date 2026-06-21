#pragma once

#include "aimrt_module_cpp_interface/module_base.h"

namespace robotic_arm::actuator_module {

class ActuatorModule : public aimrt::ModuleBase {
 public:
  ActuatorModule() = default;
  ~ActuatorModule() override = default;

  aimrt::ModuleInfo Info() const override {
    return aimrt::ModuleInfo{.name = "ActuatorModule"};
  }

  bool Initialize(aimrt::CoreRef core) override;

  bool Start() override;

  void Shutdown() override;

 private:
  auto GetLogger() { return core_.GetLogger(); }

 private:
  aimrt::CoreRef core_;
};

}  // namespace robotic_arm::actuator_module
