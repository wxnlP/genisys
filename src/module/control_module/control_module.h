#pragma once

#include "aimrt_module_cpp_interface/module_base.h"

namespace robotic_arm::control_module {

class ControlModule : public aimrt::ModuleBase {
 public:
  ControlModule() = default;
  ~ControlModule() override = default;

  aimrt::ModuleInfo Info() const override {
    return aimrt::ModuleInfo{.name = "ControlModule"};
  }

  bool Initialize(aimrt::CoreRef core) override;

  bool Start() override;

  void Shutdown() override;

 private:
  auto GetLogger() { return core_.GetLogger(); }

 private:
  aimrt::CoreRef core_;
};

}  // namespace robotic_arm::control_module
