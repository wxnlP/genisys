#pragma once

#include "aimrt_module_cpp_interface/module_base.h"

namespace robotic_arm::sim_module {

class SimModule : public aimrt::ModuleBase {
 public:
  SimModule() = default;
  ~SimModule() override = default;

  aimrt::ModuleInfo Info() const override {
    return aimrt::ModuleInfo{.name = "SimModule"};
  }

  bool Initialize(aimrt::CoreRef core) override;

  bool Start() override;

  void Shutdown() override;

 private:
  auto GetLogger() { return core_.GetLogger(); }

 private:
  aimrt::CoreRef core_;
};

}  // namespace robotic_arm::sim_module
