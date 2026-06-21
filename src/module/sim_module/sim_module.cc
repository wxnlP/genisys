#include "sim_module/sim_module.h"

#include "yaml-cpp/yaml.h"

namespace robotic_arm::sim_module {

bool SimModule::Initialize(aimrt::CoreRef core) {
  // Save aimrt framework handle
  core_ = core;

  try {
    // Read cfg
    auto file_path = core_.GetConfigurator().GetConfigFilePath();
    if (!file_path.empty()) {
      YAML::Node cfg_node = YAML::LoadFile(std::string(file_path));
      for (const auto& itr : cfg_node) {
        std::string k = itr.first.as<std::string>();
        std::string v = itr.second.as<std::string>();
        AIMRT_INFO("cfg [{} : {}]", k, v);
      }
    }

  } catch (const std::exception& e) {
    AIMRT_ERROR("Init failed, {}", e.what());
    return false;
  }

  AIMRT_INFO("Init succeeded.");

  return true;
}

bool SimModule::Start() {
  AIMRT_INFO("sim_module Started succeeded.");
  return true;
}

void SimModule::Shutdown() {
  AIMRT_INFO("sim_module Shutdown succeeded.");
}

}  // namespace robotic_arm::sim_module
