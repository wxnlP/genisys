#include <cstring>

#include "actuator_module/actuator_module.h"
#include "aimrt_pkg_c_interface/pkg_macro.h"
#include "control_module/control_module.h"

static std::tuple<std::string_view, std::function<aimrt::ModuleBase*()>>
    aimrt_module_register_array[]{
        {"ControlModule",
         []() -> aimrt::ModuleBase* {
           return new robotic_arm::control_module::ControlModule();
         }},
        {"ActuatorModule",
         []() -> aimrt::ModuleBase* {
           return new robotic_arm::actuator_module::ActuatorModule();
         }},
    };

AIMRT_PKG_MAIN(aimrt_module_register_array)
