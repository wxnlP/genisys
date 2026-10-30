#include "aimrt_pkg_c_interface/pkg_macro.h"
#include "ros2_proxy_module/ros2_proxy_module.h"

static std::tuple<std::string_view, std::function<aimrt::ModuleBase*()>>
    aimrt_module_register_array[]{
        {"ROS2ProxyModule",
         []() -> aimrt::ModuleBase* {
           return new genisys::ros2_proxy_module::Ros2ProxyModule();
         }},
    };

AIMRT_PKG_MAIN(aimrt_module_register_array)
