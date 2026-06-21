# Set namespace
set_root_namespace("robotic_arm")


# protocols
add_subdirectory(protocols/pb_proto)
add_subdirectory(protocols/ros2_proto)

# module
add_subdirectory(module/control_module)
add_subdirectory(module/actuator_module)
add_subdirectory(module/sim_module)

# pkg
add_subdirectory(pkg/arm_control_pkg)

# install
if(CMAKE_SYSTEM_NAME MATCHES "Linux")
  set(CUR_INSTALL_SOURCE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/install/linux)
elseif(CMAKE_SYSTEM_NAME MATCHES "Windows")
  set(CUR_INSTALL_SOURCE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/install/win)
else()
  message(FATAL_ERROR "Unsupport os")
endif()

install(
  DIRECTORY ${CUR_INSTALL_SOURCE_DIR}/bin
  DESTINATION ./
  USE_SOURCE_PERMISSIONS)

# build all
add_custom_target(
  robotic_arm_build_all ALL
  COMMAND ${CMAKE_COMMAND} -E copy_directory ${CUR_INSTALL_SOURCE_DIR}/bin ${CMAKE_BINARY_DIR}
  DEPENDS aimrt::runtime::main
          robotic_arm::arm_control_pkg)


