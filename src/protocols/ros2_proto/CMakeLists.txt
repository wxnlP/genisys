cmake_minimum_required(VERSION 3.24)

# Get the current folder name
string(REGEX REPLACE ".*/\(.*\)" "\\1" CUR_DIR ${CMAKE_CURRENT_SOURCE_DIR})

# Get namespace
get_namespace(CUR_SUPERIOR_NAMESPACE)
string(REPLACE "::" "_" CUR_SUPERIOR_NAMESPACE_UNDERLINE ${CUR_SUPERIOR_NAMESPACE})

# Set target name
set(CUR_TARGET_NAME ${CUR_DIR})
set(CUR_TARGET_ALIAS_NAME ${CUR_SUPERIOR_NAMESPACE}::${CUR_DIR})

project(${CUR_TARGET_NAME})

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
# find_package(rclcpp REQUIRED)

# BUG of ros2
set(CUR_BUILD_SHARED_LIBS ${BUILD_SHARED_LIBS})
set(BUILD_SHARED_LIBS ON)
# cmake-format: off
rosidl_generate_interfaces(${CUR_TARGET_NAME}
  "msg/RosChangeMe.msg"
  "msg/Joint.msg"
  "msg/MoveResponse.msg"
  "msg/ServoJCommand.msg"
  "msg/ServoJPoseCommand.msg"
  "msg/ServoLCommand.msg"
  "srv/MoveJ.srv"
  "srv/MoveJPose.srv"
  "srv/MoveJMulti.srv"
  "srv/MoveJPoseMulti.srv"
  "srv/MoveL.srv"
  "srv/MoveLMulti.srv"
  "srv/Gripper.srv"
  DEPENDENCIES std_msgs geometry_msgs)
# cmake-format: on
set(BUILD_SHARED_LIBS ${CUR_BUILD_SHARED_LIBS})

if(NOT TARGET ${CUR_TARGET_NAME}::${CUR_TARGET_NAME}__rosidl_typesupport_cpp)
  add_library(${CUR_TARGET_NAME}::${CUR_TARGET_NAME}__rosidl_typesupport_cpp ALIAS ${CUR_TARGET_NAME}__rosidl_typesupport_cpp)
endif()

if(NOT TARGET ${CUR_TARGET_NAME}::${CUR_TARGET_NAME}__rosidl_typesupport_fastrtps_cpp)
  add_library(${CUR_TARGET_NAME}::${CUR_TARGET_NAME}__rosidl_typesupport_fastrtps_cpp ALIAS ${CUR_TARGET_NAME}__rosidl_typesupport_fastrtps_cpp)
endif()

ament_export_dependencies(rosidl_default_runtime)
ament_package()

# aimrt rpc gencode
add_ros2_aimrt_rpc_gencode_target_for_one_file(
  TARGET_NAME ${CUR_TARGET_NAME}_move_rpc_gencode
  PACKAGE_NAME ${CUR_TARGET_NAME}
  PROTO_FILE ${CMAKE_CURRENT_SOURCE_DIR}/srv/MoveJ.srv
  GENCODE_PATH ${CMAKE_CURRENT_BINARY_DIR}
  DEP_PROTO_TARGETS
    ${CUR_TARGET_NAME}::${CUR_TARGET_NAME}__rosidl_generator_cpp
    ${CUR_TARGET_NAME}::${CUR_TARGET_NAME}__rosidl_typesupport_cpp
    ${CUR_TARGET_NAME}::${CUR_TARGET_NAME}__rosidl_typesupport_fastrtps_cpp
    ${CUR_TARGET_NAME}::${CUR_TARGET_NAME}__rosidl_typesupport_introspection_cpp)
add_library(${CUR_TARGET_ALIAS_NAME}_move_rpc_gencode ALIAS ${CUR_TARGET_NAME}_move_rpc_gencode)

add_ros2_aimrt_rpc_gencode_target_for_one_file(
  TARGET_NAME ${CUR_TARGET_NAME}_move_jpose_rpc_gencode
  PACKAGE_NAME ${CUR_TARGET_NAME}
  PROTO_FILE ${CMAKE_CURRENT_SOURCE_DIR}/srv/MoveJPose.srv
  GENCODE_PATH ${CMAKE_CURRENT_BINARY_DIR}
  DEP_PROTO_TARGETS
    ${CUR_TARGET_NAME}::${CUR_TARGET_NAME}__rosidl_generator_cpp
    ${CUR_TARGET_NAME}::${CUR_TARGET_NAME}__rosidl_typesupport_cpp
    ${CUR_TARGET_NAME}::${CUR_TARGET_NAME}__rosidl_typesupport_fastrtps_cpp
    ${CUR_TARGET_NAME}::${CUR_TARGET_NAME}__rosidl_typesupport_introspection_cpp)
add_library(${CUR_TARGET_ALIAS_NAME}_move_jpose_rpc_gencode ALIAS ${CUR_TARGET_NAME}_move_jpose_rpc_gencode)

add_ros2_aimrt_rpc_gencode_target_for_one_file(
  TARGET_NAME ${CUR_TARGET_NAME}_move_jmulti_rpc_gencode
  PACKAGE_NAME ${CUR_TARGET_NAME}
  PROTO_FILE ${CMAKE_CURRENT_SOURCE_DIR}/srv/MoveJMulti.srv
  GENCODE_PATH ${CMAKE_CURRENT_BINARY_DIR}
  DEP_PROTO_TARGETS
    ${CUR_TARGET_NAME}::${CUR_TARGET_NAME}__rosidl_generator_cpp
    ${CUR_TARGET_NAME}::${CUR_TARGET_NAME}__rosidl_typesupport_cpp
    ${CUR_TARGET_NAME}::${CUR_TARGET_NAME}__rosidl_typesupport_fastrtps_cpp
    ${CUR_TARGET_NAME}::${CUR_TARGET_NAME}__rosidl_typesupport_introspection_cpp)
add_library(${CUR_TARGET_ALIAS_NAME}_move_jmulti_rpc_gencode ALIAS ${CUR_TARGET_NAME}_move_jmulti_rpc_gencode)

add_ros2_aimrt_rpc_gencode_target_for_one_file(
  TARGET_NAME ${CUR_TARGET_NAME}_move_jposemulti_rpc_gencode
  PACKAGE_NAME ${CUR_TARGET_NAME}
  PROTO_FILE ${CMAKE_CURRENT_SOURCE_DIR}/srv/MoveJPoseMulti.srv
  GENCODE_PATH ${CMAKE_CURRENT_BINARY_DIR}
  DEP_PROTO_TARGETS
    ${CUR_TARGET_NAME}::${CUR_TARGET_NAME}__rosidl_generator_cpp
    ${CUR_TARGET_NAME}::${CUR_TARGET_NAME}__rosidl_typesupport_cpp
    ${CUR_TARGET_NAME}::${CUR_TARGET_NAME}__rosidl_typesupport_fastrtps_cpp
    ${CUR_TARGET_NAME}::${CUR_TARGET_NAME}__rosidl_typesupport_introspection_cpp)
add_library(${CUR_TARGET_ALIAS_NAME}_move_jposemulti_rpc_gencode ALIAS ${CUR_TARGET_NAME}_move_jposemulti_rpc_gencode)

add_ros2_aimrt_rpc_gencode_target_for_one_file(
  TARGET_NAME ${CUR_TARGET_NAME}_move_l_rpc_gencode
  PACKAGE_NAME ${CUR_TARGET_NAME}
  PROTO_FILE ${CMAKE_CURRENT_SOURCE_DIR}/srv/MoveL.srv
  GENCODE_PATH ${CMAKE_CURRENT_BINARY_DIR}
  DEP_PROTO_TARGETS
    ${CUR_TARGET_NAME}::${CUR_TARGET_NAME}__rosidl_generator_cpp
    ${CUR_TARGET_NAME}::${CUR_TARGET_NAME}__rosidl_typesupport_cpp
    ${CUR_TARGET_NAME}::${CUR_TARGET_NAME}__rosidl_typesupport_fastrtps_cpp
    ${CUR_TARGET_NAME}::${CUR_TARGET_NAME}__rosidl_typesupport_introspection_cpp)
add_library(${CUR_TARGET_ALIAS_NAME}_move_l_rpc_gencode ALIAS ${CUR_TARGET_NAME}_move_l_rpc_gencode)

add_ros2_aimrt_rpc_gencode_target_for_one_file(
  TARGET_NAME ${CUR_TARGET_NAME}_move_lmulti_rpc_gencode
  PACKAGE_NAME ${CUR_TARGET_NAME}
  PROTO_FILE ${CMAKE_CURRENT_SOURCE_DIR}/srv/MoveLMulti.srv
  GENCODE_PATH ${CMAKE_CURRENT_BINARY_DIR}
  DEP_PROTO_TARGETS
    ${CUR_TARGET_NAME}::${CUR_TARGET_NAME}__rosidl_generator_cpp
    ${CUR_TARGET_NAME}::${CUR_TARGET_NAME}__rosidl_typesupport_cpp
    ${CUR_TARGET_NAME}::${CUR_TARGET_NAME}__rosidl_typesupport_fastrtps_cpp
    ${CUR_TARGET_NAME}::${CUR_TARGET_NAME}__rosidl_typesupport_introspection_cpp)
add_library(${CUR_TARGET_ALIAS_NAME}_move_lmulti_rpc_gencode ALIAS ${CUR_TARGET_NAME}_move_lmulti_rpc_gencode)

add_ros2_aimrt_rpc_gencode_target_for_one_file(
  TARGET_NAME ${CUR_TARGET_NAME}_gripper_rpc_gencode
  PACKAGE_NAME ${CUR_TARGET_NAME}
  PROTO_FILE ${CMAKE_CURRENT_SOURCE_DIR}/srv/Gripper.srv
  GENCODE_PATH ${CMAKE_CURRENT_BINARY_DIR}
  DEP_PROTO_TARGETS
    ${CUR_TARGET_NAME}::${CUR_TARGET_NAME}__rosidl_generator_cpp
    ${CUR_TARGET_NAME}::${CUR_TARGET_NAME}__rosidl_typesupport_cpp
    ${CUR_TARGET_NAME}::${CUR_TARGET_NAME}__rosidl_typesupport_fastrtps_cpp
    ${CUR_TARGET_NAME}::${CUR_TARGET_NAME}__rosidl_typesupport_introspection_cpp)
add_library(${CUR_TARGET_ALIAS_NAME}_gripper_rpc_gencode ALIAS ${CUR_TARGET_NAME}_gripper_rpc_gencode)

add_library(${CUR_TARGET_NAME}_rpc_gencode INTERFACE)
target_link_libraries(
  ${CUR_TARGET_NAME}_rpc_gencode
  INTERFACE
    ${CUR_TARGET_NAME}_move_rpc_gencode
    ${CUR_TARGET_NAME}_move_jpose_rpc_gencode
    ${CUR_TARGET_NAME}_move_jmulti_rpc_gencode
    ${CUR_TARGET_NAME}_move_jposemulti_rpc_gencode
    ${CUR_TARGET_NAME}_move_l_rpc_gencode
    ${CUR_TARGET_NAME}_move_lmulti_rpc_gencode
    ${CUR_TARGET_NAME}_gripper_rpc_gencode)
add_library(${CUR_TARGET_ALIAS_NAME}_rpc_gencode ALIAS ${CUR_TARGET_NAME}_rpc_gencode)
