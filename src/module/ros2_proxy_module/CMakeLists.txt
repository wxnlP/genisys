# Get the current folder name
string(REGEX REPLACE ".*/\(.*\)" "\\1" CUR_DIR ${CMAKE_CURRENT_SOURCE_DIR})

# Get namespace
get_namespace(CUR_SUPERIOR_NAMESPACE)
string(REPLACE "::" "_" CUR_SUPERIOR_NAMESPACE_UNDERLINE ${CUR_SUPERIOR_NAMESPACE})

# Set target name
set(CUR_TARGET_NAME ${CUR_SUPERIOR_NAMESPACE_UNDERLINE}_${CUR_DIR})
set(CUR_TARGET_ALIAS_NAME ${CUR_SUPERIOR_NAMESPACE}::${CUR_DIR})

# Set file collection
file(GLOB_RECURSE head_files ${CMAKE_CURRENT_SOURCE_DIR}/*.h)
file(GLOB_RECURSE src ${CMAKE_CURRENT_SOURCE_DIR}/*.cc)

find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

# Add target
add_library(${CUR_TARGET_NAME} STATIC)
add_library(${CUR_TARGET_ALIAS_NAME} ALIAS ${CUR_TARGET_NAME})

# Set source file of target
target_sources(${CUR_TARGET_NAME} PRIVATE ${src})

# Set include path of target
target_include_directories(
  ${CUR_TARGET_NAME}
  PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/..>)

# Set link libraries of target
target_link_libraries(
  ${CUR_TARGET_NAME}
  PRIVATE yaml-cpp::yaml-cpp
  PUBLIC aimrt::interface::aimrt_module_cpp_interface
         aimrt::interface::aimrt_module_protobuf_interface
         aimrt::interface::aimrt_module_ros2_interface
         aimrt::protocols::sensor_pb_gencode
         genisys::protocols::genisys_pb_gencode
         sensor_msgs::sensor_msgs__rosidl_typesupport_cpp
         geometry_msgs::geometry_msgs__rosidl_typesupport_cpp)
