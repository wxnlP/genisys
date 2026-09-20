cmake_minimum_required(VERSION 3.24)

project(genisys LANGUAGES C CXX)

# Set cmake path
list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake)


# Some necessary settings
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

set(INSTALL_CONFIG_NAME ${PROJECT_NAME}-config)

# Master project configuration
if(CMAKE_CURRENT_SOURCE_DIR STREQUAL CMAKE_SOURCE_DIR)
  set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR})
  set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR})
  set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR})

  set(CMAKE_C_VISIBILITY_PRESET hidden)
  set(CMAKE_C_VISIBILITY_INLINES_HIDDEN ON)
  set(CMAKE_CXX_VISIBILITY_PRESET hidden)
  set(CMAKE_CXX_VISIBILITY_INLINES_HIDDEN ON)
  set(BUILD_SHARED_LIBS OFF)

  if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
    if(GENISYS_BUILD_TESTS)
      set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} --coverage")
      set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} --coverage")
    endif()
  endif()

  set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
endif()

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fPIC")
  set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -fPIC")
elseif(CMAKE_CXX_COMPILER_ID MATCHES "MSVC")
  add_compile_options(/utf-8 /wd4819)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /Zc:__cplusplus")
endif()

if(WIN32)
  add_compile_definitions(NOMINMAX)
endif()

option(GENISYS_BUILD_MUJOCO_SIM "Build AimRT Mujoco Sim" OFF)
option(GENISYS_BUILD_SDK_EXAMPLES "Build genisys-core examples" OFF)

# Include cmake module
include(NamespaceTool)

if(GENISYS_BUILD_TESTS)
  include(GetGTest)
  enable_testing()
endif()

set(YAML_CPP_INSTALL ON)
include(GetAimRT)

if(GENISYS_BUILD_MUJOCO_SIM)
  include(GetMujocoSim)
endif()

# find_package(rclcpp REQUIRED)

# Add subdirectory
add_subdirectory(src)

# Install
install(
  DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/cmake
  DESTINATION ./
  FILES_MATCHING
  PATTERN "*.cmake")
install(EXPORT ${INSTALL_CONFIG_NAME} DESTINATION lib/cmake/${PROJECT_NAME})
