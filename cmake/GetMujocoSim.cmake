include(FetchContent)

message(STATUS "get aimrt mujoco sim ...")

if(aimrt_mujoco_sim_LOCAL_SOURCE)
  FetchContent_Declare(aimrt_mujoco_sim SOURCE_DIR ${aimrt_mujoco_sim_LOCAL_SOURCE} OVERRIDE_FIND_PACKAGE)
else()
  FetchContent_Declare(
    aimrt_mujoco_sim 
    GIT_REPOSITORY "https://gh-proxy.org/https://github.com/wxnlP/aimrt_mujoco_sim.git"
    GIT_TAG "wxnlP/test_auto_detach"
    DOWNLOAD_EXTRACT_TIMESTAMP TRUE
    OVERRIDE_FIND_PACKAGE)
endif()

function(get_aimrt_mujoco_sim)
  FetchContent_GetProperties(aimrt_mujoco_sim)
  if(NOT aimrt_mujoco_sim_POPULATED)

    set(AIMRT_MUJOCO_SIM_BUILD_WITH_ROS2 ${GENISYS_BUILD_MUJOCO_ROS2})
    set(AIMRT_MUJOCO_SIM_BUILD_WITH_CAMERA ${GENISYS_BUILD_MUJOCO_CAMERA})

    FetchContent_MakeAvailable(aimrt_mujoco_sim)
  endif()
endfunction()

get_aimrt_mujoco_sim()