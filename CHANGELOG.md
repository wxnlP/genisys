# 更新日志

## 2026-11-23

### 新增功能

- aimrt_mujoco_sim 模块改用自开发分支，仿真新增 RGB-D 相机支持，可发布彩色图、深度图和相机内参，支持 Protobuf 和 ROS 2 两种消息类型；
- mjcf 和 cfg 新增相机支持；
- 重构 CMake Option，所有底层 Options 均通过顶层 `GENISYS Option` 管理；
- 增加更新日志 ` CHANGELOG.md`。

### 问题修复

- ros2 配置文件中`/genisys/task_context`修正为 ros2 后端。
