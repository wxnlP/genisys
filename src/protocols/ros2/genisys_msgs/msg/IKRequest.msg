uint8 FRAME_UNSPECIFIED = 0   # 未指定：服务端按 INVALID_INPUT 处理
uint8 FRAME_TCP = 1           # TCP 位姿
uint8 FRAME_FLANGE = 2        # Flange 位姿

uint64 id                     # 调用方自定义编号，原样回填到 IKResult.id
uint8 frame                   # FRAME_TCP / FRAME_FLANGE
bool do_fk                    # 是否做正解检验
geometry_msgs/Pose pose       # 位置 [m] + 姿态四元数
