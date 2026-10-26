uint8 OK = 0
uint8 NO_SOLUTION = 1         # 无解
uint8 INVALID_INPUT = 2       # 位姿非法 或 frame 未指定
uint8 FK_VERIFY_WARN = 3      # FK 验证容差不过
uint8 OTHER_ERRORS = 4        # 其他错误（如总线错误）

uint64 id                     # 与 IKRequest.id 一致
uint8 status                  # OK / NO_SOLUTION / INVALID_INPUT / FK_VERIFY_WARN / OTHER_ERRORS
bool solved                   # 是否有解，等价于 status == OK
Joint joint                   # 最优解关节角 [rad]，仅 OK 时有效
