uint64 id                     # 与 IKRequest.id 一致
uint8 status                  # OK / NO_SOLUTION / INVALID_INPUT / FK_VERIFY_WARN / OTHER_ERRORS
bool solved                   # 是否有解，等价于 status == OK
Joint joint                   # 最优解关节角 [rad]，仅 OK 时有效
