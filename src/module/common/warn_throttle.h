#pragma once

#include <cstdint>

namespace genisys {

// 失败计数节流：第一次失败必记录，之后每 interval 次记录一次，成功后 Reset。
class WarnThrottle {
 public:
  bool ShouldLog(uint32_t interval) { return count_++ % interval == 0; }
  void Reset() { count_ = 0; }
  uint32_t count() const { return count_; }

 private:
  uint32_t count_ = 0;
};

}  // namespace genisys
