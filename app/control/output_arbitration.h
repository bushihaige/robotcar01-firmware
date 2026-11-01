// 输出仲裁（Iteration 006，robotcar01::control）
//
// 全仓唯一调用 PwmSink::Apply 的位置（INV-006-4，arch A-003）。
//   - Submit：控制任务写入请求（只存，不写 PWM）；006.5 开环扫点也必须经 Submit（D-006-10）
//   - Tick（1 ms）：allow_motion=false / 请求超龄（含从未收到）/ 任一侧 fault / 故障保持期内
//     ⇒ 双侧 coast；fault 恢复需连续 fault_recover_submits 个无故障请求（回差，D-006-3）
// 无动态内存、无异常；不引用厂商外设头。

#ifndef ROBOTCAR01_APP_CONTROL_OUTPUT_ARBITRATION_H_
#define ROBOTCAR01_APP_CONTROL_OUTPUT_ARBITRATION_H_

#include <cstdint>

#include "app/control/motor_output.h"

namespace robotcar01::control {

struct ArbitrationConfig {
  uint32_t max_request_age_us = 10000;  // 2 个控制周期
  uint32_t fault_hold_us = 50000;
  uint32_t fault_recover_submits = 3;
  bool IsValid() const {
    return max_request_age_us > 0u && fault_hold_us > 0u && fault_recover_submits > 0u;
  }
};

struct ArbitrationStats {
  uint32_t ticks = 0;
  uint32_t apply_calls = 0;
  uint32_t drive_applies = 0;
  uint32_t stale_zero = 0;
  uint32_t safety_zero = 0;
  uint32_t fault_zero = 0;
  uint32_t fault_hold_active = 0;
  uint32_t platform_errors = 0;
  uint32_t submits = 0;
};

class OutputArbitration {
 public:
  bool Init(PwmSink& sink, const ArbitrationConfig& config);
  bool initialized() const { return sink_ != nullptr; }
  void Submit(const MotorOutputPair& request);
  void Tick(bool allow_motion, uint64_t now_us);

  const ArbitrationStats& stats() const { return stats_; }
  const MotorOutputRequest& last_left() const { return last_left_; }
  const MotorOutputRequest& last_right() const { return last_right_; }

 private:
  PwmSink* sink_ = nullptr;
  ArbitrationConfig config_{};
  MotorOutputPair pending_{};
  bool has_request_ = false;
  bool fault_latched_ = false;
  uint64_t fault_since_us_ = 0;
  uint32_t good_submits_since_fault_ = 0;
  MotorOutputRequest last_left_{};
  MotorOutputRequest last_right_{};
  ArbitrationStats stats_{};
};

}  // namespace robotcar01::control

#endif  // ROBOTCAR01_APP_CONTROL_OUTPUT_ARBITRATION_H_
