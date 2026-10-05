// 反馈限频（Iteration 003）
//
// 相位语义冻结（评审 S6）：
//   - Init(now) 后**第一次**调用即到期（首帧不等待一个周期）；
//   - 到期判定用跨度式比较 (now - last) >= period_ms（uint64 微秒转毫秒后比较，回绕安全）；
//   - 发送失败只计数、**不推进**基线，下一周期重试（不阻塞、不影响控制路径，INV-003-6）。
// 本模块不含任何发送逻辑：只回答"现在该不该发"。

#ifndef ROBOTCAR01_APP_PROTOCOL_FEEDBACK_LIMITER_H_
#define ROBOTCAR01_APP_PROTOCOL_FEEDBACK_LIMITER_H_

#include <cstdint>

namespace robotcar01::protocol {

struct FeedbackRateConfig {
  uint32_t status_period_ms = 20;
  uint32_t diag_period_ms = 100;
};

class FeedbackLimiter {
 public:
  void Init(const FeedbackRateConfig& config, uint64_t now_us);

  bool ShouldSendStatus(uint64_t now_us);
  bool ShouldSendDiag(uint64_t now_us);

  // 由发送方在发送失败时调用；只累加对应计数，不改变相位（下一周期重试）。
  void OnSendFailed(bool was_status_frame);

  uint32_t status_dropped() const { return status_failed_; }
  uint32_t diag_dropped() const { return diag_failed_; }

 private:
  static bool Due(uint64_t now_us, uint64_t last_us, uint32_t period_ms);

  FeedbackRateConfig config_{};
  uint64_t status_last_us_ = 0;
  uint64_t diag_last_us_ = 0;
  bool status_never_sent_ = true;
  bool diag_never_sent_ = true;
  uint32_t status_failed_ = 0;
  uint32_t diag_failed_ = 0;
};

}  // namespace robotcar01::protocol

#endif  // ROBOTCAR01_APP_PROTOCOL_FEEDBACK_LIMITER_H_
