#include "app/protocol/feedback_limiter.h"

namespace robotcar01::protocol {

void FeedbackLimiter::Init(const FeedbackRateConfig& config, uint64_t now_us) {
  config_ = config;
  status_last_us_ = now_us;
  diag_last_us_ = now_us;
  status_never_sent_ = true;
  diag_never_sent_ = true;
  status_failed_ = 0;
  diag_failed_ = 0;
}

bool FeedbackLimiter::Due(uint64_t now_us, uint64_t last_us, uint32_t period_ms) {
  const uint64_t elapsed_us = now_us - last_us;  // 单调时基 ⇒ 非负
  return elapsed_us >= static_cast<uint64_t>(period_ms) * 1000ull;
}

bool FeedbackLimiter::ShouldSendStatus(uint64_t now_us) {
  if (status_never_sent_ || Due(now_us, status_last_us_, config_.status_period_ms)) {
    status_never_sent_ = false;
    status_last_us_ = now_us;
    return true;
  }
  return false;
}

bool FeedbackLimiter::ShouldSendDiag(uint64_t now_us) {
  if (diag_never_sent_ || Due(now_us, diag_last_us_, config_.diag_period_ms)) {
    diag_never_sent_ = false;
    diag_last_us_ = now_us;
    return true;
  }
  return false;
}

// 只计数：相位由 ShouldSend* 推进，失败不改变基线（下一周期重试）。
void FeedbackLimiter::OnSendFailed(bool was_status_frame) {
  if (was_status_frame) {
    ++status_failed_;
  } else {
    ++diag_failed_;
  }
}

}  // namespace robotcar01::protocol
