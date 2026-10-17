#include "app/control/output_arbitration.h"

namespace robotcar01::control {

namespace {

// 对单侧请求做形状校正（INV-006-1）：非 drive 一律 duty=0；drive 必须有方向。
MotorOutputRequest Sanitize(const MotorOutputRequest& in) {
  MotorOutputRequest out = in;
  if (out.mode != OutputMode::kDrive || out.direction == MotorDirection::kNone ||
      out.duty_permille == 0u) {
    if (out.mode == OutputMode::kDrive) {
      out.mode = OutputMode::kCoast;
    }
    out.duty_permille = 0;
    if (out.mode == OutputMode::kCoast) {
      out.direction = MotorDirection::kNone;
    }
  }
  if (out.duty_permille > 1000u) {
    out.duty_permille = 1000u;
  }
  return out;
}

}  // namespace

bool OutputArbitration::Init(PwmSink& sink, const ArbitrationConfig& config) {
  if (!config.IsValid()) {
    sink_ = nullptr;
    return false;
  }
  sink_ = &sink;
  config_ = config;
  pending_ = MotorOutputPair{};
  has_request_ = false;
  fault_latched_ = false;
  fault_since_us_ = 0;
  good_submits_since_fault_ = 0;
  stats_ = ArbitrationStats{};
  return true;
}

void OutputArbitration::Submit(const MotorOutputPair& request) {
  pending_ = request;
  has_request_ = true;
  ++stats_.submits;
  if (request.left.fault || request.right.fault) {
    if (!fault_latched_) {
      fault_since_us_ = request.produced_at_us;
    }
    fault_latched_ = true;
    good_submits_since_fault_ = 0;
  } else if (fault_latched_) {
    ++good_submits_since_fault_;
  }
}

void OutputArbitration::Tick(bool allow_motion, uint64_t now_us) {
  if (sink_ == nullptr) {
    return;  // 未初始化：无输出通道（装配期 fail-closed）
  }
  ++stats_.ticks;

  MotorOutputRequest left = CoastRequest(kOutputNone);
  MotorOutputRequest right = CoastRequest(kOutputNone);

  // 故障保持与回差：保持期满 且 连续 N 个无故障请求 才解除
  if (fault_latched_ && now_us >= fault_since_us_ &&
      (now_us - fault_since_us_) >= config_.fault_hold_us &&
      good_submits_since_fault_ >= config_.fault_recover_submits) {
    fault_latched_ = false;
  }

  const bool stale = !has_request_ || now_us < pending_.produced_at_us ||
                     (now_us - pending_.produced_at_us) > config_.max_request_age_us;

  if (!allow_motion) {
    ++stats_.safety_zero;
    left.reason = right.reason = kOutputSafetyBlocked;
  } else if (fault_latched_) {
    ++stats_.fault_zero;
    ++stats_.fault_hold_active;
    left.reason = right.reason = kOutputControllerFault;
    left.fault = right.fault = true;
  } else if (stale) {
    ++stats_.stale_zero;
    left.reason = right.reason = kOutputStale;
  } else {
    left = Sanitize(pending_.left);
    right = Sanitize(pending_.right);
  }

  ++stats_.apply_calls;
  if (left.duty_permille > 0u || right.duty_permille > 0u) {
    ++stats_.drive_applies;
  }
  last_left_ = left;
  last_right_ = right;
  const PwmApplyStatus status = sink_->Apply(left, right);  // 唯一调用点（INV-006-4）
  if (!status.ok) {
    ++stats_.platform_errors;
  }
}

}  // namespace robotcar01::control
