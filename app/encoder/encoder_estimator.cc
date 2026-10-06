#include "app/encoder/encoder_estimator.h"

#include <cmath>

namespace robotcar01::encoder {

namespace {

constexpr uint32_t kMaxAccumulatedWindows = 255u;

// 一阶滤波系数：由端到端延迟预算反推 τ（D-004-9 的延迟不等式）。
// α = 1 − exp(−dt/τ)，τ 取 max_estimation_delay_us 的 1/4（为累加窗口预留预算）。
float FilterAlpha(const EncoderConfig& config, uint32_t dt_us) {
  const float tau_us = static_cast<float>(config.max_estimation_delay_us) / 4.0f;
  if (tau_us <= 0.0f) {
    return 1.0f;
  }
  const float alpha = 1.0f - std::exp(-static_cast<float>(dt_us) / tau_us);
  if (alpha < 0.0f) {
    return 0.0f;
  }
  if (alpha > 1.0f) {
    return 1.0f;
  }
  return alpha;
}

uint32_t MaxAccumulatedUs(const EncoderConfig& config) {
  const uint32_t windows = config.max_estimation_delay_us / EncoderConfig::kNominalWindowUs;
  const uint32_t usable = (windows > 1u) ? (windows - 1u) : 1u;  // 为滤波预留一个窗口预算
  return usable * EncoderConfig::kNominalWindowUs;
}

}  // namespace

bool EncoderEstimator::Init(const EncoderConfig& config) {
  config_ = config;
  if (!config_.IsValid()) {
    initialized_ = false;
    Reset();
    initialized_ = false;  // Reset 不改 initialized_，这里显式保持不可用
    return false;
  }
  Reset();
  initialized_ = true;
  return true;
}

void EncoderEstimator::Reset() {
  has_last_sample_ = false;
  left_ = SideRuntime{};
  right_ = SideRuntime{};
  state_ = WheelState{};
  stats_ = EncoderEstimatorStats{};
}

void EncoderEstimator::ApplyFilter(SideRuntime& runtime, float raw_speed, uint32_t dt_us,
                                   WheelSideState& out) {
  if (!runtime.has_filter) {
    runtime.filtered_speed_mps = raw_speed;
    runtime.has_filter = true;
  } else {
    const float alpha = FilterAlpha(config_, dt_us);
    runtime.filtered_speed_mps += alpha * (raw_speed - runtime.filtered_speed_mps);
  }
  out.speed_mps = runtime.filtered_speed_mps;
}

bool EncoderEstimator::UpdateSide(bool is_left, const EncoderSideSample& sample,
                                  const mcu_os_lite::TimePoint& stamp, uint32_t dt_us,
                                  bool dt_is_continuation, bool motion_desired,
                                  WheelSideState& out, SideRuntime& runtime) {
  const EncoderSideConfig& side_config =
      is_left ? config_.left : config_.right;

  // --- 1) 平台质量门 ---
  if (!sample.hardware_ok) {
    ++stats_.side_hardware_fault;
    out = WheelSideState{};
    out.dt_us = dt_us;
    out.quality = kQualityHardwareFault;
    runtime.last_quality = out.quality;
    return false;
  }
  if (sample.count_baseline_reset) {
    ++stats_.baseline_resets;
    runtime.has_baseline = false;
    runtime.accumulated_counts = 0;
    runtime.accumulated_us = 0;
    runtime.accumulated_windows = 0;
    out = WheelSideState{};
    out.dt_us = dt_us;
    out.quality = kQualityHardwareFault;
    runtime.last_quality = out.quality;
    return false;
  }

  // --- 2) 基线 ---
  if (!runtime.has_baseline) {
    runtime.has_baseline = true;
    runtime.prev_count = sample.count;
    runtime.prev_timestamp = stamp;
    runtime.accumulated_counts = 0;
    runtime.accumulated_us = 0;
    runtime.accumulated_windows = 0;
    runtime.zero_windows = 0;
    out = WheelSideState{};
    out.dt_us = 0;
    return true;  // 基线建立成功（不产出速度）
  }

  // --- 3) 时间戳质量 ---
  if (dt_us == 0u || dt_us > config_.max_estimation_delay_us) {
    ++stats_.timestamp_invalid;
    runtime.last_quality = kQualityTimestampInvalid;
    out = WheelSideState{};
    out.dt_us = dt_us;
    out.quality = kQualityTimestampInvalid;
    return false;  // 不推进基线（评审 H-6）：dt 由调用方累计
  }

  // --- 4) 回绕差分 ---
  int32_t delta = 0;
  if (!SignedCountDelta(sample.count, runtime.prev_count, side_config.counter_bits, delta)) {
    ++stats_.count_out_of_range;
    runtime.last_quality = kQualityCountOutOfRange;
    out = WheelSideState{};
    out.dt_us = dt_us;
    out.quality = kQualityCountOutOfRange;
    return false;
  }

  // 基线推进（dt_is_continuation 由调用方保证：dt 已含此前被拒绝窗口的时长）
  runtime.prev_count = sample.count;
  runtime.prev_timestamp = stamp;
  (void)dt_is_continuation;

  // --- 5) 停滞/计数不变的观测 ---
  bool zero_window = (delta == 0);
  if (zero_window) {
    if (motion_desired) {
      if (runtime.zero_windows < 0xFFFFFFFFu) {
        ++runtime.zero_windows;
      }
      if (runtime.zero_windows >= config_.stall_windows_threshold) {
        ++stats_.stall_candidates;
      }
    } else {
      runtime.zero_windows = 0;  // 未要求运动：不累计停滞
    }
  } else {
    runtime.zero_windows = 0;
  }

  // --- 6) 高速路径 vs 低速累加（D-004-9 分支）---
  const uint32_t accumulate_threshold = config_.low_speed_accumulate_counts;
  const uint32_t accumulate_max_us = MaxAccumulatedUs(config_);
  // delta == 0 走停滞观测路径；delta 非零但低于累加阈值走低速累加路径（D-004-9）
  const uint32_t delta_magnitude =
      (delta < 0) ? static_cast<uint32_t>(-delta) : static_cast<uint32_t>(delta);
  const bool low_speed_path = (delta != 0) && (delta_magnitude < accumulate_threshold);

  uint32_t accumulated_windows = 1;
  uint32_t quality = kQualityValid;
  int32_t effective_delta = delta;
  uint32_t effective_dt = dt_us;

  if (low_speed_path) {
    runtime.accumulated_counts += static_cast<int64_t>(delta);
    runtime.accumulated_us += dt_us;
    if (runtime.accumulated_windows < kMaxAccumulatedWindows) {
      ++runtime.accumulated_windows;
    }
    const int64_t magnitude =
        runtime.accumulated_counts < 0 ? -runtime.accumulated_counts : runtime.accumulated_counts;
    const bool enough_counts = magnitude >= static_cast<int64_t>(accumulate_threshold);
    const bool timeout = runtime.accumulated_us >= accumulate_max_us;
    if (!enough_counts && !timeout) {
      // 累加中：valid=false 但速度字段保留上次发布值（消费方据 kQualityAccumulating 判定）
      out = WheelSideState{};
      out.count_delta = delta;
      out.dt_us = dt_us;
      out.quality = kQualityAccumulating;
      out.valid = false;
      out.accumulated_windows = runtime.accumulated_windows;
      out.speed_mps = runtime.has_filter ? runtime.filtered_speed_mps : 0.0f;
      runtime.last_quality = out.quality;
      return false;
    }
    if (timeout && !enough_counts) {
      ++stats_.low_speed_timeouts;
      quality |= kQualityLowSpeedTimeout;
    }
    effective_delta = static_cast<int32_t>(runtime.accumulated_counts);
    effective_dt = runtime.accumulated_us;
    accumulated_windows = runtime.accumulated_windows;
    runtime.accumulated_counts = 0;
    runtime.accumulated_us = 0;
    runtime.accumulated_windows = 0;
  } else {
    // 高速路径：清空累加器（避免残留跨窗口污染）
    runtime.accumulated_counts = 0;
    runtime.accumulated_us = 0;
    runtime.accumulated_windows = 0;
  }

  // --- 7) 换算与判零 ---
  WheelSideInputs inputs{};
  inputs.count_delta = effective_delta;
  inputs.dt_us = effective_dt;
  inputs.quality = quality;
  inputs.is_left = is_left;
  inputs.config = &config_;
  WheelSideState side = EstimateSide(inputs);
  if (quality != kQualityValid) {
    side.quality = quality;
  }
  side.accumulated_windows = static_cast<uint8_t>(accumulated_windows);

  // --- 8) 滤波与位置累积 ---
  if (side.valid) {
    const float measured = side.speed_mps;
    side.raw_speed_mps = measured;
    ApplyFilter(runtime, measured, effective_dt, side);
  } else {
    side.speed_mps = runtime.has_filter ? runtime.filtered_speed_mps : 0.0f;
  }

  // 停滞候选叠加（不改速度数值）
  if (zero_window && motion_desired &&
      runtime.zero_windows >= config_.stall_windows_threshold) {
    side.quality |= kQualityStallCandidate;
  } else if (zero_window && !motion_desired) {
    side.quality |= kQualityNoCountChange;
  }
  runtime.last_quality = side.quality;
  out = side;
  return true;
}

bool EncoderEstimator::OnSample(const EncoderSnapshot& snapshot, bool motion_desired) {
  if (!initialized_) {
    return false;
  }
  ++stats_.samples_submitted;
  ++state_.samples_submitted;

  if (!snapshot.valid) {
    ++stats_.snapshot_invalid;
    return false;  // 不推进基线（评审 H-6）
  }

  // dt 计算：优先与上一次被接受的快照比较；首帧或时间戳倒退时按异常处理。
  uint32_t dt_us = 0;
  bool dt_available = false;
  if (has_last_sample_) {
    const uint64_t elapsed = snapshot.timestamp.ElapsedUsSince(last_sample_timestamp_);
    if (elapsed > 0xFFFFFFFFull) {
      ++stats_.timestamp_invalid;
      return false;
    }
    dt_us = static_cast<uint32_t>(elapsed);
    dt_available = true;
  }

  // 单侧更新（首帧只建立基线）
  WheelSideState left_state{};
  WheelSideState right_state{};
  const bool left_ok =
      UpdateSide(true, snapshot.left, snapshot.timestamp, dt_us, dt_available, motion_desired,
                 left_state, left_);
  const bool right_ok =
      UpdateSide(false, snapshot.right, snapshot.timestamp, dt_us, dt_available, motion_desired,
                 right_state, right_);

  // 位置累积（只在有效发布窗口累加；INV-004-3）
  if (left_state.valid) {
    left_.filtered_speed_mps = left_state.speed_mps;
    const float previous = state_.left.position_m;
    left_state.position_m = previous + left_state.position_delta_m;
  } else {
    left_state.position_m = state_.left.position_m;
  }
  if (right_state.valid) {
    right_.filtered_speed_mps = right_state.speed_mps;
    const float previous = state_.right.position_m;
    right_state.position_m = previous + right_state.position_delta_m;
  } else {
    right_state.position_m = state_.right.position_m;
  }

  has_last_sample_ = true;
  last_sample_timestamp_ = snapshot.timestamp;
  state_.left = left_state;
  state_.right = right_state;
  state_.timestamp = snapshot.timestamp;
  ++stats_.samples_accepted;
  ++state_.samples_accepted;
  return left_ok && right_ok;
}

}  // namespace robotcar01::encoder
