#include "app/control/wheel_speed_controller.h"

#include <cmath>

namespace robotcar01::control {

namespace {

int SignOf(float value) { return value > 0.0f ? 1 : (value < 0.0f ? -1 : 0); }

MotorDirection DirectionOf(int dir) {
  return dir > 0 ? MotorDirection::kForward : (dir < 0 ? MotorDirection::kReverse : MotorDirection::kNone);
}

uint32_t SaturatingAdd(uint32_t a, uint32_t b) {
  const uint64_t sum = static_cast<uint64_t>(a) + static_cast<uint64_t>(b);
  return sum > 0xFFFFFFFFull ? 0xFFFFFFFFu : static_cast<uint32_t>(sum);
}

}  // namespace

bool WheelSpeedController::Init(const WheelControllerConfig& config) {
  config_ = config;
  initialized_ = config_.IsValid();
  Reset();
  stats_ = WheelControlStats{};
  return initialized_;
}

void WheelSpeedController::Reset() {
  has_time_base_ = false;
  last_update_us_ = 0;
  state_ = ControlState::kIdle;
  applied_dir_ = 0;
  integral_ = 0.0f;
  prev_duty_ = 0;
  soft_start_active_ = false;
  stop_dwell_accum_us_ = 0;
  stopping_elapsed_us_ = 0;
  stall_elapsed_us_ = 0;
  stall_fault_ = false;
  last_drive_dir_ = 0;
}

void WheelSpeedController::EnterStopping() {
  if (state_ != ControlState::kStopping) {
    ++stats_.stopping_entries;
    stop_dwell_accum_us_ = 0;
    stopping_elapsed_us_ = 0;
  }
  state_ = ControlState::kStopping;
  integral_ = 0.0f;
  prev_duty_ = 0;
  soft_start_active_ = false;
}

void WheelSpeedController::LeaveDrive() {
  if (state_ == ControlState::kDrive && applied_dir_ != 0) {
    EnterStopping();
  } else if (state_ == ControlState::kIdle) {
    integral_ = 0.0f;
    prev_duty_ = 0;
  }
}

MotorOutputRequest WheelSpeedController::Finish(const MotorOutputRequest& request) {
  stats_.last_duty_permille = request.duty_permille;
  stats_.last_mode = static_cast<uint8_t>(request.mode);
  stats_.applied_direction = static_cast<uint8_t>(DirectionOf(applied_dir_));
  stats_.last_reason = request.reason;
  if (request.duty_permille > 0u) {
    const int dir = request.direction == MotorDirection::kForward ? 1 : -1;
    if (last_drive_dir_ != 0 && dir != last_drive_dir_) {
      ++stats_.direction_changes;
    }
    last_drive_dir_ = dir;
    ++stats_.drive_cycles;
  }
  return request;
}

MotorOutputRequest WheelSpeedController::Update(float target_mps,
                                               const encoder::WheelSideState& measured,
                                               uint64_t measured_at_us, bool allow_motion,
                                               uint64_t now_us) {
  ++stats_.updates;

  // dt：首个 Update 只建立时间基线（S6）；dt==0 / 倒退 / 超限 ⇒ 本周期不积分
  uint32_t dt_us = 0;
  bool dt_usable = false;
  if (has_time_base_ && now_us > last_update_us_) {
    const uint64_t elapsed = now_us - last_update_us_;
    if (elapsed <= config_.max_period_us) {
      dt_us = static_cast<uint32_t>(elapsed);
      dt_usable = true;
    } else {
      dt_us = config_.max_period_us;  // 计时用钳制值，不积分
    }
  }
  if (!has_time_base_ || now_us >= last_update_us_) {
    last_update_us_ = now_us;
    has_time_base_ = true;
  }

  // ① 门控
  if (!initialized_) {
    MotorOutputRequest out = CoastRequest(kOutputControllerFault);
    out.fault = true;
    return Finish(out);
  }
  if (!std::isfinite(target_mps)) {
    LeaveDrive();
    MotorOutputRequest out = CoastRequest(kOutputControllerFault);
    out.fault = true;
    return Finish(out);
  }

  // ② 测量有效性 + ③ 新鲜度（停止确认也依赖它们）
  const bool finite_meas = std::isfinite(measured.speed_mps);
  const bool trustworthy = encoder::IsTrustworthyForControl(measured) && finite_meas &&
                           std::fabs(measured.speed_mps) <= config_.max_measured_speed_mps;
  const bool fresh = now_us >= measured_at_us &&
                     (now_us - measured_at_us) <= config_.max_measurement_age_us;
  const float meas = trustworthy ? measured.speed_mps : 0.0f;

  // 停止确认推进（STOPPING 状态下，与门控结果无关地持续评估物理静止）
  if (state_ == ControlState::kStopping) {
    stopping_elapsed_us_ = SaturatingAdd(stopping_elapsed_us_, dt_us);
    if (trustworthy && fresh && std::fabs(meas) < config_.stop_speed_threshold_mps) {
      stop_dwell_accum_us_ = SaturatingAdd(stop_dwell_accum_us_, dt_us);
    } else {
      stop_dwell_accum_us_ = 0;
    }
    if (stop_dwell_accum_us_ >= config_.stop_dwell_us) {
      state_ = ControlState::kIdle;  // 物理停止已确认：释放方向锁存
      applied_dir_ = 0;
      stop_dwell_accum_us_ = 0;
      stopping_elapsed_us_ = 0;
    }
  }

  if (!allow_motion) {
    LeaveDrive();
    uint32_t reason = kOutputSafetyBlocked;
    if (state_ == ControlState::kStopping) {
      reason |= kOutputStopping;
    }
    return Finish(CoastRequest(reason));
  }
  if (!trustworthy) {
    ++stats_.measurement_invalid_cycles;
    LeaveDrive();
    stop_dwell_accum_us_ = 0;
    return Finish(CoastRequest(kOutputMeasurementInvalid |
                               (state_ == ControlState::kStopping ? kOutputStopping : kOutputNone)));
  }
  if (!fresh) {
    ++stats_.measurement_stale_cycles;
    LeaveDrive();
    stop_dwell_accum_us_ = 0;
    return Finish(CoastRequest(kOutputMeasurementStale |
                               (state_ == ControlState::kStopping ? kOutputStopping : kOutputNone)));
  }

  // ④ 目标归一
  float target = target_mps;
  if (std::fabs(target) > config_.max_target_speed_mps) {
    target = (target > 0.0f ? 1.0f : -1.0f) * config_.max_target_speed_mps;
  }
  uint32_t extra_reason = kOutputNone;
  if (std::fabs(target) < config_.zero_target_threshold_mps) {
    if (target != 0.0f) {
      extra_reason |= kOutputBelowFloor;
      ++stats_.below_floor_cycles;
    }
    target = 0.0f;
  }
  const int target_dir = SignOf(target);
  if (target_dir == 0) {
    stall_fault_ = false;  // 停滞故障在目标归零（操作者重新下令）后清除
    stall_elapsed_us_ = 0;
  }
  if (stall_fault_) {
    LeaveDrive();
    MotorOutputRequest out = CoastRequest(kOutputStallFault |
                                          (state_ == ControlState::kStopping ? kOutputStopping : kOutputNone));
    out.fault = true;
    return Finish(out);
  }

  // ⑤ 状态机
  if (state_ == ControlState::kDrive && target_dir != applied_dir_) {
    EnterStopping();  // 目标为零或反号：保留锁存，等待物理停止（C1）
  }
  if (state_ == ControlState::kStopping && target_dir != 0 && target_dir == applied_dir_) {
    state_ = ControlState::kDrive;  // 同号回归：duty 原为 0，无换向，软启动
    soft_start_active_ = true;
    prev_duty_ = 0;
  }
  if (state_ == ControlState::kIdle && target_dir != 0) {
    const int meas_dir = SignOf(meas);
    if (std::fabs(meas) >= config_.stop_speed_threshold_mps && meas_dir != target_dir) {
      applied_dir_ = meas_dir;  // 恢复时按实测锁存：轮子仍反向转动 ⇒ 先停（C1-2）
      EnterStopping();
    } else {
      state_ = ControlState::kDrive;
      applied_dir_ = target_dir;
      integral_ = 0.0f;
      prev_duty_ = 0;
      soft_start_active_ = true;
    }
  }

  if (state_ == ControlState::kStopping) {
    uint32_t reason = kOutputStopping | extra_reason;
    if (target_dir == 0) {
      reason |= kOutputZeroTarget;
    }
    if (stopping_elapsed_us_ > config_.stop_timeout_us) {
      reason |= kOutputStopTimeout;  // 不释放锁存（INV-006-8）
      ++stats_.stop_timeouts;
    }
    return Finish(CoastRequest(reason));
  }
  if (state_ == ControlState::kIdle) {
    integral_ = 0.0f;
    prev_duty_ = 0;
    return Finish(CoastRequest(kOutputZeroTarget | extra_reason));
  }

  // ⑥ DRIVE：幅值域 FF + PI
  const float s = static_cast<float>(applied_dir_);
  const float magnitude = std::fabs(target);
  const float error = magnitude - s * meas;
  uint32_t reason = extra_reason;
  const float max_duty = static_cast<float>(config_.max_duty_permille);
  const float u_before = config_.kff_permille_per_mps * magnitude +
                         config_.kp_permille_per_mps * error + integral_;

  const bool stall_candidate = (measured.quality & encoder::kQualityStallCandidate) != 0u;
  if (stall_candidate) {
    stall_elapsed_us_ = SaturatingAdd(stall_elapsed_us_, dt_us);
    if (stall_elapsed_us_ > config_.stall_fault_timeout_us) {
      stall_fault_ = true;  // 停滞超时：判故障（C3 的安全目标：盲驱时间有界）
      ++stats_.stall_faults;
      EnterStopping();
      MotorOutputRequest out = CoastRequest(kOutputStallFault | kOutputStopping);
      out.fault = true;
      return Finish(out);
    }
  } else {
    stall_elapsed_us_ = 0;
  }
  if (stall_candidate && integral_ >= config_.stall_integral_cap_permille && error > 0.0f) {
    reason |= kOutputIntegratorFrozen;  // 停滞期积分已达起步上限：冻结（C3，有界）
    ++stats_.integrator_frozen_cycles;
  } else if (dt_usable) {
    // 条件积分（D-006-2）：用"积分后"的候选输出判定饱和，确保输出被夹到 max_duty 的周期
    // 积分绝不朝饱和方向增长（饱和边界附近也不会出现"先积分、后饱和"的爬升，INV-006-3）。
    const float increment =
        config_.ki_permille_per_mps_s * error * (static_cast<float>(dt_us) / 1e6f);
    const float u_candidate = u_before + increment;
    const bool saturated_high = (u_before >= max_duty || u_candidate >= max_duty) && error > 0.0f;
    const bool saturated_low = (u_before <= 0.0f || u_candidate <= 0.0f) && error < 0.0f;
    if (!saturated_high && !saturated_low) {
      integral_ += increment;
    }
    if (stall_candidate && integral_ > config_.stall_integral_cap_permille) {
      integral_ = config_.stall_integral_cap_permille;
    }
    if (integral_ > config_.integral_limit_permille) integral_ = config_.integral_limit_permille;
    if (integral_ < -config_.integral_limit_permille) integral_ = -config_.integral_limit_permille;
  }

  const float u_raw = config_.kff_permille_per_mps * magnitude +
                      config_.kp_permille_per_mps * error + integral_;
  if (u_raw <= 0.0f) {
    prev_duty_ = 0;
    return Finish(CoastRequest(reason | kOutputZeroTarget));
  }
  float u = u_raw;
  if (u >= max_duty) {
    u = max_duty;
    reason |= kOutputSaturated;
    ++stats_.saturated_cycles;
  }
  if (u < static_cast<float>(config_.min_effective_duty_permille)) {
    u = static_cast<float>(config_.min_effective_duty_permille);
  }
  uint16_t duty = static_cast<uint16_t>(u + 0.5f);
  if (soft_start_active_) {
    const uint32_t limit = static_cast<uint32_t>(prev_duty_) + config_.soft_start_step_permille;
    if (duty > limit) {
      duty = static_cast<uint16_t>(limit);
      reason |= kOutputSoftStart;
    } else {
      soft_start_active_ = false;
    }
  }
  prev_duty_ = duty;

  MotorOutputRequest out{};
  out.mode = OutputMode::kDrive;
  out.direction = DirectionOf(applied_dir_);
  out.duty_permille = duty;
  out.reason = reason;
  return Finish(out);
}

}  // namespace robotcar01::control
