#include "app/kinematics/differential_kinematics.h"

#include <algorithm>
#include <cmath>

namespace robotcar01::kinematics {

namespace {

// 朝目标步进：加速用 up_limit，减速用 down_limit；返回是否被速率限制截断。
float StepToward(float current, float target, float up_limit, float down_limit, bool& limited) {
  if (target > current) {
    const float next = current + up_limit;
    if (next < target) {
      limited = true;
      return next;
    }
    return target;
  }
  if (target < current) {
    const float next = current - down_limit;
    if (next > target) {
      limited = true;
      return next;
    }
    return target;
  }
  return target;
}

}  // namespace

bool DifferentialKinematics::Init(const KinematicsConfig& config) {
  Reset();               // 清斜坡状态与统计
  initialized_ = false;  // fail-closed：先保持不可用，校验通过才置位
  config_ = config;
  if (!config_.IsValid()) {
    return false;
  }
  initialized_ = true;
  return true;
}

void DifferentialKinematics::Reset() {
  ClearRampState();
  stats_.Reset();
}

void DifferentialKinematics::ClearRampState() {
  has_ramp_state_ = false;
  last_update_us_ = 0;
  ramped_v_mps_ = 0.0f;
  ramped_omega_radps_ = 0.0f;
}

// 立即零路径使用：目标归零且"已知静止"——斜坡状态保留为**已知的零**，
// 使恢复后的第一周期从零按 accel 重新加速（A6），而不是"直接采用目标"的首次语义。
void DifferentialKinematics::ZeroRampState(uint64_t now_us) {
  has_ramp_state_ = true;
  last_update_us_ = now_us;  // 时间仍在流逝：恢复后的首个 dt 不得被停机时长放大
  ramped_v_mps_ = 0.0f;
  ramped_omega_radps_ = 0.0f;
}

chassis::WheelTarget DifferentialKinematics::ImmediateZero(bool valid,
                                                           chassis::LimitReason reason) {
  chassis::WheelTarget target{};
  target.scale = 0.0f;
  target.limit_reason = reason;
  target.valid = valid;
  return target;
}

chassis::WheelTarget DifferentialKinematics::Update(const KinematicsInputs& inputs,
                                                    uint64_t now_us) {
  // (0) fail-closed：未初始化/配置非法绝不产出有限非零目标（INV-005-4）。
  if (!initialized_) {
    // fail-closed 且**不污染统计**（详设 §7 真值表）：未 Init 时 stats 保持全 0，
    // 使 updates/invalid_commands 只反映"已初始化的运行期周期数"。
    return ImmediateZero(false, chassis::LimitReason::kInvalid);
  }

  ++stats_.updates;
  const chassis::MotionCommand& command = inputs.command;

  // (1) 命令无效或非有限 ⇒ 立即零目标 + kInvalid，斜坡状态归零（D-005-2）。
  if (!command.valid || !std::isfinite(command.v_mps) || !std::isfinite(command.omega_radps)) {
    ++stats_.invalid_commands;
    ZeroRampState(now_us);
    return ImmediateZero(false, chassis::LimitReason::kInvalid);
  }

  // (2) 运行门/安全禁止 ⇒ 立即零目标 + kSafety，斜坡状态归零（INV-005-5）。
  //     valid 保持 true：命令本身语义有效，禁止来自运行门/安全（与 001 WheelTarget.valid 语义一致）。
  if (!inputs.motion_allowed) {
    ++stats_.blocked_by_motion_gate;
    ZeroRampState(now_us);
    return ImmediateZero(true, chassis::LimitReason::kSafety);
  }

  // (3) dt 计算与钳制：时间不连续必须可观测，且不得让斜坡一次跨过目标。
  uint32_t dt_us = 0;
  if (has_ramp_state_) {
    if (now_us > last_update_us_) {
      const uint64_t delta = now_us - last_update_us_;
      if (delta > config_.max_update_interval_us) {
        ++stats_.dt_clamped;
        dt_us = config_.max_update_interval_us;
      } else {
        dt_us = static_cast<uint32_t>(delta);
      }
    } else {
      ++stats_.timestamp_anomalies;  // dt == 0 或时间倒退：本周期不推进斜坡
    }
  }
  last_update_us_ = now_us;

  // (4) 车体域统一比例限幅（严格 > 判定；单一比例保曲率）。
  const float abs_v = std::fabs(command.v_mps);
  const float abs_omega = std::fabs(command.omega_radps);
  float body_scale = 1.0f;
  if (abs_v > config_.drive.max_body_speed_mps) {
    body_scale = std::min(body_scale, config_.drive.max_body_speed_mps / abs_v);
  }
  if (abs_omega > config_.drive.max_yaw_rate_radps) {
    body_scale = std::min(body_scale, config_.drive.max_yaw_rate_radps / abs_omega);
  }
  const float v_body = command.v_mps * body_scale;
  const float omega_body = command.omega_radps * body_scale;

  // (5) 车体域加减速斜坡（D-005-4）：无"直接采用目标"的旁路——首次只建立基线（视为静止）。
  // 依据（评审 C2）：上电/Reset 后车辆的已知事实是"静止"，不存在 0 → v_max 的合法阶跃；
  // dt 未知时本周期不推进，下一周期起按 accel/decel 爬升。该语义与 004
  // "count_baseline_reset 后首帧只重建基线、不产出速度"一致（详设 §6.2）。
  bool ramp_limited = false;
  if (!has_ramp_state_) {
    has_ramp_state_ = true;
    ramped_v_mps_ = 0.0f;
    ramped_omega_radps_ = 0.0f;
    ramp_limited = (v_body > 0.0f) || (v_body < 0.0f) || (omega_body > 0.0f) || (omega_body < 0.0f);
  } else if (dt_us != 0u) {
    const float dt_s = static_cast<float>(dt_us) / 1'000'000.0f;
    ramped_v_mps_ = StepToward(ramped_v_mps_, v_body, config_.max_body_accel_mps2 * dt_s,
                               config_.max_body_decel_mps2 * dt_s, ramp_limited);
    ramped_omega_radps_ =
        StepToward(ramped_omega_radps_, omega_body, config_.max_yaw_accel_radps2 * dt_s,
                   config_.max_yaw_decel_radps2 * dt_s, ramp_limited);
  } else {
    // dt == 0/时间倒退：保持上一斜坡目标；是否仍在斜坡中由阶段(8)的比较判定。
    if ((v_body > ramped_v_mps_) || (v_body < ramped_v_mps_) || (omega_body > ramped_omega_radps_) ||
        (omega_body < ramped_omega_radps_)) {
      ramp_limited = true;
    }
  }
  if (ramp_limited) {
    ++stats_.ramp_clamped;
  }
  // 斜坡滞后峰值（评审 S1）：目标域限幅后值与已发布值之差的绝对值。
  const float ramp_lag = std::fabs(v_body - ramped_v_mps_);
  if (ramp_lag > stats_.max_ramp_lag_mps) {
    stats_.max_ramp_lag_mps = ramp_lag;
  }

  // 降级判定（D-005-7）：无测量或阻塞质量位 ⇒ 使用降级轮速上限。
  // kQualityAccumulating / kQualityStallCandidate 不在阻塞掩码内 ⇒ 不降级（INV-005-9）。
  bool degraded = false;
  if (inputs.measurement == nullptr) {
    degraded = true;
  } else {
    const uint32_t quality =
        inputs.measurement->left.quality | inputs.measurement->right.quality;
    degraded = (quality & encoder::kQualityBlockingMask) != 0u;
  }
  if (degraded) {
    ++stats_.quality_degraded;
  }

  // (6) 逆运动学。raw_* = 未做任何限幅的 IK 结果（保留原始意图，arch §8.1）。
  const float half_track = 0.5f * config_.drive.track_width_m;
  const float raw_left = command.v_mps - command.omega_radps * half_track;
  const float raw_right = command.v_mps + command.omega_radps * half_track;
  // 发布目标基于（域限幅 + 斜坡）后的车体意图。
  float left = ramped_v_mps_ - ramped_omega_radps_ * half_track;
  float right = ramped_v_mps_ + ramped_omega_radps_ * half_track;

  // (7) 轮速域统一比例限幅（含降级上限）：INV-005-1 由 max_abs 判定直接保证。
  const float wheel_limit =
      degraded ? config_.degraded_max_wheel_speed_mps : config_.drive.max_wheel_speed_mps;
  const float max_abs = std::max(std::fabs(left), std::fabs(right));
  float wheel_scale = 1.0f;
  if (max_abs > wheel_limit) {
    wheel_scale = wheel_limit / max_abs;
  }
  left *= wheel_scale;
  right *= wheel_scale;

  // 发布值反写斜坡状态：斜坡的起点必须是"上次**实际发布**的车体意图"。否则轮速域限幅或
  // 降级上限的突变（触发/恢复）会造成目标阶跃，违反"目标物理可行"的初衷（详设 §6.5/A7）。
  // IK⁻¹∘IK 精确（线性），且轮域统一比例缩放等价于车体域同比例缩放 ⇒ 曲率保持不被破坏。
  ramped_v_mps_ = 0.5f * (left + right);
  ramped_omega_radps_ = (right - left) / config_.drive.track_width_m;

  // 域候选比例（与 001 同口径的可解释性判定：谁最紧谁置位；两域同紧时两位都置）。
  const float max_raw = std::max(std::fabs(raw_left), std::fabs(raw_right));
  const float wheel_candidate_scale = (max_raw > wheel_limit) ? (wheel_limit / max_raw) : 1.0f;

  // (8) 组装 WheelTarget。
  chassis::WheelTarget target{};
  target.raw_left_mps = raw_left;
  target.raw_right_mps = raw_right;
  target.left_mps = left;
  target.right_mps = right;
  target.scale = body_scale * wheel_scale;
  target.valid = true;

  chassis::LimitReason reason = chassis::LimitReason::kNone;
  const bool body_limited = body_scale < 1.0f;
  const bool wheel_limited = wheel_scale < 1.0f;
  if (body_limited || wheel_limited) {
    if (body_limited && body_scale <= wheel_candidate_scale) {
      reason |= chassis::LimitReason::kBodySpeed;
    }
    if (wheel_candidate_scale <= body_scale) {
      reason |= chassis::LimitReason::kWheelSpeed;
    }
    // 降级上限实际限制了输出时才置位（保持"scale < 1 ⇒ 有限幅位"可解释性）。
    if (wheel_limited && degraded) {
      reason |= chassis::LimitReason::kQualityDegraded;
    }
  }
  if (ramp_limited) {
    reason |= chassis::LimitReason::kAccelLimit;
  }

  if (body_limited) {
    ++stats_.body_limited;
  }
  if (wheel_limited) {
    ++stats_.wheel_limited;
  }
  target.limit_reason = reason;
  return target;
}

}  // namespace robotcar01::kinematics
