// 车体速度估计（正运动学，Iteration 005，robotcar01::kinematics）
//
// 职责：把 004 的 WheelState（左右轮实测速度 + 质量位）换算为车体速度估计 (v, ω)，并聚合质量。
// 消费方：007 安全状态机（STOPPING 的"轮速达到安全零"判定）、008 板级验收、005 自身降级判定。
//
// 平台无关：不引用任何厂商外设头（INV-005-7，grep 自证）。
// 质量语义（详设 §5.2 / D-005-7）：
//   quality = left.quality | right.quality（保留"任一侧有问题"的信息）
//   valid   = 两侧 valid 且 (quality & kQualityBlockingMask) == 0
//   IsMeasurementTrustworthy = (quality & kQualityBlockingMask) == 0（**不含** valid 三态：
//     低速累加期间 valid=false 但测量仍可用于降级判定，沿用 004 契约）

#ifndef ROBOTCAR01_APP_KINEMATICS_BODY_STATE_H_
#define ROBOTCAR01_APP_KINEMATICS_BODY_STATE_H_

#include <cstdint>

#include "app/encoder/wheel_state.h"
#include "app/kinematics/kinematics_config.h"

namespace robotcar01::kinematics {

// 测量可用性三态（评审 H6）。消费方（007 STOPPING 判零、006 闭环）必须区分：
//   kUnavailable = 数值不可用（阻塞质量位/几何非法）——"没有测量"，不是"测到零"；
//   kStale       = 低速累加中（004 D-004-9）：数值为**上次发布值**，最长 max_estimation_delay
//                  后必然转为 kFresh 或（到期未达阈值）发布零值并置 kQualityLowSpeedTimeout；
//   kFresh       = 本窗口新值且两侧 valid。
enum class MeasurementState : uint8_t {
  kUnavailable = 0,
  kStale = 1,
  kFresh = 2,
};

struct BodyVelocityEstimate {
  float v_mps = 0.0f;        // 车体线速度估计，前进为正（m/s）；state != kFresh 时为"上次发布值"
  float omega_radps = 0.0f;  // 车体角速度估计，逆时针为正（rad/s）；同上
  uint32_t quality = 0;      // 左右质量位合并（保留"任一侧有问题"的信息）
  uint32_t left_quality = 0;   // 逐侧质量位（评审 H6：007 需要知道"哪一侧坏"）
  uint32_t right_quality = 0;
  bool left_valid = false;   // 左轮本窗口是否产出了质量合格的发布值（004 语义）
  bool right_valid = false;
  bool valid = false;        // 两侧 valid 且无阻塞质量位（== state == kFresh）
  // 三态测量状态（评审 H6）：007 需要区分"测得为零"与"速度不可用"，二态 bool 不够。
  // 消费约定：state != kFresh 时 v/ω **不得**被当作本窗口测量值（kStale = 上次发布值）。
  MeasurementState state = MeasurementState::kUnavailable;
};

// 正运动学（纯几何，无配置耦合；评审 S6）：v = (left + right)/2，ω = (right - left)/track_width_m。
// 只吃 track_width_m 而不是整个 KinematicsConfig：007/008 不必装配加速度等无关参数。
// 质量不可信或几何非法时不发布数值（v/ω 保持 0，state = kUnavailable）。
BodyVelocityEstimate EstimateBodyVelocity(float track_width_m, const encoder::WheelState& state);

// 测量是否可作为"可降级判定"的输入（无阻塞质量位）。
// 注意：与 BodyVelocityEstimate::valid 不同，本函数不要求两侧 valid（低速累加中仍可信，D-005-7）。
bool IsMeasurementTrustworthy(const BodyVelocityEstimate& estimate);

}  // namespace robotcar01::kinematics

#endif  // ROBOTCAR01_APP_KINEMATICS_BODY_STATE_H_
