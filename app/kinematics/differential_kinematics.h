// 差速运动学有状态管线（Iteration 005，robotcar01::kinematics）
//
// 职责（详设 §6）：把车体意图 (v, ω) 变成可解释、物理可行的左右轮目标。
// 固定管线（顺序不可交换，D-005-1/§6.6）：
//   (0) 未初始化/非法配置 → 零目标 + kInvalid（fail-closed）
//   (1) 命令无效或非有限   → 零目标 + kInvalid，斜坡状态归零
//   (2) motion_allowed=false（含急停/安全禁止）→ 零目标 + kSafety，斜坡状态归零
//   (3) dt 计算与钳制（不连续保护）
//   (4) 车体域统一比例限幅（严格 > 判定，保曲率）
//   (5) 车体域加减速斜坡（accel/decel，D-005-4）
//   (6) 逆运动学 IK（raw_* 保留未限幅原始意图）
//   (7) 轮速域统一比例限幅（含质量降级上限，D-005-7）
//   (8) 组装 WheelTarget（raw/actual/scale/reason）
//
// 平台无关：不引用任何厂商外设头（INV-005-7，grep 自证）；无动态内存、无异常、固定计算量。
// 无全局状态：全部状态只在实例内（INV-005-6），now_us 由调用方注入（可 fake 时钟重放）。

#ifndef ROBOTCAR01_APP_KINEMATICS_DIFFERENTIAL_KINEMATICS_H_
#define ROBOTCAR01_APP_KINEMATICS_DIFFERENTIAL_KINEMATICS_H_

#include <cstdint>

#include "app/chassis/chassis_types.h"
#include "app/encoder/wheel_state.h"
#include "app/kinematics/kinematics_config.h"

namespace robotcar01::kinematics {

// 单周期输入。command 由装配层用 003 的 EffectiveSnapshot 填充（valid 表示租约内有效）；
// motion_allowed = motion_desired（004 D-004-8 唯一构造式，见 protocol::MakeMotionDesired）；
// measurement 为 nullptr 表示"无测量数据源"（保守降级）。
struct KinematicsInputs {
  chassis::MotionCommand command{};
  bool motion_allowed = false;
  const encoder::WheelState* measurement = nullptr;
};

struct KinematicsStats {
  uint32_t updates = 0;
  uint32_t invalid_commands = 0;        // 未初始化 / command.valid=false / 非有限
  uint32_t blocked_by_motion_gate = 0;  // motion_allowed=false
  uint32_t body_limited = 0;            // 车体域限幅生效的周期数
  uint32_t wheel_limited = 0;           // 轮速域限幅生效的周期数
  uint32_t quality_degraded = 0;        // 处于降级上限状态的周期数
  uint32_t ramp_clamped = 0;            // 加减速斜坡真正截断目标的周期数
  uint32_t dt_clamped = 0;              // dt > max_update_interval_us
  uint32_t timestamp_anomalies = 0;     // dt == 0 或 now_us 倒退（本周期不推进斜坡）
  // 斜坡滞后峰值 |v_目标 − v_发布|（评审 S1）：判断占位加速度是否过保守/过激的现场判据。
  // 说明：本结构体全部字段都是"周期计数"口径；当帧的位组合语义在 WheelTarget::limit_reason。
  float max_ramp_lag_mps = 0.0f;

  void Reset() { *this = KinematicsStats{}; }
};

class DifferentialKinematics {
 public:
  // fail-closed 初始化：配置非法时返回 false 且模块保持不可用（可再次 Init 重试）。
  bool Init(const KinematicsConfig& config);
  // 复位斜坡状态与统计（视为车辆静止），保留配置；用于会话重置/重连装配。
  void Reset();

  // 每周期调用一次（arch §6：KinematicsTask 5 ms）。now_us 必须单调。
  chassis::WheelTarget Update(const KinematicsInputs& inputs, uint64_t now_us);

  const KinematicsStats& stats() const { return stats_; }
  const KinematicsConfig& config() const { return config_; }
  bool initialized() const { return initialized_; }
  // 诊断/测试可观察（不参与控制）：当前斜坡后的车体意图。
  float ramped_body_speed_mps() const { return ramped_v_mps_; }
  float ramped_yaw_rate_radps() const { return ramped_omega_radps_; }

 private:
  void ClearRampState();
  void ZeroRampState(uint64_t now_us);
  static chassis::WheelTarget ImmediateZero(bool valid, chassis::LimitReason reason);

  KinematicsConfig config_{};
  KinematicsStats stats_{};
  bool initialized_ = false;
  bool has_ramp_state_ = false;  // false = 尚未建立斜坡状态（Init/Reset 后的首次 Update 直接采用目标）
  uint64_t last_update_us_ = 0;
  // 斜坡状态 = 上次**实际发布**的车体意图（由发布轮速目标反算，见 .cc 的阶段(8)注释）。
  float ramped_v_mps_ = 0.0f;
  float ramped_omega_radps_ = 0.0f;
};

}  // namespace robotcar01::kinematics

#endif  // ROBOTCAR01_APP_KINEMATICS_DIFFERENTIAL_KINEMATICS_H_
