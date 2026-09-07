// RobotCar01 底盘平台无关纯数据结构（Iteration 001 walking skeleton）
//
// 边界不变量：本头文件只含 POD 与位运算辅助，不依赖任何 HAL 类型、
// 不进行堆分配；host 与 ARM GCC 两平台语义一致（C++20）。
// 输出契约裁决（D-001-1）：001 输出级为速度域 WheelSpeedRequest；
// PWM/方向域 MotorOutputRequest（Controller→OutputArbitration）留给 Iteration 006。

#ifndef ROBOTCAR01_APP_CHASSIS_CHASSIS_TYPES_H_
#define ROBOTCAR01_APP_CHASSIS_CHASSIS_TYPES_H_

#include <cstdint>

namespace robotcar01::chassis {

// 限幅/安全原因位掩码。0 值即 kNone；多原因并存时按位或组合。
// 位运算辅助见文件底部，避免在 -Werror 下依赖 static_cast。
enum class LimitReason : uint32_t {
  kNone       = 0x00,
  kInvalid    = 0x01,  // valid=false 或 v/ω 非有限
  kBodySpeed  = 0x02,  // 车体 v/ω 域约束生效（统一比例缩放）
  kWheelSpeed = 0x04,  // 轮速域约束生效（统一比例缩放）
  kSafety     = 0x08,  // allow_motion=false / 安全禁止
};

// 上位机运动意图（001 最小字段集）。
// 时间戳与命令租约由 Iteration 003/007 监督路径引入，本结构不承载墙钟。
struct MotionCommand {
  float v_mps = 0.0f;        // 车体线速度，前进为正（单位 m/s，P0）
  float omega_radps = 0.0f;  // 车体角速度，逆时针为正（单位 rad/s，P0）
  uint32_t seq = 0;          // 命令序号（003 起用于会话/租约）
  bool valid = false;        // 命令是否有效
};

// 差速运动学输出；同时保留原始意图与实际裁剪值，供 FeedbackTask 回传（arch §8.1）。
struct WheelTarget {
  float raw_left_mps = 0.0f;   // 限幅前左轮目标（保留意图）
  float raw_right_mps = 0.0f;  // 限幅前右轮目标
  float left_mps = 0.0f;       // 实际（限幅后）左轮目标
  float right_mps = 0.0f;      // 实际（限幅后）右轮目标
  float scale = 1.0f;          // 统一比例缩放因子（<1 表示被限幅）
  LimitReason limit_reason = LimitReason::kNone;  // 本次限幅原因（位组合）
  bool valid = false;          // 本次命令是否被接受
};

// 唯一安全使能（001 最小形态）。007 扩展为带原因位与状态机的完整 SafetyStatus，
// 扩展时保持成员访问方式兼容，不破坏 001 消费面。
struct SafetyStatus {
  bool allow_motion = false;
};

// 安全门输出 = 仲裁后的速度域输出请求（001 输出级，测试为唯一消费方；
// 003/008 的 FeedbackTask 将读取 limit_reason）。
struct WheelSpeedRequest {
  float left_mps = 0.0f;                 // 实际允许的左轮目标（仅放行且 valid 时非零）
  float right_mps = 0.0f;                // 实际允许的右轮目标
  bool force_disable = true;             // true ⇒ sink 必须输出零/关闭
  LimitReason limit_reason = LimitReason::kNone;  // 限幅/安全原因（001 消费方=测试）
};

// --- LimitReason 位运算辅助（评审 S1） ---
constexpr LimitReason operator|(LimitReason lhs, LimitReason rhs) {
  return static_cast<LimitReason>(static_cast<uint32_t>(lhs) |
                                  static_cast<uint32_t>(rhs));
}

constexpr LimitReason operator&(LimitReason lhs, LimitReason rhs) {
  return static_cast<LimitReason>(static_cast<uint32_t>(lhs) &
                                  static_cast<uint32_t>(rhs));
}

constexpr LimitReason& operator|=(LimitReason& lhs, LimitReason rhs) {
  lhs = lhs | rhs;
  return lhs;
}

constexpr bool IsSet(LimitReason flags, LimitReason bit) {
  return (flags & bit) != LimitReason::kNone;
}

}  // namespace robotcar01::chassis

#endif  // ROBOTCAR01_APP_CHASSIS_CHASSIS_TYPES_H_
