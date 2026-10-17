// 电机输出契约与 PWM 能力接口（Iteration 006，robotcar01::control）
//
// 冻结语义（006 详设 §5，D-006-9）：
//   - OutputMode：kCoast = 驱动失能（高阻滑行，安全默认）；kDrive = 施加占空比；kBrake = 半桥双低（006.5 标定项）。
//   - INV-006-1：mode != kDrive ⇒ duty = 0；duty > 0 ⇒ mode == kDrive ∧ direction != kNone。
//   - PwmSink::Apply 只允许由 OutputArbitration::Tick 调用（INV-006-4，grep 静态证明 + 运行期计数）。
// 平台无关：不含任何厂商外设类型；无动态内存。

#ifndef ROBOTCAR01_APP_CONTROL_MOTOR_OUTPUT_H_
#define ROBOTCAR01_APP_CONTROL_MOTOR_OUTPUT_H_

#include <cstdint>

namespace robotcar01::control {

enum class OutputMode : uint8_t { kCoast = 0, kDrive = 1, kBrake = 2 };
enum class MotorDirection : uint8_t { kNone = 0, kForward = 1, kReverse = 2 };

// 原因位（位组合；0 = 正常驱动）。
enum OutputReason : uint32_t {
  kOutputNone = 0u,
  kOutputSaturated = 1u << 0,           // duty 达到 max_duty_permille
  kOutputZeroTarget = 1u << 1,          // 目标按零处理
  kOutputStopping = 1u << 2,            // 方向已锁存、等待物理停止确认（duty = 0）
  kOutputMeasurementInvalid = 1u << 3,  // 阻塞质量 / 非有限 / 超速
  kOutputSafetyBlocked = 1u << 4,       // allow_motion = false
  kOutputStale = 1u << 5,               // 仲裁侧：请求超龄（未更新按零）
  kOutputControllerFault = 1u << 6,     // 未初始化 / 配置不可用 / 非有限目标
  kOutputMeasurementStale = 1u << 7,    // 测量超龄
  kOutputIntegratorFrozen = 1u << 8,    // 停滞候选 ⇒ 积分冻结
  kOutputBelowFloor = 1u << 9,          // 0 < |target| < 测量底噪 ⇒ 按零目标处理（D-006-7）
  kOutputStopTimeout = 1u << 10,        // 停止确认超时（不释放方向锁存，INV-006-8）
  kOutputSoftStart = 1u << 11,          // 从零恢复的单向软启动步长生效
  kOutputStallFault = 1u << 12,         // 停滞超时：零输出 + fault（目标归零后清除，供 007 判断）
};

struct MotorOutputRequest {
  OutputMode mode = OutputMode::kCoast;
  MotorDirection direction = MotorDirection::kNone;
  uint16_t duty_permille = 0;
  uint32_t reason = kOutputNone;
  bool fault = false;  // 显式故障：仲裁据此双侧 coast 并保持（D-006-3）
};

struct MotorOutputPair {
  MotorOutputRequest left{};
  MotorOutputRequest right{};
  uint64_t produced_at_us = 0;
};

// 平台返回状态（arch §12：PWM/方向异常由平台返回状态或诊断）。
struct PwmApplyStatus {
  bool ok = true;
  uint32_t platform_error = 0;
};

// PWM 能力契约。写入顺序（D-006-9）：同一次调用内两侧一起应用；任一侧方向改变时，平台必须
// 先令该侧 duty = 0、再切方向、再施加 duty；mode = kCoast ⇒ 驱动失能（BTS7960 EN 拉低）。
class PwmSink {
 public:
  virtual PwmApplyStatus Apply(const MotorOutputRequest& left, const MotorOutputRequest& right) = 0;
  virtual ~PwmSink() = default;
};

// 安全零输出（coast）。
inline MotorOutputRequest CoastRequest(uint32_t reason) {
  MotorOutputRequest request{};
  request.mode = OutputMode::kCoast;
  request.direction = MotorDirection::kNone;
  request.duty_permille = 0;
  request.reason = reason;
  return request;
}

}  // namespace robotcar01::control

#endif  // ROBOTCAR01_APP_CONTROL_MOTOR_OUTPUT_H_
