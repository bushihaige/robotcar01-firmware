// Safety gate（Iteration 001 最小安全门）
//
// 冻结组合规则（详见详细设计 §安全门组合规则，007 只扩展输入面、不改本规则）：
//   force_disable  = !allow_motion || !WheelTarget.valid
//   limit_reason   = WheelTarget.limit_reason | (allow_motion ? kNone : kSafety)
//   输出目标       = (allow_motion && WheelTarget.valid) ? WheelTarget.actual : 0
// 本模块只做 allow_motion 与 valid 裁决；不做去抖/锁存/清故障/状态转移（007 职责）。
// 纯函数，无内部状态。

#ifndef ROBOTCAR01_APP_CHASSIS_SAFETY_GATE_H_
#define ROBOTCAR01_APP_CHASSIS_SAFETY_GATE_H_

#include "app/chassis/chassis_types.h"

namespace robotcar01::chassis {

// 将运动学输出经安全门裁决为仲裁后的速度域输出请求。
// input 顺序：target（只读输入）→ safety（只读输入）；输出经返回值。
WheelSpeedRequest RunSafetyGate(const WheelTarget& target,
                                const SafetyStatus& safety);

}  // namespace robotcar01::chassis

#endif  // ROBOTCAR01_APP_CHASSIS_SAFETY_GATE_H_
