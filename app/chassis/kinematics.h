// Differential kinematics（Iteration 001 最小真实版）
//
// 冻结语义（详见详细设计 §运动学公式，供 Iteration 005 沿用）：
//   1. 校验：!valid || !isfinite(v) || !isfinite(ω) ⇒ valid=false、零目标、kInvalid；
//   2. 差速：raw_left = v - ω*T/2, raw_right = v + ω*T/2（T=track_width，前进为正）；
//   3. 单一统一比例 s=min(1, v_max/|v|, ω_max/|ω|, wheel_max/max(|raw|))，
//      对 raw 同乘 s 保曲率；与"先限速后差速"数值等价；
//   4. 超限判定严格用 >（恰等于上限不触发）；
//   5. reason 置位：凡候选 s<1 且等于最终 s 的域置位（可多位置位）。
// 纯函数，无内部状态；不写全局。

#ifndef ROBOTCAR01_APP_CHASSIS_KINEMATICS_H_
#define ROBOTCAR01_APP_CHASSIS_KINEMATICS_H_

#include "app/chassis/chassis_config.h"
#include "app/chassis/chassis_types.h"

namespace robotcar01::chassis {

// 将车体意图 (v, ω) 换算为左右轮目标并做统一比例限幅。
// input 顺序：config（只读输入）→ command（只读输入）；输出经返回值。
WheelTarget ComputeWheelTarget(const RobotCarConfig& config,
                               const MotionCommand& command);

}  // namespace robotcar01::chassis

#endif  // ROBOTCAR01_APP_CHASSIS_KINEMATICS_H_
