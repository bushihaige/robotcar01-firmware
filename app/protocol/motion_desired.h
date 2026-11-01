// motion_desired 唯一构造式（Iteration 005，落地 004 D-004-8）
//
// 背景：004 详细设计 D-004-8 冻结了 motion_desired 的**唯一构造式**：
//   armed && run_requested && !stop_requested && 租约未过期 && allow_motion
// 004 只消费 bool 入参（避免 encoder 依赖 protocol 形成环）；005 把它落到装配层可复用的
// 自由函数，供 004 的停滞门控、005 的运动门、008 的任务装配共用，消除装配层内联副本。
//
// 与 CommandManager 的关系：`租约未过期` 的判定与 CommandManager::CommandValidAt 等价
// （has_command && now_us <= valid_until_us，INV-003-8）；本函数只读快照字段，不持有管理器。
// 职责边界：本函数不做安全裁决（007 职责），只把既有事实组合成"是否要求运动"。

#ifndef ROBOTCAR01_APP_PROTOCOL_MOTION_DESIRED_H_
#define ROBOTCAR01_APP_PROTOCOL_MOTION_DESIRED_H_

#include <cstdint>

#include "app/chassis/chassis_types.h"
#include "app/protocol/command_manager.h"

namespace robotcar01::protocol {

// safety = 007 发布的 SafetyStatus（唯一 allow_motion 裁决点）。签名与 004 详设 D-004-8 一致：
// 收 SafetyStatus 而不是裸 bool，避免同一"唯一构造式"出现两个签名（评审 H8）。
inline bool MakeMotionDesired(const CommandSnapshot& snapshot, uint64_t now_us,
                              const chassis::SafetyStatus& safety) {
  if (!safety.allow_motion) {
    return false;
  }
  if (!snapshot.armed || !snapshot.run_requested || snapshot.stop_requested) {
    return false;
  }
  if (!snapshot.has_command) {
    return false;
  }
  return now_us <= snapshot.valid_until_us;
}

}  // namespace robotcar01::protocol

#endif  // ROBOTCAR01_APP_PROTOCOL_MOTION_DESIRED_H_
