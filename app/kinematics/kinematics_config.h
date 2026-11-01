// 运动学配置（Iteration 005，robotcar01::kinematics）
//
// 设计依据：005 详细设计 §5.1（8 条 fail-closed 校验）、D-005-5（几何与域上限复用 001
// RobotCarConfig 单一真源）、arch A-004（参数只在应用配置收敛）。
//
// 平台无关：不引用任何厂商外设头（grep 自证）；纯 POD + 纯函数判定。
// 所有新增阈值均为**占位非标定**（详设 §11 给出量级依据），必须由 006.5/008 台架标定后冻结。

#ifndef ROBOTCAR01_APP_KINEMATICS_KINEMATICS_CONFIG_H_
#define ROBOTCAR01_APP_KINEMATICS_KINEMATICS_CONFIG_H_

#include <cstdint>

#include "app/chassis/chassis_config.h"

namespace robotcar01::kinematics {

struct KinematicsConfig {
  // 几何与域上限：001 的 RobotCarConfig 是唯一真源（轮距、车体线速度/角速度上限、轮速上限）。
  chassis::RobotCarConfig drive{};

  // 车体域加减速度约束（目标域斜坡；占位非标定，详设 §11）。
  float max_body_accel_mps2 = 0.5f;
  float max_body_decel_mps2 = 0.8f;
  float max_yaw_accel_radps2 = 2.0f;
  float max_yaw_decel_radps2 = 3.0f;

  // 测量不可信（无测量或阻塞质量位）时的轮速上限（占位非标定，D-005-7）。
  // 0.2 m/s ⇒ 5 ms 窗口约 12.6 counts，高于 004 的低速累加阈值 5 counts（详设 §11 核算）。
  float degraded_max_wheel_speed_mps = 0.2f;

  // dt 钳制上界（时间不连续保护）：默认为 4 × 5 ms 调度周期。
  uint32_t max_update_interval_us = 20000;

  // fail-closed 配置校验（详设 §5.1 的 8 条）。任一条不满足 ⇒ 模块必须保持不可用。
  bool IsValid() const;
};

}  // namespace robotcar01::kinematics

#endif  // ROBOTCAR01_APP_KINEMATICS_KINEMATICS_CONFIG_H_
