// Walking skeleton 薄执行入口（Iteration 001）
//
// 每 tick 组装 kinematics → safety_gate，只产出仲裁后请求，不写 sink；
// sink 由测试/宿主侧注入写入。002 引入真实调度器与任务表后本装配器退役，
// 其职责迁移到 KinematicsTask/WheelControlTask/OutputArbitrationTask。
// 纯函数，无跨 tick 状态。

#ifndef ROBOTCAR01_APP_CHASSIS_WALKING_SKELETON_H_
#define ROBOTCAR01_APP_CHASSIS_WALKING_SKELETON_H_

#include "app/chassis/chassis_config.h"
#include "app/chassis/chassis_types.h"

namespace robotcar01::chassis {

// 单次执行运动学→安全门链路。
// input 顺序：config → command → safety；输出经返回值。
WheelSpeedRequest RunMotionSliceOnce(const RobotCarConfig& config,
                                     const MotionCommand& command,
                                     const SafetyStatus& safety);

}  // namespace robotcar01::chassis

#endif  // ROBOTCAR01_APP_CHASSIS_WALKING_SKELETON_H_
