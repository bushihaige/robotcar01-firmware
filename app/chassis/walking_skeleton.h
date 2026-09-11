// Walking skeleton 薄执行入口（Iteration 001）
//
// 每 tick 组装 kinematics → safety_gate，只产出仲裁后请求，不写 sink；
// sink 由测试/宿主侧注入写入。002 已引入真实调度器（robotcar01::mcu_os_lite），
// 但本装配器的退役按 D-002-3 顺延：真实业务任务（KinematicsTask/WheelControlTask/
// OutputArbitrationTask）在 003/005/006 注册轮落地时，其职责迁移到任务体，本入口
// 才退役；002 不创建业务任务，故本切片保持为 001 垂直切片回归（22 用例含本入口）。
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
