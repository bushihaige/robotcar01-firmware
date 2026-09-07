// RobotCar01 底盘应用配置（Iteration 001 walking skeleton）
//
// 配置是车辆几何与软件安全上限的唯一事实来源（arch §8.1 / 概念设计 D-003）。
// 默认值取自硬件规格目标（轮距约 0.33 m、车体限速 0.5 m/s），属"占位非标定"；
// 测试必须构造显式配置，禁止依赖隐式默认。

#ifndef ROBOTCAR01_APP_CHASSIS_CHASSIS_CONFIG_H_
#define ROBOTCAR01_APP_CHASSIS_CHASSIS_CONFIG_H_

namespace robotcar01::chassis {

struct RobotCarConfig {
  float track_width_m = 0.33f;        // 左右轮距（单位 m，P0；规格目标，待实测）
  float max_body_speed_mps = 0.5f;    // 车体最大线速度（单位 m/s；规格目标上限）
  float max_yaw_rate_radps = 2.0f;    // 车体最大角速度（单位 rad/s；占位非标定）
  float max_wheel_speed_mps = 1.0f;   // 单轮最大线速度（单位 m/s；软件安全上限）
};

}  // namespace robotcar01::chassis

#endif  // ROBOTCAR01_APP_CHASSIS_CHASSIS_CONFIG_H_
