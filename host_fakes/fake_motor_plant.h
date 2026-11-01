// FakeMotorPlant：单轮一阶电机模型（Iteration 006，header-only，仅 host）。
//
// 参数为独立物理假设（详设 §9，评审 H1：不得由控制器 kff 反推）：
//   增益 0.55 m/s @1000‰、死区 120‰（> min_effective 80‰）、τ = 100 ms。
//   drive：v_ss = sign · gain · max(0, duty − deadband) / (1000 − deadband)
//   coast：无驱动，按 coast_tau 自由减速（摩擦）。
// 方向切换在模型内无保护——是否带载换向由控制器/仲裁负责；plugging_events 用于检测违例。

#ifndef ROBOTCAR01_HOST_FAKES_FAKE_MOTOR_PLANT_H_
#define ROBOTCAR01_HOST_FAKES_FAKE_MOTOR_PLANT_H_

#include <cmath>
#include <cstdint>

#include "app/control/motor_output.h"

namespace robotcar01::host_fakes {

class FakeMotorPlant {
 public:
  float gain_mps = 0.55f;
  float deadband_permille = 120.0f;
  float tau_s = 0.10f;
  float coast_tau_s = 0.25f;
  float load_bias_mps = 0.0f;

  void Step(const control::MotorOutputRequest& request, uint32_t dt_us) {
    const float dt = static_cast<float>(dt_us) / 1e6f;
    float target = 0.0f;
    float tau = coast_tau_s;
    if (request.mode == control::OutputMode::kDrive && request.duty_permille > 0u) {
      const float effective =
          std::fmax(0.0f, static_cast<float>(request.duty_permille) - deadband_permille);
      float v_ss = gain_mps * effective / (1000.0f - deadband_permille);
      v_ss = std::fmax(0.0f, v_ss - load_bias_mps);
      const float applied_sign =
          request.direction == control::MotorDirection::kReverse ? -1.0f : 1.0f;
      target = applied_sign * v_ss;
      tau = tau_s;
      if (speed_ * applied_sign < 0.0f && std::fabs(speed_) > 0.05f) {
        ++plugging_events_;  // 带载换向：施加方向与运动方向相反且速度未落入阈值
      }
    }
    speed_ += (target - speed_) * (1.0f - std::exp(-dt / tau));
    position_m_ += static_cast<double>(speed_) * dt;
  }

  float speed() const { return speed_; }
  double position_m() const { return position_m_; }
  uint32_t plugging_events() const { return plugging_events_; }
  void SetSpeed(float v) { speed_ = v; }

 private:
  float speed_ = 0.0f;
  double position_m_ = 0.0;
  uint32_t plugging_events_ = 0;
};

}  // namespace robotcar01::host_fakes

#endif  // ROBOTCAR01_HOST_FAKES_FAKE_MOTOR_PLANT_H_
