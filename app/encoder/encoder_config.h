// 编码器估计配置（Iteration 004，robotcar01::encoder）
//
// 参数收敛原则（arch A-004）：CPR / 减速比 / 轮半径 / 位宽 / 极性 / 阈值全部来自应用配置，
// 本模块只提供**注入式值对象**，不内建生效值。所有默认值均为"占位非标定"，装配方必须显式填充
// （D-004-5）；`IsValid()` 是 fail-closed 的守门函数（D-004-6）。
//
// 结构性约束见 IsValid() 注释与 004 详细设计 §API（评审 C-4/H-8：量纲自洽必须可校验）。

#ifndef ROBOTCAR01_APP_ENCODER_ENCODER_CONFIG_H_
#define ROBOTCAR01_APP_ENCODER_ENCODER_CONFIG_H_

#include <cstdint>

namespace robotcar01::encoder {

// 编码器安装位置：电机轴（减速比参与换算）或输出轴（gear_ratio 必须为 1）。
enum class EncoderLocation : uint8_t { kMotorShaft = 0, kOutputShaft = 1 };

// 相对"车体前进为正"的极性。
enum class EncoderPolarity : uint8_t { kNormal = 0, kInverted = 1 };

struct EncoderSideConfig {
  EncoderPolarity polarity = EncoderPolarity::kNormal;
  uint8_t counter_bits = 16;  // TIM 计数器位宽；只支持 16 或 32
};

struct EncoderConfig {
  // --- 安装与标定（占位非标定）---
  EncoderLocation location = EncoderLocation::kMotorShaft;
  // 编码器**安装位置侧**的每转计数（含 X4 倍频）；kMotorShaft 时为电机轴侧计数。
  float counts_per_encoder_rev = 0.0f;
  float gear_ratio = 1.0f;       // 电机轴模型使用；输出轴模型必须为 1
  float wheel_radius_m = 0.05f;  // 100 mm 直径轮（硬件规格；待实测）

  EncoderSideConfig left{};
  EncoderSideConfig right{};

  // --- 软件质量策略（占位非标定）---
  float low_speed_zero_threshold_mps = 0.01f;
  uint32_t max_estimation_delay_us = 20000;  // 20 ms；必须 > 2×窗口
  uint32_t stall_windows_threshold = 8;      // ≥8（40 ms ≥ 2×50 Hz 命令间隔，D-004-8）
  uint32_t low_speed_accumulate_counts = 4;  // 低速多窗口累加阈值（counts）

  // 用于量纲/半程自检的应用上限（注入自 001 的 RobotCarConfig.max_wheel_speed_mps）
  float max_wheel_speed_mps = 0.5f;

  // fail-closed 校验（任一不满足 ⇒ EncoderEstimator::Init 返回 false 且保持不可用）：
  //  1. counts_per_encoder_rev > 0、gear_ratio > 0、wheel_radius_m > 0、max_wheel_speed_mps > 0；
  //  2. counter_bits ∈ {16, 32}（左右各自）；
  //  3. location == kOutputShaft ⇒ gear_ratio == 1；
  //  4. 量纲自洽：counts_per_window_max = max_wheel_speed_mps / per_count_mps ≤ 2^(bits-1) / 4，
  //     per_count_mps = 2πr / (cpr_eff · T_s)，T_s = 5 ms，cpr_eff = cpr · (motor ? gear : 1)；
  //  5. low_speed_zero_threshold_mps ≥ per_count_mps（阈值必须高于单 count 当量）；
  //  6. low_speed_accumulate_counts ≥ ceil(threshold / per_count_mps) + 1 且 ≥ 2；
  //  7. max_estimation_delay_us > 2 × 5000（> 10 ms，留给累加与滤波）；
  //  8. stall_windows_threshold ≥ 8。
  bool IsValid() const;

  // 单窗口标称周期（µs）——与 arch §6 的 5 ms 对齐；用于量纲自检与累加窗口上界。
  static constexpr uint32_t kNominalWindowUs = 5000;
};

// 单侧"每 count 轮面位移"（米）与其倒数"每 count 速度当量"（m/s）。
// cpr_eff：电机轴位置时 = cpr × gear_ratio（电机转数→输出轴转数），输出轴位置时 = cpr。
float PerCountDistanceMeters(const EncoderConfig& config);
float PerCountSpeedMps(const EncoderConfig& config);

}  // namespace robotcar01::encoder

#endif  // ROBOTCAR01_APP_ENCODER_ENCODER_CONFIG_H_
