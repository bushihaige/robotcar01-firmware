// 轮速/位置状态与单窗口估计（Iteration 004，robotcar01::encoder）
//
// 职责：把"计数增量 + 采样间隔"换算为可解释的轮速/位移与质量标志。
// 平台无关：不引用任何厂商外设头（INV-004-7 grep 自证）；纯数据 + 纯函数。
//
// 回绕差分规则（D-004-3，评审 H-2 冻结）：
//   diff = (uint32)(now - prev) & mask(位宽)
//   diff == 0                      ⇒ delta = 0
//   0 < diff < 2^(bits-1)          ⇒ delta = +diff
//   diff > 2^(bits-1)              ⇒ delta = -(2^bits - diff)（显式补码，不依赖实现定义转换）
//   diff == 2^(bits-1)（恰好半程） ⇒ **不可分辨**：返回 false，置 kQualityCountOutOfRange
//
// 位置语义（D-004-1/A3）：position_m 为窗口位移累积的"诊断级里程"，float 累积在 ~1 km 后
// 低速窗口增量会被舍入吞掉，故**不作控制输入**；控制路径只用 position_delta_m。

#ifndef ROBOTCAR01_APP_ENCODER_WHEEL_STATE_H_
#define ROBOTCAR01_APP_ENCODER_WHEEL_STATE_H_

#include <cstdint>

#include "app/encoder/encoder_config.h"
#include "app/mcu_os_lite/clock.h"

namespace robotcar01::encoder {

// 质量位（位组合；0 = 无问题）。
enum EncoderQuality : uint32_t {
  kQualityValid = 0u,
  kQualityCountOutOfRange = 1u << 0,   // 有符号差不可分辨（超半程或恰好半程）
  kQualityTimestampInvalid = 1u << 1,  // dt == 0 / dt > max / dt 倒退
  kQualityHardwareFault = 1u << 2,     // 平台 hardware_ok=false 或基线重建
  kQualityStallCandidate = 1u << 3,    // 连续零增量且当前被要求运动（停滞候选，非断线结论）
  kQualityLowSpeedTimeout = 1u << 4,   // 低速累加到期发布（D-004-9）
  kQualityAccumulating = 1u << 5,      // 低速累加中：valid=false 但速度字段仍为上次发布值
  kQualityNoCountChange = 1u << 6,     // 未要求运动期间计数不变（纯观测；不得单独判为断线）
};

struct WheelSideState {
  int32_t count_delta = 0;      // 本窗口有符号计数增量（已含方向极性）
  uint32_t dt_us = 0;           // 本窗口采样间隔（µs）
  float raw_speed_mps = 0.0f;   // 原始轮速（未滤波；诊断用）
  float speed_mps = 0.0f;       // 对外主输出（滤波后 / 低速累加发布值）
  float position_delta_m = 0.0f;  // 本窗口轮面位移增量
  float position_m = 0.0f;      // 累计位移（诊断级里程，D-004-1）
  uint32_t quality = kQualityValid;
  bool valid = false;           // 本窗口产出了质量合格的发布值
  uint8_t accumulated_windows = 1;  // 1 = 单窗口；>1 = 低速累加路径
};

struct WheelState {
  WheelSideState left{};
  WheelSideState right{};
  mcu_os_lite::TimePoint timestamp{};
  uint32_t samples_submitted = 0;  // 平台提交次数（含被拒绝）
  uint32_t samples_accepted = 0;   // 被接受次数；二者差值即近期拒绝数（评审 H-6）
};

// 回绕安全有符号差分。返回 false 表示不可分辨（超半程/恰好半程），out_delta 不可用。
bool SignedCountDelta(uint32_t now_count, uint32_t prev_count, uint8_t counter_bits,
                      int32_t& out_delta);

// 单侧单窗口换算（不做滤波与累计；滤波/累积在 EncoderEstimator）。
struct WheelSideInputs {
  int32_t count_delta = 0;
  uint32_t dt_us = 0;
  uint32_t quality = kQualityValid;
  bool is_left = true;
  const EncoderConfig* config = nullptr;
};

// 由 count_delta 与 dt 计算原始速度、位移增量与判零结果（质量差时速度置 0、valid=false）。
WheelSideState EstimateSide(const WheelSideInputs& inputs);

}  // namespace robotcar01::encoder

#endif  // ROBOTCAR01_APP_ENCODER_WHEEL_STATE_H_
