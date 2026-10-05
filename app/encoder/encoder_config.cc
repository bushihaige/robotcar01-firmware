// EncoderConfig 的 fail-closed 校验与量纲换算（Iteration 004，评审 C-4/H-8）
#include "app/encoder/encoder_config.h"

#include <cmath>

namespace robotcar01::encoder {

namespace {
constexpr float kPi = 3.14159265358979323846f;
bool BitsSupported(uint8_t bits) { return bits == 16u || bits == 32u; }
}  // namespace

float PerCountDistanceMeters(const EncoderConfig& config) {
  if (config.counts_per_encoder_rev <= 0.0f) {
    return 0.0f;
  }
  const float cpr_effective = (config.location == EncoderLocation::kMotorShaft)
                                  ? config.counts_per_encoder_rev * config.gear_ratio
                                  : config.counts_per_encoder_rev;
  if (cpr_effective <= 0.0f) {
    return 0.0f;
  }
  return (2.0f * kPi * config.wheel_radius_m) / cpr_effective;
}

float PerCountSpeedMps(const EncoderConfig& config) {
  const float distance = PerCountDistanceMeters(config);
  if (distance <= 0.0f) {
    return 0.0f;
  }
  return distance / (static_cast<float>(EncoderConfig::kNominalWindowUs) / 1'000'000.0f);
}

bool EncoderConfig::IsValid() const {
  // 1. 正值
  if (counts_per_encoder_rev <= 0.0f || gear_ratio <= 0.0f || wheel_radius_m <= 0.0f ||
      max_wheel_speed_mps <= 0.0f) {
    return false;
  }
  // 2. 位宽只支持 16/32
  if (!BitsSupported(left.counter_bits) || !BitsSupported(right.counter_bits)) {
    return false;
  }
  // 3. 输出轴模型强制 gear_ratio == 1（防量纲自相矛盾）
  if (location == EncoderLocation::kOutputShaft && gear_ratio != 1.0f) {
    return false;
  }
  // 7. 估计延迟预算：> 2 × 标称窗口（留给累加与滤波）
  if (max_estimation_delay_us <= 2u * kNominalWindowUs) {
    return false;
  }
  // 8. 停滞阈值 ≥ 8 窗口（40 ms ≥ 2 × 50 Hz 命令间隔）
  if (stall_windows_threshold < 8u) {
    return false;
  }
  const float per_count_mps = PerCountSpeedMps(*this);
  if (per_count_mps <= 0.0f) {
    return false;
  }
  // 5. 判零阈值必须高于单 count 速度当量
  if (low_speed_zero_threshold_mps < per_count_mps) {
    return false;
  }
  // 6. 低速累加阈值取值式：≥ ceil(threshold / per_count) + 1 且 ≥ 2
  const float required_accumulate =
      std::ceil(low_speed_zero_threshold_mps / per_count_mps) + 1.0f;
  if (low_speed_accumulate_counts < 2u ||
      static_cast<float>(low_speed_accumulate_counts) < required_accumulate) {
    return false;
  }
  // 4. 量纲/半程余量自洽：每窗口最大计数 ≤ 2^(bits-1) / 4
  const float counts_per_window_max = max_wheel_speed_mps / per_count_mps;
  const uint8_t bits =
      (left.counter_bits > right.counter_bits) ? left.counter_bits : right.counter_bits;
  const float half_range = static_cast<float>(1ull << (bits - 1u));
  if (counts_per_window_max > half_range / 4.0f) {
    return false;
  }
  return true;
}

}  // namespace robotcar01::encoder
