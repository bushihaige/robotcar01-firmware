#include "app/encoder/wheel_state.h"

namespace robotcar01::encoder {

namespace {

uint32_t MaskForBits(uint8_t counter_bits) {
  return (counter_bits >= 32u) ? 0xFFFFFFFFu : ((1u << counter_bits) - 1u);
}

uint32_t HalfRange(uint8_t counter_bits) {
  return (counter_bits >= 32u) ? 0x80000000u : (1u << (counter_bits - 1u));
}

}  // namespace

bool SignedCountDelta(uint32_t now_count, uint32_t prev_count, uint8_t counter_bits,
                      int32_t& out_delta) {
  if (counter_bits != 16u && counter_bits != 32u) {
    out_delta = 0;
    return false;
  }
  const uint32_t diff = (now_count - prev_count) & MaskForBits(counter_bits);
  const uint32_t half = HalfRange(counter_bits);
  if (diff == 0u) {
    out_delta = 0;
    return true;
  }
  if (diff == half) {
    out_delta = 0;  // 恰好半程：方向不可分辨（D-004-3）
    return false;
  }
  if (diff < half) {
    out_delta = static_cast<int32_t>(diff);
    return true;
  }
  // 反向回绕：显式补码（diff - 2^bits），不依赖实现定义的 uint32→int32 转换
  const uint32_t magnitude = (counter_bits >= 32u) ? (0u - diff) : ((1u << counter_bits) - diff);
  out_delta = -static_cast<int32_t>(magnitude);
  return true;
}

WheelSideState EstimateSide(const WheelSideInputs& inputs) {
  WheelSideState state{};
  state.count_delta = inputs.count_delta;
  state.dt_us = inputs.dt_us;
  state.quality = inputs.quality;
  if (inputs.config == nullptr) {
    state.quality |= kQualityHardwareFault;
    return state;
  }
  const EncoderConfig& config = *inputs.config;
  const EncoderPolarity polarity =
      inputs.is_left ? config.left.polarity : config.right.polarity;
  if (polarity == EncoderPolarity::kInverted) {
    state.count_delta = -state.count_delta;
  }

  const float per_count_m = PerCountDistanceMeters(config);
  if (per_count_m <= 0.0f || state.dt_us == 0u) {
    state.quality |= kQualityTimestampInvalid;
    return state;
  }

  state.position_delta_m = static_cast<float>(state.count_delta) * per_count_m;
  const float dt_s = static_cast<float>(state.dt_us) / 1'000'000.0f;
  const float raw = state.position_delta_m / dt_s;

  if (state.quality != kQualityValid) {
    return state;  // 质量差：不产出速度
  }

  state.raw_speed_mps = raw;
  // 判零只作用于发布值（D-004-4 与 D-004-9 作用域互斥）
  state.speed_mps =
      (raw > -config.low_speed_zero_threshold_mps && raw < config.low_speed_zero_threshold_mps)
          ? 0.0f
          : raw;
  state.valid = true;
  return state;
}

}  // namespace robotcar01::encoder
