// host 侧虚拟编码器源（Iteration 004，header-only，仅 host）
//
// 按设定轮速生成计数快照，用于"计数快照 → WheelState"的可运行验证与端到端工具。
// 注意（评审 S-6 的自证风险）：本 fake 用**独立**的计数生成式（wheel_speed → counts），
// 被测公式是 counts → wheel_speed，方向相反，因此不构成同式自证。

#ifndef ROBOTCAR01_HOST_FAKES_FAKE_ENCODER_SOURCE_H_
#define ROBOTCAR01_HOST_FAKES_FAKE_ENCODER_SOURCE_H_

#include <cmath>
#include <cstdint>

#include "app/encoder/encoder_config.h"
#include "app/encoder/encoder_sample.h"
#include "app/mcu_os_lite/clock.h"

namespace robotcar01::host_fakes {

class FakeEncoderSource {
 public:
  // config：用于把轮速换算为计数增量（电机轴/输出轴模型均支持）。
  void Init(const robotcar01::encoder::EncoderConfig& config) {
    config_ = config;
    left_count_ = 0;
    right_count_ = 0;
    elapsed_us_ = 0;
  }

  void SetWheelSpeeds(float left_mps, float right_mps) {
    left_mps_ = left_mps;
    right_mps_ = right_mps;
  }

  // 按 dt_us 推进并产出快照（计数器按位宽自然回绕）。
  robotcar01::encoder::EncoderSnapshot Step(uint32_t dt_us) {
    robotcar01::encoder::EncoderSnapshot snapshot{};
    elapsed_us_ += dt_us;
    snapshot.timestamp = robotcar01::mcu_os_lite::TimePoint{elapsed_us_};
    snapshot.left.count = Advance(left_mps_, dt_us, config_.left.counter_bits, left_count_);
    snapshot.right.count = Advance(right_mps_, dt_us, config_.right.counter_bits, right_count_);
    snapshot.valid = true;
    return snapshot;
  }

  // 直接注入原始计数（用于回绕/故障类用例）。
  void ForceCounts(uint32_t left, uint32_t right, uint32_t dt_us) {
    elapsed_us_ += dt_us;
    left_count_ = left;
    right_count_ = right;
  }

  robotcar01::encoder::EncoderSnapshot SnapshotFromForced() const {
    robotcar01::encoder::EncoderSnapshot snapshot{};
    snapshot.timestamp = robotcar01::mcu_os_lite::TimePoint{elapsed_us_};
    snapshot.left.count = left_count_;
    snapshot.right.count = right_count_;
    snapshot.valid = true;
    return snapshot;
  }

  uint64_t elapsed_us() const { return elapsed_us_; }

 private:
  // 轮速 → 计数增量：counts = distance / per_count_distance，带累计量化误差补偿
  // （用一个"已生成计数"的整型累加器表示，避免长期漂移）。
  uint32_t Advance(float wheel_mps, uint32_t dt_us, uint8_t counter_bits, uint32_t& counter) {
    const float per_count = robotcar01::encoder::PerCountDistanceMeters(config_);
    if (per_count > 0.0f) {
      const float distance = wheel_mps * (static_cast<float>(dt_us) / 1'000'000.0f);
      // quantization_error_m_ 保存上次舍入误差，保证长时间平均速度准确
      const float total = distance + quantization_error_m_;
      const float counts_f = total / per_count;
      const auto counts = static_cast<int32_t>(counts_f >= 0.0f ? counts_f + 0.5f : counts_f - 0.5f);
      quantization_error_m_ = total - static_cast<float>(counts) * per_count;
      const uint32_t mask = (counter_bits >= 32u) ? 0xFFFFFFFFu : ((1u << counter_bits) - 1u);
      counter = (counter + static_cast<uint32_t>(counts)) & mask;
    }
    return counter;
  }

  robotcar01::encoder::EncoderConfig config_{};
  uint32_t left_count_ = 0;
  uint32_t right_count_ = 0;
  uint64_t elapsed_us_ = 0;
  float left_mps_ = 0.0f;
  float right_mps_ = 0.0f;
  float quantization_error_m_ = 0.0f;
};

}  // namespace robotcar01::host_fakes

#endif  // ROBOTCAR01_HOST_FAKES_FAKE_ENCODER_SOURCE_H_
