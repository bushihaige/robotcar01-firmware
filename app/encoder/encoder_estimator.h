// 编码器估计器（Iteration 004，robotcar01::encoder）
//
// 每侧独立维护：计数基线、时间戳基线、一阶滤波状态、低速累加器、位置累积与停滞计数。
// 单核协作模型：只允许在 EncoderTask 上下文调用（无 ISR 读者，D-004-11）；状态按值返回。
//
// 关键契约：
//   - fail-closed：Init 配置非法 ⇒ 保持 UNAVAILABLE，OnSample 一律拒绝（D-004-6/INV-004-4）；
//   - 质量优先：质量差窗口不推进基线、不更新滤波与位置，累计 dt 到下一个可接受窗口（评审 H-6）；
//   - 低速路径（D-004-9）：单窗口计数不足时多窗口累加，发布延迟 ≤ max_estimation_delay_us；
//   - 停滞判定（D-004-8）：只发布标志，且仅在 motion_desired == true 时置位，不裁决停车。

#ifndef ROBOTCAR01_APP_ENCODER_ENCODER_ESTIMATOR_H_
#define ROBOTCAR01_APP_ENCODER_ENCODER_ESTIMATOR_H_

#include <cstdint>

#include "app/encoder/encoder_config.h"
#include "app/encoder/encoder_sample.h"
#include "app/encoder/wheel_state.h"

namespace robotcar01::encoder {

struct EncoderEstimatorStats {
  uint32_t samples_submitted = 0;
  uint32_t samples_accepted = 0;
  uint32_t snapshot_invalid = 0;      // 平台 valid=false
  uint32_t side_hardware_fault = 0;   // 单侧 hardware_ok=false
  uint32_t baseline_resets = 0;       // 平台 count_baseline_reset=true
  uint32_t count_out_of_range = 0;    // 每侧各计一次
  uint32_t timestamp_invalid = 0;
  uint32_t stall_candidates = 0;      // 置位次数（每次从"未置位"变为"置位"计一次）
  uint32_t low_speed_timeouts = 0;

  void Reset() { *this = EncoderEstimatorStats{}; }
};

class EncoderEstimator {
 public:
  bool initialized() const { return initialized_; }

  // 配置非法 ⇒ false 且对象保持不可用（fail-closed）。配置按值持有（不保存外部引用）。
  bool Init(const EncoderConfig& config);

  // 提交一份平台快照。时间戳唯一来源 = snapshot.timestamp（本函数不取时钟，便于注入 fake）。
  // motion_desired 表示"命令确实要求轮子转动且安全允许"，必须由装配侧的
  // MakeMotionDesired(...) 构造（D-004-8：唯一构造式，见 004 详设）。
  bool OnSample(const EncoderSnapshot& snapshot, bool motion_desired);

  // 按值返回最新状态（消费方无法长期持有引用绕过有效性判定，INV-004-8）。
  WheelState state() const { return state_; }

  const EncoderEstimatorStats& stats() const { return stats_; }

  // 上电/会话复位：清基线、滤波、累加器、位置与统计；配置保持不变。
  void Reset();

 private:
  struct SideRuntime {
    bool has_baseline = false;
    uint32_t prev_count = 0;
    mcu_os_lite::TimePoint prev_timestamp{};
    int64_t accumulated_counts = 0;
    uint32_t accumulated_us = 0;
    uint8_t accumulated_windows = 0;
    uint32_t zero_windows = 0;
    float filtered_speed_mps = 0.0f;
    bool has_filter = false;
    uint32_t last_quality = kQualityValid;
  };

  bool UpdateSide(bool is_left, const EncoderSideSample& sample, const mcu_os_lite::TimePoint& stamp,
                  uint32_t dt_us, bool dt_is_continuation, bool motion_desired,
                  WheelSideState& out, SideRuntime& runtime);
  void ApplyFilter(SideRuntime& runtime, float raw_speed, uint32_t dt_us, WheelSideState& out);

  EncoderConfig config_{};
  bool initialized_ = false;
  bool has_last_sample_ = false;
  mcu_os_lite::TimePoint last_sample_timestamp_{};
  SideRuntime left_{};
  SideRuntime right_{};
  WheelState state_{};
  EncoderEstimatorStats stats_{};
};

}  // namespace robotcar01::encoder

#endif  // ROBOTCAR01_APP_ENCODER_ENCODER_ESTIMATOR_H_
