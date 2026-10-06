// 平台编码器快照契约（Iteration 004）
//
// 平台层（008：TIM + 硬件时间戳）负责 AB/X4 解码与"车体前进为正"归一化后提交本结构；
// 业务层不读 TIM 句柄、不猜平台位宽（arch §8.2/§11）。

#ifndef ROBOTCAR01_APP_ENCODER_ENCODER_SAMPLE_H_
#define ROBOTCAR01_APP_ENCODER_ENCODER_SAMPLE_H_

#include <cstdint>

#include "app/mcu_os_lite/clock.h"

namespace robotcar01::encoder {

struct EncoderSideSample {
  // 契约（评审 C-4）：count **必须**是"恰好 counter_bits 位的硬件计数器原值"
  // （取值 0 … 2^counter_bits − 1）。32 位平台若只使用低 16 位，**必须由平台显式截断**
  // 后再提交（例如 cnt & 0xFFFF）；业务层不猜测平台位宽。
  uint32_t count = 0;
  // 平台已知的硬件/配置异常（TIM 未启动、通道故障等）：true 表示本侧计数不可信。
  bool hardware_ok = true;
  // 平台在 TIM 重启 / CNT 复位后置 true：基线不可比 ⇒ 业务侧只重建基线、不产出速度（H-5）。
  bool count_baseline_reset = false;
};

struct EncoderSnapshot {
  EncoderSideSample left{};
  EncoderSideSample right{};
  // 硬件采样时间戳优先于 HAL 毫秒 tick（arch §5.3）；也是本模块唯一的时间来源。
  mcu_os_lite::TimePoint timestamp{};
  bool valid = false;  // 平台是否产出了可用快照
};

}  // namespace robotcar01::encoder

#endif  // ROBOTCAR01_APP_ENCODER_ENCODER_SAMPLE_H_
