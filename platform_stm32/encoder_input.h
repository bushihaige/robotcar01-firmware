// TIM2/TIM3 编码器快照（Iteration 006.5，实现 004 的平台契约）
//
// 冻结语义（详设 §5.2，INV-006.5-4）：
//   - 编码器模式 TI12（X4）：CH1/CH2 双向计数，硬件完成，无中断；
//   - count **必须**是恰好 counter_bits 位的计数器原值：本板读 `CNT & 0xFFFF`（TIM2 虽为
//     32 位定时器，但 ARR=0xFFFF 使其等效 16 位回绕，读数显式截断与 004 契约一致）；
//   - 极性归一化（"车体前进为正"）在**平台层**完成：invert ⇒ 两位补码取反；
//   - count_baseline_reset：Init 后首帧与 Restart() 后的首帧为 true（004 据此只重建基线）；
//   - hardware_ok：定时器未使能（CR1.CEN=0）或未初始化 ⇒ false。

#ifndef ROBOTCAR01_PLATFORM_STM32_ENCODER_INPUT_H_
#define ROBOTCAR01_PLATFORM_STM32_ENCODER_INPUT_H_

#include <cstdint>

#include "app/encoder/encoder_sample.h"

namespace robotcar01::platform {

struct EncoderInputConfig {
  uint8_t counter_bits = 16U;
  bool invert_left = false;
  bool invert_right = false;
};

// 原始诊断（'e' 命令）：不经过任何换算，用来看"编码器到底有没有信号/是否悬空"。
struct EncoderRawDiagnostics {
  uint32_t tim2_count = 0;  // 右轮 TIM2(PA0/PA1) CNT 原值
  uint32_t tim3_count = 0;  // 左轮 TIM3(PA6/PA7) CNT 原值
  bool pa0 = false;         // 右编码器 A 相引脚电平
  bool pa1 = false;         // 右编码器 B 相引脚电平
  bool pa6 = false;         // 左编码器 A 相引脚电平
  bool pa7 = false;         // 左编码器 B 相引脚电平
  // 左轮 TIM2 关键寄存器：用于证明"编码器模式真的配置成功"，而不是只看计数为 0 就猜接线。
  uint32_t tim2_cr1 = 0;    // 右轮 TIM2：CEN 位应=1（计数器使能）
  uint32_t tim2_smcr = 0;   // SMS[2:0] 应为 011（TI1+TI2 编码器模式）
  uint32_t tim2_ccmr1 = 0;  // CC1S/CC2S 应为 01（两通道映射到 TI1/TI2）
  uint32_t tim2_ccer = 0;   // CC1E/CC2E 应为 1（两通道输入使能）
};

class EncoderInput {
 public:
  // 使能 GPIOA/TIM2/TIM3 时钟并启动编码器模式；失败必须保持未初始化（fail-closed）。
  bool Init(const EncoderInputConfig& config);

  bool initialized() const { return initialized_; }

  // 只读一次双通道计数器（不清零、不写 CNT）。
  encoder::EncoderSnapshot Read(uint64_t now_us);

  // 下一次 Read 置 count_baseline_reset（'z' 命令）。
  void Restart();

  // 只读原始计数与引脚电平（无副作用）。
  EncoderRawDiagnostics ReadRaw() const;

  // 自 Init 以来的累计计数（绕过 16 位计数器回绕，用于 CPR 标定）：
  // 左/右各一个，按"车体前进为正"归一化后的方向累加。
  int64_t cumulative_left() const { return cumulative_left_; }
  int64_t cumulative_right() const { return cumulative_right_; }

  // 只读 GPIOA/GPIOB 的整个输入数据寄存器（'g' 命令）：
  // 用于定位"编码器信号其实接在别的引脚上"。GPIOA/GPIOB 之外的端口时钟未使能，读回 0，不包含。
  uint32_t ReadGpioA() const;
  uint32_t ReadGpioB() const;
  uint32_t ReadOdrB() const;  // GPIOB 输出数据寄存器：区分"固件没写高"与"外部被拉低"

 private:
  EncoderInputConfig config_{};
  bool initialized_ = false;
  bool baseline_reset_pending_ = true;
  // 累计计数（CPR 标定用）：记录上一帧归一化计数，逐帧累加有符号增量。
  bool has_prev_ = false;
  uint32_t prev_left_ = 0;
  uint32_t prev_right_ = 0;
  int64_t cumulative_left_ = 0;
  int64_t cumulative_right_ = 0;
};

}  // namespace robotcar01::platform

#endif  // ROBOTCAR01_PLATFORM_STM32_ENCODER_INPUT_H_
