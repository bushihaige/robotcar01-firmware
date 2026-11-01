#include "platform_stm32/encoder_input.h"

#include "stm32f4xx_hal.h"

#include "app/encoder/wheel_state.h"
#include "app/mcu_os_lite/clock.h"

namespace robotcar01::platform {
namespace {

TIM_HandleTypeDef g_tim_left;
TIM_HandleTypeDef g_tim_right;

constexpr uint32_t kSixteenBitMask = 0xFFFFU;
constexpr uint32_t kSixteenBitSpan = 0x10000U;

// 读数显式截断到 16 位（004 契约：平台负责位宽，业务层不猜）。
uint32_t ReadCounter(const TIM_HandleTypeDef& handle) {
  return static_cast<uint32_t>(__HAL_TIM_GET_COUNTER(&handle)) & kSixteenBitMask;
}

// 极性归一化：两位补码取反，使"前进为正"。
uint32_t ApplyPolarity(uint32_t count, bool invert) {
  if (!invert) {
    return count & kSixteenBitMask;
  }
  return (kSixteenBitSpan - (count & kSixteenBitMask)) & kSixteenBitMask;
}

bool TimerRunning(const TIM_HandleTypeDef& handle) {
  return (handle.Instance != nullptr) && ((handle.Instance->CR1 & TIM_CR1_CEN) != 0U);
}

}  // namespace

bool EncoderInput::Init(const EncoderInputConfig& config) {
  config_ = config;
  if (config_.counter_bits != 16U) {
    return false;  // 本板两路编码器都按 16 位读数（TIM3 本身就是 16 位）
  }

  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_TIM2_CLK_ENABLE();
  __HAL_RCC_TIM3_CLK_ENABLE();

  GPIO_InitTypeDef gpio{};
  gpio.Mode = GPIO_MODE_AF_PP;
  gpio.Pull = GPIO_PULLUP;
  gpio.Speed = GPIO_SPEED_FREQ_LOW;
  gpio.Alternate = GPIO_AF1_TIM2;
  gpio.Pin = GPIO_PIN_0 | GPIO_PIN_1;  // TIM2_CH1/CH2 = PA0/PA1（**右**轮）
  HAL_GPIO_Init(GPIOA, &gpio);
  gpio.Alternate = GPIO_AF2_TIM3;
  gpio.Pin = GPIO_PIN_6 | GPIO_PIN_7;  // TIM3_CH1/CH2 = PA6/PA7（**左**轮）
  HAL_GPIO_Init(GPIOA, &gpio);

  TIM_Encoder_InitTypeDef encoder{};
  encoder.EncoderMode = TIM_ENCODERMODE_TI12;  // X4：两相双边沿
  encoder.IC1Polarity = TIM_ICPOLARITY_RISING;
  encoder.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  encoder.IC1Prescaler = TIM_ICPSC_DIV1;
  encoder.IC1Filter = 0U;  // 编码器信号质量未知，先不滤波（标定后可加）
  encoder.IC2Polarity = TIM_ICPOLARITY_RISING;
  encoder.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  encoder.IC2Prescaler = TIM_ICPSC_DIV1;
  encoder.IC2Filter = 0U;

  // 引脚归属（2026-10-25 现场确认）：TIM2=PA0/PA1 属于**右**轮，TIM3=PA6/PA7 属于**左**轮。
  g_tim_left.Instance = TIM3;
  g_tim_left.Init.Prescaler = 0U;
  g_tim_left.Init.CounterMode = TIM_COUNTERMODE_UP;
  g_tim_left.Init.Period = kSixteenBitMask;
  g_tim_left.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  g_tim_left.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Encoder_Init(&g_tim_left, &encoder) != HAL_OK) {
    return false;
  }

  g_tim_right.Instance = TIM2;
  g_tim_right.Init.Prescaler = 0U;
  g_tim_right.Init.CounterMode = TIM_COUNTERMODE_UP;
  g_tim_right.Init.Period = kSixteenBitMask;  // 使 32 位 TIM2 等效 16 位回绕
  g_tim_right.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  g_tim_right.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Encoder_Init(&g_tim_right, &encoder) != HAL_OK) {
    return false;
  }

  __HAL_TIM_SET_COUNTER(&g_tim_left, 0U);
  __HAL_TIM_SET_COUNTER(&g_tim_right, 0U);

  if (HAL_TIM_Encoder_Start(&g_tim_left, TIM_CHANNEL_ALL) != HAL_OK) {
    return false;
  }
  if (HAL_TIM_Encoder_Start(&g_tim_right, TIM_CHANNEL_ALL) != HAL_OK) {
    return false;
  }

  baseline_reset_pending_ = true;
  has_prev_ = false;
  cumulative_left_ = 0;
  cumulative_right_ = 0;
  initialized_ = true;
  return true;
}

encoder::EncoderSnapshot EncoderInput::Read(uint64_t now_us) {
  encoder::EncoderSnapshot snapshot{};
  snapshot.timestamp = mcu_os_lite::TimePoint{now_us};
  if (!initialized_) {
    snapshot.valid = false;
    return snapshot;
  }

  const bool running_left = TimerRunning(g_tim_left);
  const bool running_right = TimerRunning(g_tim_right);
  const bool reset = baseline_reset_pending_;
  baseline_reset_pending_ = false;

  snapshot.left.count = ApplyPolarity(ReadCounter(g_tim_left), config_.invert_left);
  snapshot.right.count = ApplyPolarity(ReadCounter(g_tim_right), config_.invert_right);

  // 累计计数：用 004 的回绕安全差分逐帧累加（uint32 → int64），供 CPR 标定读取。
  if (has_prev_ && !reset) {
    int32_t delta_left = 0;
    int32_t delta_right = 0;
    if (encoder::SignedCountDelta(snapshot.left.count, prev_left_, config_.counter_bits, delta_left)) {
      cumulative_left_ += delta_left;
    }
    if (encoder::SignedCountDelta(snapshot.right.count, prev_right_, config_.counter_bits, delta_right)) {
      cumulative_right_ += delta_right;
    }
  }
  prev_left_ = snapshot.left.count;
  prev_right_ = snapshot.right.count;
  has_prev_ = true;
  snapshot.left.hardware_ok = running_left;
  snapshot.right.hardware_ok = running_right;
  snapshot.left.count_baseline_reset = reset;
  snapshot.right.count_baseline_reset = reset;
  snapshot.valid = running_left && running_right;
  return snapshot;
}

void EncoderInput::Restart() {
  baseline_reset_pending_ = true;  // 下一帧只重建基线，不再把跨基线增量算进去
}

uint32_t EncoderInput::ReadGpioA() const { return GPIOA->IDR; }

uint32_t EncoderInput::ReadGpioB() const { return GPIOB->IDR; }

uint32_t EncoderInput::ReadOdrB() const { return GPIOB->ODR; }

EncoderRawDiagnostics EncoderInput::ReadRaw() const {
  EncoderRawDiagnostics raw{};
  raw.tim2_count = static_cast<uint32_t>(TIM2->CNT) & kSixteenBitMask;
  raw.tim3_count = static_cast<uint32_t>(TIM3->CNT) & kSixteenBitMask;
  const uint32_t idr = GPIOA->IDR;
  // PA0/PA1 属右轮编码器，PA6/PA7 属左轮编码器
  raw.pa0 = (idr & GPIO_PIN_0) != 0U;
  raw.pa1 = (idr & GPIO_PIN_1) != 0U;
  raw.pa6 = (idr & GPIO_PIN_6) != 0U;
  raw.pa7 = (idr & GPIO_PIN_7) != 0U;
  raw.tim2_cr1 = TIM2->CR1;
  raw.tim2_smcr = TIM2->SMCR;
  raw.tim2_ccmr1 = TIM2->CCMR1;
  raw.tim2_ccer = TIM2->CCER;
  return raw;
}

}  // namespace robotcar01::platform
