#include "platform_stm32/motor_pwm.h"

#include "stm32f4xx_hal.h"

#include "platform_stm32/platform_config.h"

namespace robotcar01::platform {
namespace {

TIM_HandleTypeDef g_tim_pwm;  // TIM4：CH1/CH2 = 左，CH3/CH4 = 右

// 引脚：右 RPWM=PB6(CH1)、右 LPWM=PB7(CH2)、左 RPWM=PB8(CH3)、左 LPWM=PB9(CH4)
// （2026-10-25 现场确认：PB0/PB6/PB7 这一组驱动的是右轮，PB1/PB8/PB9 驱动左轮）
constexpr uint16_t kPwmPins = GPIO_PIN_6 | GPIO_PIN_7 | GPIO_PIN_8 | GPIO_PIN_9;
constexpr uint16_t kEnablePins = GPIO_PIN_0 | GPIO_PIN_1;  // 右 EN=PB0，左 EN=PB1

constexpr uint32_t kPwmPeriod = kPwmPeriodCounts;  // ARR = 4200-1 见 Init

void WriteEnablePin(int index, bool on) {
  const GPIO_PinState level =
      (on == kEnableActiveHigh) ? GPIO_PIN_SET : GPIO_PIN_RESET;
  HAL_GPIO_WritePin(GPIOB, index == 0 ? GPIO_PIN_0 : GPIO_PIN_1, level);
}

}  // namespace

bool Ibt2PwmSink::Init() {
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_TIM4_CLK_ENABLE();

  // 1) EN 先配置为推挽输出并立即拉低（失能 = 高阻，安全默认）
  //    自审 Q1 修复：先把 ODR 预置为"失能"电平，再切 MODER。
  //    HAL_GPIO_Init 对输出模式会按结构体写 ODR；若将来把 kEnableActiveHigh 改成 false，
  //    仅靠"配置后再拉低"会经过一个使能窗口。这里显式按极性预置，切换极性也安全。
  const uint32_t safe_enable_bits =
      kEnableActiveHigh ? (static_cast<uint32_t>(kEnablePins) << 16U)   // 写 BR 位 ⇒ RESET（失能）
                        : static_cast<uint32_t>(kEnablePins);           // 写 BS 位 ⇒ SET（失能）
  GPIOB->BSRR = safe_enable_bits;

  GPIO_InitTypeDef gpio{};
  gpio.Mode = GPIO_MODE_OUTPUT_PP;
  gpio.Pull = GPIO_NOPULL;
  gpio.Speed = GPIO_SPEED_FREQ_LOW;
  gpio.Pin = kEnablePins;
  HAL_GPIO_Init(GPIOB, &gpio);
  HAL_GPIO_WritePin(GPIOB, kEnablePins,
                    (kEnableActiveHigh ? GPIO_PIN_RESET : GPIO_PIN_SET));  // 配置后再次确认失能

  // 2) 4 路 PWM 引脚复用为 TIM4
  gpio.Mode = GPIO_MODE_AF_PP;
  gpio.Pull = GPIO_NOPULL;
  gpio.Speed = GPIO_SPEED_FREQ_HIGH;
  gpio.Alternate = GPIO_AF2_TIM4;
  gpio.Pin = kPwmPins;
  HAL_GPIO_Init(GPIOB, &gpio);

  // 3) 时基：84 MHz / 4200 = 20 kHz，计数模式向上，ARR = 4200-1
  g_tim_pwm.Instance = TIM4;
  g_tim_pwm.Init.Prescaler = 0U;
  g_tim_pwm.Init.CounterMode = TIM_COUNTERMODE_UP;
  g_tim_pwm.Init.Period = kPwmPeriod - 1U;
  g_tim_pwm.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  g_tim_pwm.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_PWM_Init(&g_tim_pwm) != HAL_OK) {
    ForceSafeState();
    return false;
  }

  // 4) 四通道均为 PWM1 模式，初始比较值 0
  TIM_OC_InitTypeDef oc{};
  oc.OCMode = TIM_OCMODE_PWM1;
  oc.Pulse = 0U;
  oc.OCPolarity = TIM_OCPOLARITY_HIGH;
  oc.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&g_tim_pwm, &oc, TIM_CHANNEL_1) != HAL_OK) {
    ForceSafeState();
    return false;
  }
  if (HAL_TIM_PWM_ConfigChannel(&g_tim_pwm, &oc, TIM_CHANNEL_2) != HAL_OK) {
    ForceSafeState();
    return false;
  }
  if (HAL_TIM_PWM_ConfigChannel(&g_tim_pwm, &oc, TIM_CHANNEL_3) != HAL_OK) {
    ForceSafeState();
    return false;
  }
  if (HAL_TIM_PWM_ConfigChannel(&g_tim_pwm, &oc, TIM_CHANNEL_4) != HAL_OK) {
    ForceSafeState();
    return false;
  }

  __HAL_TIM_SET_COMPARE(&g_tim_pwm, TIM_CHANNEL_1, 0U);
  __HAL_TIM_SET_COMPARE(&g_tim_pwm, TIM_CHANNEL_2, 0U);
  __HAL_TIM_SET_COMPARE(&g_tim_pwm, TIM_CHANNEL_3, 0U);
  __HAL_TIM_SET_COMPARE(&g_tim_pwm, TIM_CHANNEL_4, 0U);

  if (HAL_TIM_PWM_Start(&g_tim_pwm, TIM_CHANNEL_1) != HAL_OK) {
    ForceSafeState();
    return false;
  }
  if (HAL_TIM_PWM_Start(&g_tim_pwm, TIM_CHANNEL_2) != HAL_OK) {
    ForceSafeState();
    return false;
  }
  if (HAL_TIM_PWM_Start(&g_tim_pwm, TIM_CHANNEL_3) != HAL_OK) {
    ForceSafeState();
    return false;
  }
  if (HAL_TIM_PWM_Start(&g_tim_pwm, TIM_CHANNEL_4) != HAL_OK) {
    ForceSafeState();
    return false;
  }

  last_duty_permille_[0] = 0U;
  last_duty_permille_[1] = 0U;
  enable_state_[0] = false;
  enable_state_[1] = false;
  guards_[0].OnZeroOutput();
  guards_[1].OnZeroOutput();
  // 默认开启主循环独立监护（INV-006.5-6）：主循环卡死时由 1 ms ISR 强制断电
  watchdog_timeout_ms_ = kMotorWatchdogTimeoutMs;
  watchdog_armed_ = false;
  isr_ms_ = 0U;
  initialized_ = true;
  return true;
}

void Ibt2PwmSink::ForceSafeState() {
  if (g_tim_pwm.Instance != nullptr) {
    __HAL_TIM_SET_COMPARE(&g_tim_pwm, TIM_CHANNEL_1, 0U);
    __HAL_TIM_SET_COMPARE(&g_tim_pwm, TIM_CHANNEL_2, 0U);
    __HAL_TIM_SET_COMPARE(&g_tim_pwm, TIM_CHANNEL_3, 0U);
    __HAL_TIM_SET_COMPARE(&g_tim_pwm, TIM_CHANNEL_4, 0U);
  }
  HAL_GPIO_WritePin(GPIOB, kEnablePins,
                    (kEnableActiveHigh ? GPIO_PIN_RESET : GPIO_PIN_SET));
  enable_state_[0] = false;
  enable_state_[1] = false;
  last_duty_permille_[0] = 0U;
  last_duty_permille_[1] = 0U;
  guards_[0].OnZeroOutput();
  guards_[1].OnZeroOutput();
}

uint32_t Ibt2PwmSink::DutyToCompare(uint16_t duty_permille) {
  if (duty_permille > 1000U) {
    return kPwmPeriod;
  }
  return (static_cast<uint32_t>(duty_permille) * kPwmPeriod) / 1000U;
}

void Ibt2PwmSink::SetCompare(int index, uint32_t rpwm_counts, uint32_t lpwm_counts) {
  if (index == 0) {
    __HAL_TIM_SET_COMPARE(&g_tim_pwm, TIM_CHANNEL_1, rpwm_counts);
    __HAL_TIM_SET_COMPARE(&g_tim_pwm, TIM_CHANNEL_2, lpwm_counts);
    last_duty_permille_[0] = static_cast<uint16_t>((rpwm_counts > 0U) ? ((rpwm_counts * 1000U) / kPwmPeriod) : ((lpwm_counts * 1000U) / kPwmPeriod));
  } else {
    __HAL_TIM_SET_COMPARE(&g_tim_pwm, TIM_CHANNEL_3, rpwm_counts);
    __HAL_TIM_SET_COMPARE(&g_tim_pwm, TIM_CHANNEL_4, lpwm_counts);
    last_duty_permille_[1] = static_cast<uint16_t>((rpwm_counts > 0U) ? ((rpwm_counts * 1000U) / kPwmPeriod) : ((lpwm_counts * 1000U) / kPwmPeriod));
  }
}

void Ibt2PwmSink::SetEnable(int index, bool on) {
  WriteEnablePin(index, on);
  enable_state_[index & 1] = on;
}

bool Ibt2PwmSink::WriteSide(int index, const control::MotorOutputRequest& request) {
  PwmSwitchGuard& guard = guards_[index & 1];

  const auto force_zero = [&](bool enable_brake) {
    SetCompare(index, 0U, 0U);
    SetEnable(index, enable_brake);
    guard.OnZeroOutput();
  };

  // ---- 请求合法性（INV-006-1）：非法组合一律 coast ----
  if (request.mode == control::OutputMode::kDrive) {
    if (request.direction == control::MotorDirection::kNone || request.duty_permille == 0U) {
      force_zero(false);
      ++rejected_requests_;
      return false;
    }
  } else if (request.duty_permille != 0U) {
    force_zero(false);
    ++rejected_requests_;
    return false;
  }

  // ---- 零输出族：coast / brake ----
  if (request.mode == control::OutputMode::kCoast) {
    force_zero(false);  // 先撤占空比、再断使能（高阻滑行）
    return true;
  }
  if (request.mode == control::OutputMode::kBrake) {
    force_zero(true);  // 低侧双低 = 制动
    return true;
  }

  // ---- kDrive：方向切换必须经过 1 ms 零窗口（INV-006.5-3，状态机见 pwm_switch_guard.h）----
  if (guard.OnDriveRequest(request.direction) == SwitchDecision::kZeroOnly) {
    SetCompare(index, 0U, 0U);
    SetEnable(index, false);  // 零窗口期间该侧为高阻
    return true;
  }

  // ---- 施加：先把该侧拉零，再使能，最后写占空比 ----
  const uint32_t compare = DutyToCompare(request.duty_permille);
  // 驱动侧方向反相（D-006.5-10）：kForward 逻辑方向可能要走 LPWM。
  // 反相只影响"逻辑方向 → 物理输入"的映射，不影响 guard 对逻辑方向的记忆与零窗口。
  // index 0 = PB0/PB6/PB7 组 = 右轮；index 1 = PB1/PB8/PB9 组 = 左轮
  const bool inverted = (index == 0) ? kMotorRightDirectionInverted : kMotorLeftDirectionInverted;
  const bool physical_forward =
      (request.direction == control::MotorDirection::kForward) != inverted;
  const uint32_t rpwm = physical_forward ? compare : 0U;
  const uint32_t lpwm = physical_forward ? 0U : compare;
  SetCompare(index, 0U, 0U);
  SetEnable(index, true);
  SetCompare(index, rpwm, lpwm);
  return true;
}

bool Ibt2PwmSink::ServiceWatchdogFromIsr(uint32_t now_ms) {
  isr_ms_ = now_ms;
  if (!initialized_) {
    return false;
  }
  if (watchdog_timeout_ms_ == 0U) {
    return false;
  }
  if (!watchdog_armed_) {
    // 首次调用建立基线（上电阶段不算超时）
    watchdog_last_ok_ms_ = now_ms;
    watchdog_armed_ = true;
    return false;
  }
  if (!PwmWatchdogShouldTrip(true, true, now_ms, watchdog_last_ok_ms_, watchdog_timeout_ms_)) {
    return false;
  }
  // 主循环已停止刷新输出：在 ISR 内直接把电机断电（高阻），不等主循环恢复。
  ForceSafeState();
  ++watchdog_trips_;
  watchdog_last_ok_ms_ = now_ms;  // 触发后重新起算，避免每拍重复计数
  return true;
}

control::PwmApplyStatus Ibt2PwmSink::Apply(const control::MotorOutputRequest& left,
                                           const control::MotorOutputRequest& right) {
  control::PwmApplyStatus status{};
  if (!initialized_) {
    status.ok = false;
    status.platform_error = kPwmErrNotInitialized;
    return status;
  }
  ++apply_calls_;
  // 组 A(index 0) = PB0/PB6/PB7 = 右轮；组 B(index 1) = PB1/PB8/PB9 = 左轮
  const bool ok_right = WriteSide(0, right);
  const bool ok_left = WriteSide(1, left);
  // 任何一次 Apply（含 coast）都说明主循环还活着 ⇒ 刷新监护基线
  watchdog_last_ok_ms_ = isr_ms_;
  watchdog_armed_ = true;
  status.ok = ok_left && ok_right;
  status.platform_error = status.ok ? kPwmErrNone : kPwmErrInvalidRequest;
  return status;
}

}  // namespace robotcar01::platform
