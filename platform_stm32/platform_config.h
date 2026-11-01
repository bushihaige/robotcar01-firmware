// RobotCar01 平台参数唯一真源（Iteration 002.5，INV-2.5-3）
//
// 板级事实（B-2.5-4，变更需工程师确认）：
//   MCU  : STM32F407ZGT6（Cortex-M4F, 1 MB Flash, 128 KB SRAM + 64 KB CCM）
//   HSE  : 8 MHz 晶振
//   PLL  : M=8, N=336, P=2  → VCO_IN=1 MHz, VCO_OUT=336 MHz, SYSCLK=168 MHz
//          （F407 的 RCC_PLLCFGR 无 PLLR 字段；Q=7/Q 分频供后续 USB，本迭代不使用）
//   AHB  : /1 = 168 MHz；APB1 /4 = 42 MHz；APB2 /2 = 84 MHz
//   Flash: 5 wait states + prefetch（168 MHz @3.3 V）
//   观测 : USART1 PA9(TX)/PA10(RX) AF7 @115200 8N1（B-2.5-5）
//
// Iteration 006.5 追加（引脚表见 detailed_design §3，改动只需改本文件）：
//   编码器: TIM2 CH1/CH2 = PA0/PA1（**右**）、TIM3 CH1/CH2 = PA6/PA7（**左**），编码器模式 TI12（X4）
//   驱动  : IBT-2(BTS7960) 双 PWM + EN
//           右 RPWM=PB6(TIM4_CH1) LPWM=PB7(TIM4_CH2) EN=PB0
//           左 RPWM=PB8(TIM4_CH3) LPWM=PB9(TIM4_CH4) EN=PB1
//   注：2026-10-25 工程师现场确认——接在 PB0/PB6/PB7 + PA0/PA1 这一组上的电机是**右轮**。

#ifndef ROBOTCAR01_PLATFORM_STM32_PLATFORM_CONFIG_H_
#define ROBOTCAR01_PLATFORM_STM32_PLATFORM_CONFIG_H_

#include <cstdint>

namespace robotcar01::platform {

// ---- 时钟 ----
constexpr uint32_t kHseHz = 8000000U;      // 板载 HSE
constexpr uint32_t kPllM = 8U;             // VCO 输入 = HSE/M = 1 MHz
constexpr uint32_t kPllN = 336U;           // VCO 输出 = 336 MHz
constexpr uint32_t kPllP = 2U;             // SYSCLK = 168 MHz
constexpr uint32_t kPllQ = 7U;             // 48 MHz，USB 预留（本迭代未使用）
constexpr uint32_t kSysclkHz = 168000000U; // 期望 SYSCLK（供横幅比对）
constexpr uint32_t kHclkHz = 168000000U;
constexpr uint32_t kPclk1Hz = 42000000U;
constexpr uint32_t kPclk2Hz = 84000000U;

// ---- USART1 观测通道 ----
constexpr uint32_t kConsoleBaud = 115200U;
constexpr uint32_t kConsoleTxTimeoutMs = 100U;
constexpr uint32_t kConsoleRxRingSize = 32U;  // RX 单字节中断环（006.5）

// ---- 心跳 ----
constexpr uint32_t kHeartbeatPeriodMs = 1000U;  // 心跳周期（验收 4 的量化对象）

// =====================================================================================
// Iteration 006.5：电机与编码器（全部为可改常量；换引脚/换电机只改这里）
// =====================================================================================

// ---- PWM（TIM4，APB1=42 MHz，定时器时钟 ×2 = 84 MHz）----
constexpr uint32_t kPwmFrequencyHz = 20000U;                     // 20 kHz（超出可听范围）
constexpr uint32_t kPwmTimerClockHz = 2U * kPclk1Hz;             // 84 MHz
constexpr uint32_t kPwmPeriodCounts = kPwmTimerClockHz / kPwmFrequencyHz;  // 4200
static_assert(kPwmPeriodCounts >= 1000U, "千分比 duty 需要 ARR+1 >= 1000");
static_assert(kPwmFrequencyHz <= 25000U, "BTS7960 的 f_pwm 上限 25 kHz");

// ---- IBT-2 使能极性（手册：R_EN/L_EN 高电平使能、低电平关闭）----
constexpr bool kEnableActiveHigh = true;

// ---- 电机方向反相（D-006.5-10）----
// 语义：kMotorXxxDirectionInverted = true 表示"驱动侧相对车体前进是反相的"，
// 即 kForward 逻辑方向要走 LPWM 而不是 RPWM。判断依据是对比：
//   ① 手转前进方向 ⇒ 编码器为正（T3 标定：cum=+19602/5 圈）；
//   ② 发 kForward + 占空比 ⇒ 编码器却读到负速度（T4 实测 v=-63 mm/s）。
// 两者矛盾 ⇒ 电机/驱动侧反相。修正后命令符号与反馈符号一致，闭环才不会正反馈发散。
constexpr bool kMotorRightDirectionInverted = true;   // 2026-10-25 实测右轮（PB0/PB6/PB7 组）反相
constexpr bool kMotorLeftDirectionInverted = false;   // 左电机未接，待实测后确认

// ---- 编码器 ----
constexpr uint8_t kEncoderCounterBits = 16U;  // TIM2/TIM3 读数显式截断到 16 位
constexpr bool kEncoderLeftInverted = false;   // 左编码器未接，待实测后确认
constexpr bool kEncoderRightInverted = false;  // 实测：手转前进方向为正计数，无需反相（A1）

// ---- 编码器标定占位（D-006.5-2：输出轴模型，标定后覆盖）----
// 位置 = 输出轴 ⇒ gear_ratio 必须为 1；counts 为轮子转一圈的 X4 计数。
// 当前按 "11 线 × X4 × 90:1 = 3960" 估算（与 005 详设的单 count 当量 7.93e-5 m 一致）。
constexpr float kEncoderCountsPerWheelRev = 3960.0f;
constexpr float kGearRatio = 1.0f;
constexpr float kWheelRadiusM = 0.05f;  // 100 mm 直径轮

// ---- 控制档位（串口命令的步进与上下限）----
constexpr float kTargetStepMps = 0.05f;
constexpr float kTargetMaxMps = 0.30f;    // 空载粗调上限（远低于车体上限 0.5）
constexpr uint16_t kOpenLoopStepPermille = 100U;   // 开环扫点步进 10%
constexpr uint16_t kOpenLoopMaxPermille = 900U;

// 004 测量底噪占位：必须 ≥ 单 count 当量（0.0159 m/s @ CPR 3960 / 5 ms 窗口），
// 同时绑定为 006 的 measurement_floor_mps（D-006-7）。host 侧由 tests/platform_config_test.cc 守门。
constexpr float kMeasurementFloorMps = 0.02f;

// ---- 主循环独立监护（INV-006.5-6）----
// 距上次成功 Apply 超过该值 ⇒ 1 ms ISR 直接把 EN 拉低、CCR 清零（不等主循环恢复）。
// 取值权衡：必须 > 最长正常停摆（分帧状态行单行约 1-2 ms、控制周期 5 ms），
// 且要远小于人手反应时间。50 ms 同时满足两者。
constexpr uint32_t kMotorWatchdogTimeoutMs = 50U;

// ---- 观测 ----
constexpr uint32_t kStatusPeriodMs = 200U;  // 状态行周期

}  // namespace robotcar01::platform

#endif  // ROBOTCAR01_PLATFORM_STM32_PLATFORM_CONFIG_H_
