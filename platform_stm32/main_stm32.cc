// RobotCar01 固件入口（Iteration 002.5 最小启动 → 006.5 电机闭环装配）
//
// 启动链（详设 §6）：
//   Reset → SystemInit(vendor) → main → HAL_Init(SysTick 1ms) → ClockInit(168MHz)
//   → UartConsoleInit(USART1 PA9/PA10 @115200) → 横幅（运行时实测值）
//   → BoardApp::Init（编码器/PWM/仲裁/任务表；失败则 EN 保持失能）
//   → 主循环 { scheduler.TickLoopOnce + 命令 + 状态行 } + 1s 心跳
//
// 本文件承担 fw 入口（原 app/main.cc 占位成为孤儿，B-2.5-1）。

#include "platform_stm32/board_app.h"
#include "platform_stm32/clock_init.h"
#include "platform_stm32/clock_source.h"
#include "platform_stm32/diag.h"
#include "platform_stm32/platform_config.h"
#include "platform_stm32/uart_console.h"
#include "stm32f4xx_hal.h"

using robotcar01::platform::ClockResult;
using robotcar01::platform::ClockStage;

// HAL 不提供 SysTick_Handler（HAL 源码中无该符号），必须由应用实现，
// 否则 HAL_GetTick() 永不推进、心跳与所有超时都会失效（评审 C-5 关注点之一）。
// 006.5：同一次 1 ms 中断里推进 mcu_os_lite 的微秒时基（ISR 内只做 ++）。
extern "C" void SysTick_Handler(void) {
  HAL_IncTick();
  robotcar01::platform::SysTickClockSource::OnTickFromIsr();
  // 主循环独立监护（INV-006.5-6）：主循环卡死时在这里把电机断电
  robotcar01::platform::MotorWatchdogTickFromIsr();
}

namespace {

void PrintResetFlags() {
  robotcar01::platform::UartWrite("rst: ");
  robotcar01::platform::UartWrite(__HAL_RCC_GET_FLAG(RCC_FLAG_PORRST) ? "POR=1 " : "POR=0 ");
  robotcar01::platform::UartWrite(__HAL_RCC_GET_FLAG(RCC_FLAG_BORRST) ? "BOR=1 " : "BOR=0 ");
  robotcar01::platform::UartWrite(__HAL_RCC_GET_FLAG(RCC_FLAG_PINRST) ? "PIN=1 " : "PIN=0 ");
  robotcar01::platform::UartWrite(__HAL_RCC_GET_FLAG(RCC_FLAG_SFTRST) ? "SFT=1 " : "SFT=0 ");
  robotcar01::platform::UartWrite(__HAL_RCC_GET_FLAG(RCC_FLAG_IWDGRST) ? "IWDG=1 " : "IWDG=0 ");
  robotcar01::platform::UartWrite(__HAL_RCC_GET_FLAG(RCC_FLAG_LPWRRST) ? "LPWR=1" : "LPWR=0");
  robotcar01::platform::UartWriteLine("");
  __HAL_RCC_CLEAR_RESET_FLAGS();
}

// 横幅打印**运行时实测值**（评审 H-2：不接受只抄配置常量）。
void PrintBanner(const ClockResult& clock) {
  const uint32_t sysclk = HAL_RCC_GetSysClockFreq();
  const uint32_t hclk = HAL_RCC_GetHCLKFreq();
  const uint32_t pclk1 = HAL_RCC_GetPCLK1Freq();
  const uint32_t pclk2 = HAL_RCC_GetPCLK2Freq();

  robotcar01::platform::UartWriteLine("");
  robotcar01::platform::UartWriteLine("=== RobotCar01 bring-up (Iteration 002.5) ===");
  robotcar01::platform::UartWrite("build: ");
  robotcar01::platform::UartWriteLine(__DATE__ " " __TIME__);

  robotcar01::platform::UartWrite("clk: stage=");
  robotcar01::platform::UartWriteU32(static_cast<uint32_t>(clock.stage));
  robotcar01::platform::UartWrite(" hal_code=");
  robotcar01::platform::UartWriteU32(clock.hal_code);
  robotcar01::platform::UartWriteLine("");

  robotcar01::platform::UartWrite("clk: SYSCLK=");
  robotcar01::platform::UartWriteU32(sysclk);
  robotcar01::platform::UartWrite(" HCLK=");
  robotcar01::platform::UartWriteU32(hclk);
  robotcar01::platform::UartWrite(" PCLK1=");
  robotcar01::platform::UartWriteU32(pclk1);
  robotcar01::platform::UartWrite(" PCLK2=");
  robotcar01::platform::UartWriteU32(pclk2);
  robotcar01::platform::UartWriteLine("");

  robotcar01::platform::UartWrite("clk: expect SYSCLK=");
  robotcar01::platform::UartWriteU32(robotcar01::platform::kSysclkHz);
  robotcar01::platform::UartWrite(" HCLK=");
  robotcar01::platform::UartWriteU32(robotcar01::platform::kHclkHz);
  robotcar01::platform::UartWrite(" PCLK1=");
  robotcar01::platform::UartWriteU32(robotcar01::platform::kPclk1Hz);
  robotcar01::platform::UartWrite(" PCLK2=");
  robotcar01::platform::UartWriteU32(robotcar01::platform::kPclk2Hz);
  robotcar01::platform::UartWriteLine("");

  robotcar01::platform::UartWriteLabelHex("reg: RCC_PLLCFGR=", RCC->PLLCFGR);
  robotcar01::platform::UartWriteLine("");
  robotcar01::platform::UartWriteLabelHex("reg: RCC_CFGR=", RCC->CFGR);
  robotcar01::platform::UartWriteLine("");
  robotcar01::platform::UartWriteLabelHex("reg: FLASH_ACR=", FLASH->ACR);
  robotcar01::platform::UartWriteLine("");
  // F40x 的 VOS 语义（stm32f4xx_hal_pwr_ex.h）：PWR_REGULATOR_VOLTAGE_SCALE1 == PWR_CR_VOS，
  // 即"VOS 位 = 1"是 Scale 1（复位默认，fHCLK 上限 168 MHz）；位 = 0 才是 Scale 2（上限 144 MHz）。
  const bool vos_scale1 = (PWR->CR & PWR_CR_VOS) != 0U;
  robotcar01::platform::UartWrite("reg: VOS=");
  robotcar01::platform::UartWriteLine(vos_scale1 ? "scale1 (168MHz allowed)" : "scale2 (max 144MHz!)");
  PrintResetFlags();

  robotcar01::platform::UartWrite("match: sysclk=");
  robotcar01::platform::UartWrite(sysclk == robotcar01::platform::kSysclkHz ? "OK" : "MISMATCH");
  robotcar01::platform::UartWrite(" vos=");
  robotcar01::platform::UartWrite(vos_scale1 ? "OK" : "MISMATCH");
  robotcar01::platform::UartWrite(" flash_ws=");
  robotcar01::platform::UartWriteU32(FLASH->ACR & FLASH_ACR_LATENCY);
  robotcar01::platform::UartWriteLine("");

  robotcar01::platform::UartWrite("uart: ");
  robotcar01::platform::UartWriteU32(robotcar01::platform::kConsoleBaud);
  robotcar01::platform::UartWriteLine(" 8N1 on USART1 PA9/PA10");
}

}  // namespace

int main(void) {
  HAL_Init();

  const ClockResult clock = robotcar01::platform::ClockInit();
  robotcar01::platform::SysTickClockSource::Reset();

  // 即使时钟降级也要建立可打印通路：HAL_RCC_GetPCLK2Freq() 会按当前实际时钟算 BRR，
  // 因此 HSI 16MHz 下 115200 依旧可读（评审 C-6 的可观测降级设计）。
  (void)robotcar01::platform::UartConsoleInit(robotcar01::platform::kConsoleBaud);

  PrintBanner(clock);
  if (clock.stage != ClockStage::kOk) {
    robotcar01::platform::DiagReportClockFailure(static_cast<uint32_t>(clock.stage),
                                                 clock.hal_code);
  }

  // 静态存储：避免把整个装配（栈上数 KB）放在 main 的栈帧里。
  static robotcar01::platform::BoardApp app;
  if (!app.Init()) {
    robotcar01::platform::UartWriteLine("");
    robotcar01::platform::UartWrite("[FATAL] app init failed: ");
    robotcar01::platform::UartWriteLine(app.last_error());
    robotcar01::platform::UartWriteLine("motor output stays DISABLED (EN=0, CCR=0)");
    // 不进入主循环：调度器未运行 ⇒ 没有 Submit ⇒ Apply 永不被调用 ⇒ EN 保持失能。
    for (;;) {
    }
  }

  robotcar01::platform::UartWriteLine("");
  robotcar01::platform::UartWriteLine("=== RobotCar01 motor loop on board (Iteration 006.5) ===");
  app.PrintHelp();

  uint32_t heartbeat = 0U;
  uint32_t next_due_ms = HAL_GetTick() + robotcar01::platform::kHeartbeatPeriodMs;
  for (;;) {
    app.Step();
    if (static_cast<int32_t>(HAL_GetTick() - next_due_ms) >= 0) {
      next_due_ms += robotcar01::platform::kHeartbeatPeriodMs;
      ++heartbeat;
      robotcar01::platform::UartWrite("[hb] n=");
      robotcar01::platform::UartWriteU32(heartbeat);
      robotcar01::platform::UartWrite(" tick=");
      robotcar01::platform::UartWriteU32(HAL_GetTick());
      robotcar01::platform::UartWrite(" sched=");
      robotcar01::platform::UartWriteU32(app.SchedulerTick());
      robotcar01::platform::UartWrite(" assert=");
      robotcar01::platform::UartWriteU32(robotcar01::platform::DiagAssertCount());
      robotcar01::platform::UartWriteLine("");
    }
  }
}
