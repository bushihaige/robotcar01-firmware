// 降级诊断与断言（Iteration 002.5；评审 C-6/S-3）
//
// 原则：任何启动期失败都必须"可打印、可区分"，禁止静默死循环。
//   - DiagReportClockFailure：打印 [ERR] clk stage=N code=M（区分 HSE/PLL/切频）
//   - assert_failed：HAL 参数断言（USE_FULL_ASSERT=1）打印文件/行号后返回，不挂死

#ifndef ROBOTCAR01_PLATFORM_STM32_DIAG_H_
#define ROBOTCAR01_PLATFORM_STM32_DIAG_H_

#include <cstdint>

namespace robotcar01::platform {

// 打印时钟失败原因（stage 见 clock_init.h 的 ClockStage；code 为 HAL_StatusTypeDef）。
void DiagReportClockFailure(uint32_t stage, uint32_t hal_code);

// 断言触发次数（供主循环降级展示）。
uint32_t DiagAssertCount();

}  // namespace robotcar01::platform

#endif  // ROBOTCAR01_PLATFORM_STM32_DIAG_H_
