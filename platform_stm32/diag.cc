#include "platform_stm32/diag.h"

#include "platform_stm32/uart_console.h"

namespace {
// 断言计数：DiagAssertCount() 与 assert_failed() 都要访问，放在全局匿名命名空间。
uint32_t g_assert_count = 0U;
}  // namespace

namespace robotcar01::platform {

void DiagReportClockFailure(uint32_t stage, uint32_t hal_code) {
  // 格式固定，便于脚本判定： [ERR] clk stage=<n> code=<m>
  UartWrite("[ERR] clk stage=");
  UartWriteU32(stage);
  UartWrite(" code=");
  UartWriteU32(hal_code);
  UartWriteLine(" (1=HSE 2=PLL 3=SWITCH; HSI 16MHz fallback, baud may be inaccurate)");
}

uint32_t DiagAssertCount() { return g_assert_count; }

}  // namespace robotcar01::platform

// HAL 参数断言回调（USE_FULL_ASSERT=1）。打印后返回而非死循环（B-2.5-7）。
extern "C" void assert_failed(uint8_t* file, uint32_t line) {
  ++g_assert_count;
  robotcar01::platform::UartWrite("[ERR] assert ");
  robotcar01::platform::UartWrite(reinterpret_cast<const char*>(file));
  robotcar01::platform::UartWrite(":");
  robotcar01::platform::UartWriteU32(line);
  robotcar01::platform::UartWriteLine("");
}
