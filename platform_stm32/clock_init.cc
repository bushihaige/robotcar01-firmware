#include "platform_stm32/clock_init.h"

#include "platform_stm32/platform_config.h"
#include "stm32f4xx_hal.h"

namespace robotcar01::platform {

ClockResult ClockInit() {
  ClockResult result{};

  // 1) HSE 起振（先不开 PLL，便于把"HSE 坏"与"PLL 坏"分开定位，评审 S-10 的思路）。
  RCC_OscInitTypeDef osc_init{};
  osc_init.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  osc_init.HSEState = RCC_HSE_ON;
  osc_init.PLL.PLLState = RCC_PLL_OFF;
  HAL_StatusTypeDef status = HAL_RCC_OscConfig(&osc_init);
  if (status != HAL_OK) {
    result.stage = ClockStage::kHse;
    result.hal_code = static_cast<uint32_t>(status);
    return result;
  }

  // 2) PLL 配置并开启：HSE(8 MHz)/M(8)=1 MHz → ×N(336)=336 MHz → /P(2)=168 MHz。
  osc_init.PLL.PLLState = RCC_PLL_ON;
  osc_init.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  osc_init.PLL.PLLM = kPllM;
  osc_init.PLL.PLLN = kPllN;
  osc_init.PLL.PLLP = kPllP;
  osc_init.PLL.PLLQ = kPllQ;
  status = HAL_RCC_OscConfig(&osc_init);
  if (status != HAL_OK) {
    result.stage = ClockStage::kPll;
    result.hal_code = static_cast<uint32_t>(status);
    return result;
  }

  // 3) 切频：SYSCLK=PLL，AHB/1=168 MHz，APB1/4=42 MHz，APB2/2=84 MHz，Flash 5WS。
  //    HAL_RCC_ClockConfig 内部会更新 SystemCoreClock 并重定时 SysTick（评审 C-5）。
  RCC_ClkInitTypeDef clk_init{};
  clk_init.ClockType =
      RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  clk_init.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  clk_init.AHBCLKDivider = RCC_SYSCLK_DIV1;
  clk_init.APB1CLKDivider = RCC_HCLK_DIV4;
  clk_init.APB2CLKDivider = RCC_HCLK_DIV2;
  status = HAL_RCC_ClockConfig(&clk_init, FLASH_LATENCY_5);
  if (status != HAL_OK) {
    result.stage = ClockStage::kSwitch;
    result.hal_code = static_cast<uint32_t>(status);
    return result;
  }

  result.stage = ClockStage::kOk;
  return result;
}

}  // namespace robotcar01::platform
