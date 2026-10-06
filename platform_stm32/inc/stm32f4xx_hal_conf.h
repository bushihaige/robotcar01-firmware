// RobotCar01 平台侧 HAL 配置（Iteration 002.5，本项目自写，非 vendor）
//
// 依据 third_party/stm32f4-hal/Inc/stm32f4xx_hal_conf_template.h 裁剪：
//   - HSE_VALUE = 8 MHz（板载晶振，B-2.5-4 冻结值；模板默认 25 MHz 与实物不符，必须覆写）
//   - 只启用本迭代实际编译的模块，缩短编译并避免"未定义引用"歧义（评审 H-4）
//   - bring-up 期 USE_FULL_ASSERT=1：参数错误走 assert_failed（打印后降级，不死循环，B-2.5-7）
// 维护约定：本文件属于 platform_stm32/（参与 -Werror）；vendor 头文件不得修改。

#ifndef ROBOTCAR01_PLATFORM_STM32_STM32F4XX_HAL_CONF_H_
#define ROBOTCAR01_PLATFORM_STM32_STM32F4XX_HAL_CONF_H_

#ifdef __cplusplus
extern "C" {
#endif

// ---------- 模块裁剪（只留本迭代需要且已编入的模块） ----------
#define HAL_RCC_MODULE_ENABLED
#define HAL_GPIO_MODULE_ENABLED
#define HAL_UART_MODULE_ENABLED
#define HAL_CORTEX_MODULE_ENABLED
#define HAL_PWR_MODULE_ENABLED
#define HAL_DMA_MODULE_ENABLED
#define HAL_FLASH_MODULE_ENABLED
#define HAL_EXTI_MODULE_ENABLED

// ---------- 振荡器与电源参数 ----------
#if !defined(HSE_VALUE)
#define HSE_VALUE 8000000U  // 板载 HSE 晶振 8 MHz（B-2.5-4）
#endif
#if !defined(HSE_STARTUP_TIMEOUT)
#define HSE_STARTUP_TIMEOUT 100U  // ms（8 MHz 晶振起振远快于此）
#endif
#if !defined(HSI_VALUE)
#define HSI_VALUE 16000000U  // 内部 RC，降级路径使用
#endif
#if !defined(LSI_VALUE)
#define LSI_VALUE 32000U
#endif
#if !defined(LSE_VALUE)
#define LSE_VALUE 32768U
#endif
#if !defined(LSE_STARTUP_TIMEOUT)
#define LSE_STARTUP_TIMEOUT 5000U
#endif
#if !defined(EXTERNAL_CLOCK_VALUE)
#define EXTERNAL_CLOCK_VALUE 12288000U
#endif
#if !defined(VDD_VALUE)
#define VDD_VALUE 3300U
#endif
#if !defined(TICK_INT_PRIORITY)
#define TICK_INT_PRIORITY 0x00U
#endif
#if !defined(USE_RTOS)
#define USE_RTOS 0U
#endif
#if !defined(PREFETCH_ENABLE)
#define PREFETCH_ENABLE 1U
#endif
#if !defined(INSTRUCTION_CACHE_ENABLE)
#define INSTRUCTION_CACHE_ENABLE 1U
#endif
#if !defined(DATA_CACHE_ENABLE)
#define DATA_CACHE_ENABLE 1U
#endif

// ---------- 模块头（顺序与模板一致：RCC/FLASH 先于其它） ----------
#ifdef HAL_RCC_MODULE_ENABLED
#include "stm32f4xx_hal_rcc.h"
#endif
#ifdef HAL_GPIO_MODULE_ENABLED
#include "stm32f4xx_hal_gpio.h"
#endif
#ifdef HAL_EXTI_MODULE_ENABLED
#include "stm32f4xx_hal_exti.h"
#endif
#ifdef HAL_DMA_MODULE_ENABLED
#include "stm32f4xx_hal_dma.h"
#endif
#ifdef HAL_CORTEX_MODULE_ENABLED
#include "stm32f4xx_hal_cortex.h"
#endif
#ifdef HAL_PWR_MODULE_ENABLED
#include "stm32f4xx_hal_pwr.h"
#endif
#ifdef HAL_FLASH_MODULE_ENABLED
#include "stm32f4xx_hal_flash.h"
#endif
#ifdef HAL_UART_MODULE_ENABLED
#include "stm32f4xx_hal_uart.h"
#endif

// ---------- 断言：bring-up 期开启（B-2.5-7） ----------
// 设为 0 可关闭；assert_failed 由 platform_stm32/diag.cc 提供（打印后降级，不死循环）。
#define USE_FULL_ASSERT 1U

#ifdef USE_FULL_ASSERT
#ifdef __cplusplus
extern "C" {
#endif
void assert_failed(uint8_t* file, uint32_t line);
#ifdef __cplusplus
}
#endif
#define assert_param(expr) ((expr) ? (void)0U : assert_failed((uint8_t*)__FILE__, __LINE__))
#else
#define assert_param(expr) ((void)0U)
#endif

#ifdef __cplusplus
}
#endif

#endif  // ROBOTCAR01_PLATFORM_STM32_STM32F4XX_HAL_CONF_H_
