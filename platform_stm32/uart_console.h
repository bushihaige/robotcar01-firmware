// USART1 控制台（Iteration 002.5）
//
// 观测通道（B-2.5-5）：USART1 PA9(TX)/PA10(RX) AF7 @115200 8N1。
// MSP 回调（HAL_UART_MspInit）在本文件实现——HAL 里它是 __weak 空实现，
// 不覆盖就不会开 GPIO/USART1 时钟（评审 C-5/H-4）。
// 仅使用整数格式：nano.specs 下避免 %f（无需 -u _printf_float）。

#ifndef ROBOTCAR01_PLATFORM_STM32_UART_CONSOLE_H_
#define ROBOTCAR01_PLATFORM_STM32_UART_CONSOLE_H_

#include <cstdint>

namespace robotcar01::platform {

// 初始化 USART1（需在时钟配置完成之后调用：BRR 依赖 PCLK2）。
bool UartConsoleInit(uint32_t baud);

// 是否可用（初始化成功且未降级）。
bool UartConsoleReady();

// 写出 NUL 结尾字符串（阻塞发送，固定超时）。
void UartWrite(const char* text);

// 行输出（自动追加 \r\n）。
void UartWriteLine(const char* text);

// 十进制 / 十六进制输出与带标签形式。
void UartWriteU32(uint32_t value);
void UartWriteHex32(uint32_t value);
void UartWriteLabelU32(const char* label, uint32_t value);
void UartWriteLabelHex(const char* label, uint32_t value);

}  // namespace robotcar01::platform

#endif  // ROBOTCAR01_PLATFORM_STM32_UART_CONSOLE_H_
