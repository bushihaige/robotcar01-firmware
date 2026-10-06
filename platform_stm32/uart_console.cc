#include "platform_stm32/uart_console.h"

#include "platform_stm32/platform_config.h"
#include "stm32f4xx_hal.h"

namespace robotcar01::platform {
namespace {

UART_HandleTypeDef g_uart1;
bool g_ready = false;
bool g_initialized = false;

void WriteRaw(const char* data, uint16_t length) {
  if (!g_initialized || data == nullptr || length == 0U) {
    return;
  }
  // 阻塞发送：本迭代无实时性约束（详设 §时序）。
  (void)HAL_UART_Transmit(&g_uart1, reinterpret_cast<uint8_t*>(const_cast<char*>(data)), length,
                          kConsoleTxTimeoutMs);
}

}  // namespace

bool UartConsoleInit(uint32_t baud) {
  g_uart1.Instance = USART1;
  g_uart1.Init.BaudRate = baud;
  g_uart1.Init.WordLength = UART_WORDLENGTH_8B;
  g_uart1.Init.StopBits = UART_STOPBITS_1;
  g_uart1.Init.Parity = UART_PARITY_NONE;
  g_uart1.Init.Mode = UART_MODE_TX_RX;
  g_uart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  g_uart1.Init.OverSampling = UART_OVERSAMPLING_16;  // 84 MHz/115200 → BRR=0x2D9，误差 0.02%
  g_initialized = true;                              // 先置位：失败也要能打印错误
  g_ready = (HAL_UART_Init(&g_uart1) == HAL_OK);
  return g_ready;
}

bool UartConsoleReady() { return g_ready; }

void UartWrite(const char* text) {
  if (text == nullptr) {
    return;
  }
  uint16_t length = 0U;
  while (text[length] != '\0') {
    ++length;
  }
  WriteRaw(text, length);
}

void UartWriteLine(const char* text) {
  UartWrite(text);
  WriteRaw("\r\n", 2U);
}

void UartWriteU32(uint32_t value) {
  char buffer[11];
  uint32_t index = sizeof(buffer);
  if (value == 0U) {
    WriteRaw("0", 1U);
    return;
  }
  while (value > 0U && index > 0U) {
    buffer[--index] = static_cast<char>('0' + (value % 10U));
    value /= 10U;
  }
  WriteRaw(&buffer[index], static_cast<uint16_t>(sizeof(buffer) - index));
}

void UartWriteHex32(uint32_t value) {
  constexpr char kDigits[] = "0123456789ABCDEF";
  char buffer[10];
  buffer[0] = '0';
  buffer[1] = 'x';
  for (uint32_t i = 0U; i < 8U; ++i) {
    const uint32_t shift = (7U - i) * 4U;
    buffer[2U + i] = kDigits[(value >> shift) & 0xFU];
  }
  WriteRaw(buffer, sizeof(buffer));
}

void UartWriteLabelU32(const char* label, uint32_t value) {
  UartWrite(label);
  UartWriteU32(value);
}

void UartWriteLabelHex(const char* label, uint32_t value) {
  UartWrite(label);
  UartWriteHex32(value);
}

}  // namespace robotcar01::platform

// ---- HAL MSP 回调（覆盖 __weak 空实现） ----
extern "C" void HAL_UART_MspInit(UART_HandleTypeDef* huart) {
  if (huart == nullptr || huart->Instance != USART1) {
    return;
  }
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_USART1_CLK_ENABLE();

  GPIO_InitTypeDef gpio_init{};
  gpio_init.Pin = GPIO_PIN_9 | GPIO_PIN_10;  // PA9=TX, PA10=RX
  gpio_init.Mode = GPIO_MODE_AF_PP;
  gpio_init.Pull = GPIO_PULLUP;
  gpio_init.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  gpio_init.Alternate = GPIO_AF7_USART1;
  HAL_GPIO_Init(GPIOA, &gpio_init);
}

extern "C" void HAL_UART_MspDeInit(UART_HandleTypeDef* huart) {
  if (huart == nullptr || huart->Instance != USART1) {
    return;
  }
  __HAL_RCC_USART1_CLK_DISABLE();
  HAL_GPIO_DeInit(GPIOA, GPIO_PIN_9 | GPIO_PIN_10);
}
