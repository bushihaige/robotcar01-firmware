#include "platform_stm32/uart_console.h"

#include "platform_stm32/platform_config.h"
#include "stm32f4xx_hal.h"

namespace robotcar01::platform {
namespace {

UART_HandleTypeDef g_uart1;
bool g_ready = false;
bool g_initialized = false;

// ---- Iteration 006.5：RX 单字节中断环（ISR 写 head，主循环写 tail） ----
uint8_t g_rx_byte = 0U;
uint8_t g_rx_ring[kConsoleRxRingSize];
volatile uint32_t g_rx_head = 0U;
volatile uint32_t g_rx_tail = 0U;
volatile uint32_t g_rx_overflow = 0U;
bool g_rx_started = false;

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

void UartWriteI32(int32_t value) {
  if (value < 0) {
    WriteRaw("-", 1U);
    // -INT32_MIN 溢出：先转成负数取反的等价写法
    const uint32_t magnitude =
        static_cast<uint32_t>(-(static_cast<int64_t>(value)));
    UartWriteU32(magnitude);
    return;
  }
  UartWriteU32(static_cast<uint32_t>(value));
}

bool UartConsoleStartRx() {
  if (!g_ready) {
    return false;
  }
  g_rx_started = (HAL_UART_Receive_IT(&g_uart1, &g_rx_byte, 1U) == HAL_OK);
  return g_rx_started;
}

bool UartConsolePopByte(uint8_t& out) {
  if (g_rx_tail == g_rx_head) {
    return false;
  }
  out = g_rx_ring[g_rx_tail];
  g_rx_tail = (g_rx_tail + 1U) % kConsoleRxRingSize;
  return true;
}

uint32_t UartConsoleRxOverflowCount() { return g_rx_overflow; }

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

  // 006.5：RX 命令通道需要 USART1 中断
  HAL_NVIC_SetPriority(USART1_IRQn, 5U, 0U);
  HAL_NVIC_EnableIRQ(USART1_IRQn);
}

extern "C" void HAL_UART_MspDeInit(UART_HandleTypeDef* huart) {
  if (huart == nullptr || huart->Instance != USART1) {
    return;
  }
  __HAL_RCC_USART1_CLK_DISABLE();
  HAL_GPIO_DeInit(GPIOA, GPIO_PIN_9 | GPIO_PIN_10);
}

// ---- Iteration 006.5：RX 单字节中断 ----
// 注意：定义在 platform 命名空间内以保证 C 链接名（HAL 覆盖）且可直接访问匿名命名空间的状态。
namespace robotcar01::platform {
extern "C" void HAL_UART_RxCpltCallback(UART_HandleTypeDef* huart) {
  if (huart == nullptr || huart->Instance != USART1) {
    return;
  }
  const uint32_t next = (g_rx_head + 1U) % kConsoleRxRingSize;
  if (next == g_rx_tail) {
    g_rx_overflow = g_rx_overflow + 1U;  // 环满：丢弃新字节并计数（不覆盖未消费数据）
  } else {
    g_rx_ring[g_rx_head] = g_rx_byte;
    g_rx_head = next;
  }
  (void)HAL_UART_Receive_IT(huart, &g_rx_byte, 1U);
}

extern "C" void USART1_IRQHandler(void) { HAL_UART_IRQHandler(&g_uart1); }
}  // namespace robotcar01::platform
