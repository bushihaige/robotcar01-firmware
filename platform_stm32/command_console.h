// 串口命令解析（Iteration 006.5）
//
// 通道：USART1 RX 单字节中断 → 静态环 → 主循环 Poll 解析（详设 §5.4）。
// 只做字节→命令的单字符映射；命令语义与状态输出在 BoardApp。
// 未知字节丢弃并计数（诊断）；不做行缓冲、不分配内存。

#ifndef ROBOTCAR01_PLATFORM_STM32_COMMAND_CONSOLE_H_
#define ROBOTCAR01_PLATFORM_STM32_COMMAND_CONSOLE_H_

#include <cstdint>

namespace robotcar01::platform {

enum class ConsoleCommand : uint8_t {
  kNone = 0,
  kStop,          // s：目标 0、开环占空比 0
  kTargetUp,      // +：闭环目标 +0.05 m/s
  kTargetDown,    // -：闭环目标 -0.05 m/s
  kOpenDutyUp,    // ]：开环占空比 +10%
  kOpenDutyDown,  // [：开环占空比 -10%
  kModeToggle,    // o：闭环/开环切换
  kPrintOnce,     // p：立即打印一次状态
  kZeroBaseline,  // z：清编码器基线与控制器状态
  kOpenDirectionToggle,  // r：开环方向取反（标定时验证两个半桥）
  kInfoPrint,     // i：打印固件构建信息与生效配置（替代易丢的启动横幅）
  kDebugStall,    // x：主循环忙等 120 ms（仅目标为 0 时生效；用于实测主循环独立监护）
  kEncoderRaw,    // e：打印编码器原始计数与引脚电平（判断接线/悬空）
  kGpioScan,      // g：打印 GPIOA/GPIOB 全部输入位（定位信号实际接在哪根脚）
  kWheelBoth,     // 0：双轮
  kWheelLeft,     // 1：仅左轮（PB1/PB8/PB9 + PA6/PA7 组）
  kWheelRight,    // 2：仅右轮（PB0/PB6/PB7 + PA0/PA1 组，2026-10-25 已接线）
};

struct ConsoleStats {
  uint32_t bytes = 0;
  uint32_t unknown = 0;
  uint32_t overflow = 0;
  void Reset() { *this = ConsoleStats{}; }
};

class CommandConsole {
 public:
  // 启动 RX 中断接收；失败返回 false（此时仍可打印状态，只是不能收命令）。
  bool Init();

  // 取一个命令（每次最多消费环中若干字节）；无命令返回 kNone。
  ConsoleCommand Poll();

  const ConsoleStats& stats() const { return stats_; }

 private:
  ConsoleStats stats_{};
};

// 打印命令帮助（启动时一次）。
void PrintCommandHelp();

}  // namespace robotcar01::platform

#endif  // ROBOTCAR01_PLATFORM_STM32_COMMAND_CONSOLE_H_
