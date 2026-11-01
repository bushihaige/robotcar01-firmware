#include "platform_stm32/command_console.h"

#include "platform_stm32/platform_config.h"
#include "platform_stm32/uart_console.h"

namespace robotcar01::platform {
namespace {

constexpr uint32_t kMaxBytesPerPoll = 8U;

ConsoleCommand MapByte(uint8_t byte) {
  switch (byte) {
    case 's':
    case 'S':
      return ConsoleCommand::kStop;
    case '+':
    case '=':
      return ConsoleCommand::kTargetUp;
    case '-':
    case '_':
      return ConsoleCommand::kTargetDown;
    case ']':
      return ConsoleCommand::kOpenDutyUp;
    case '[':
      return ConsoleCommand::kOpenDutyDown;
    case 'o':
    case 'O':
      return ConsoleCommand::kModeToggle;
    case 'p':
    case 'P':
      return ConsoleCommand::kPrintOnce;
    case 'z':
    case 'Z':
      return ConsoleCommand::kZeroBaseline;
    case 'r':
    case 'R':
      return ConsoleCommand::kOpenDirectionToggle;
    case 'i':
    case 'I':
      return ConsoleCommand::kInfoPrint;
    case 'x':
    case 'X':
      return ConsoleCommand::kDebugStall;
    case 'e':
    case 'E':
      return ConsoleCommand::kEncoderRaw;
    case 'g':
    case 'G':
      return ConsoleCommand::kGpioScan;
    case '0':
      return ConsoleCommand::kWheelBoth;
    case '1':
      return ConsoleCommand::kWheelLeft;
    case '2':
      return ConsoleCommand::kWheelRight;
    default:
      return ConsoleCommand::kNone;
  }
}

}  // namespace

bool CommandConsole::Init() {
  stats_.Reset();
  return UartConsoleStartRx();
}

ConsoleCommand CommandConsole::Poll() {
  for (uint32_t index = 0U; index < kMaxBytesPerPoll; ++index) {
    uint8_t byte = 0U;
    if (!UartConsolePopByte(byte)) {
      return ConsoleCommand::kNone;
    }
    ++stats_.bytes;
    const ConsoleCommand command = MapByte(byte);
    if (command == ConsoleCommand::kNone) {
      ++stats_.unknown;
      continue;
    }
    return command;
  }
  return ConsoleCommand::kNone;
}

void PrintCommandHelp() {
  UartWriteLine("cmd: s=stop +=target up -=target down o=mode ]/[=open duty r=open dir 0/1/2=wheels p=print z=reset i=info x=stall-test e=enc-raw g=gpio-scan");
  UartWriteLine("cmd: start in CLOSED loop; target 0.00 m/s; press + to run");
}

}  // namespace robotcar01::platform
