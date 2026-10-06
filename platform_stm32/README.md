# STM32 HAL 适配（Iteration 002.5 起实现；此目录不放业务代码）

- 职责：HAL/外设适配层——时钟树（8 MHz HSE → 168 MHz）、USART1（PA9/PA10 @115200）、
  后续迭代的 PWM/编码器/GPIO/USB/IWDG 适配。
- 边界不变量：业务代码不进本目录（S-000-1）；`app/` 保持平台无关且本迭代不改动（INV-2.5-1）。
- 本迭代交付：`platform_config.h`（参数 SSOT）、`clock_init.*`、`uart_console.*`、
  `main_stm32.cc`（fw 入口，取代原 `app/main.cc` 占位）。
- vendor 依赖位于 `third_party/stm32f4/`（只读，见其 README 的来源与许可证登记）。
- 构建与烧录：Mac 侧交叉编译预检；Windows 侧构建 + ST-Link 烧录 + 串口采证
  （链路需求见 `docs/design docs/robotcar01_chassis_control/review_exec/iterations/002.5-minimal-board-bringup/windows-build-flash-chain.md`）。
- 注：`app/main.cc` 在本迭代后不再参与任何 target 构建（fw 入口改为本目录），
  标记为孤儿文件，去留由 006.5/008 决定。
