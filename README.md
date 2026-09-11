# RobotCar01 底盘固件

## 构建要求（当前阶段）

| 工具 | 用途 | 状态 |
|---|---|---|
| cmake ≥ 3.24 | 构建 | brew 已装 |
| AppleClang / clang++ | host 单测编译 | CommandLineTools 自带 |
| GoogleTest | 单元测试框架 | brew 已装（googletest） |
| arm-none-eabi-gcc | 固件交叉编译 | **延后至 Iteration 008**（002.5 决策） |

## 命令行入口（全部不依赖 IDE）

在 `firmware/` 目录内执行：

```bash
# 1) 配置 + 构建 host 单测
cmake --preset host
cmake --build --preset host

# 2) 运行测试
ctest --preset host
```

预期：全部用例通过，退出码 0（当前 43 个用例，含 000 smoke 回归与 001 的 22 例）。

## 固件 target（预留，暂不构建）

交叉 target `robotcar01_fw` 默认 OFF。待 Iteration 002.5/008 安装
`arm-none-eabi-gcc` 并接入 CubeMX/HAL 生成代码后启用：

```bash
cmake -S . -B build/fw -DROBOTCAR01_BUILD_FW=ON \
  -DCMAKE_TOOLCHAIN_FILE=cmake/arm-none-eabi.cmake
cmake --build build/fw
```

## 目录约定

firmware/
├── CMakeLists.txt              # ① 构建系统入口（顶层：host 库/测试 + fw 预留）
├── CMakePresets.json           # ② 命令行预设（host）
├── README.md                   # ③ 使用说明 / 命令入口
├── .gitignore
├── app/                        # ④ 平台无关业务 + MCU OS Lite
│   ├── main.cc                 #    固件入口占位（fw target）
│   ├── chassis/                #    001 起：底盘纯数据/运动学/安全门/walking skeleton
│   └── mcu_os_lite/            #    002 起：MCU OS Lite 核心（clock/scheduler/health）
├── platform_stm32/             # ⑤ STM32 HAL 适配（仅固件）
│   └── README.md
├── host_fakes/                 # ⑥ 测试替身 fake 外设（仅 host，按迭代递增填充）
│   ├── README.md
│   ├── fake_output.h           #    001：仲裁后输出请求 sink
│   └── fake_clock.h            #    002：可步进单调时钟（驱动 scheduler）
├── cmake/                      # ⑦ 构建辅助
│   └── arm-none-eabi.cmake     #    交叉编译工具链文件
└── tests/                      # ⑧ 单元测试（仅 host，链接业务库与 OS 库）
    ├── CMakeLists.txt
    ├── smoke_test.cc           #    000：框架接入冒烟
    ├── kinematics_test.cc      #    001：差速/统一限幅/无效输入
    ├── safety_gate_test.cc     #    001：安全门四组合
    ├── walking_skeleton_test.cc  # 001：端到端四类断言
    ├── scheduler_test.cc       #    002：周期/相位/回绕/不追赶/顺延
    └── health_monitor_test.cc  #    002：overrun 停止/心跳窗口/锁存/恢复

边界不变量：`platform_stm32/` 不放业务代码；HAL 类型不得进入 `app/`；
fake 外设只进 `host_fakes/`（仅 host 编译）。host 分支现有两个静态库：
`robotcar01_chassis`（001 业务，`app/chassis/`）与 `robotcar01_mcu_os_lite`
（002 OS 核心，`app/mcu_os_lite/`，源列表变量 `ROBOTCAR01_MCU_OS_LITE_SOURCES`
为 fw 分支复用预留）。
