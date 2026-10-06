# RobotCar01 底盘固件

## 构建要求（当前阶段）

| 工具 | 用途 | 状态 |
|---|---|---|
| cmake ≥ 3.24 | 构建 | brew 已装 |
| AppleClang / clang++ | host 单测编译 | CommandLineTools 自带 |
| GoogleTest | 单元测试框架 | brew 已装（googletest） |
| arm-none-eabi-gcc | 固件交叉编译 | **Iteration 002.5 起启用**；Mac 侧已装（brew，用于交叉编译预检）；烧录在 Windows 侧（ST-Link） |

## 命令行入口（全部不依赖 IDE）

在 `firmware/` 目录内执行：

```bash
# 1) 配置 + 构建 host 单测
cmake --preset host
cmake --build --preset host

# 2) 运行测试
ctest --preset host
```

预期：全部用例通过，退出码 0（当前 **44** 个用例：000 smoke 1 + 001 的 22 + 002 的 22，其中 scheduler 10 / health_monitor 12）。

## 固件 target（Iteration 002.5 起启用）

交叉 target `robotcar01_fw` 默认 OFF。002.5 起启用：HAL/CMSIS 以 **vendor 源码**引入
（不使用 CubeMX 生成物），fw 入口为 `platform_stm32/main_stm32.cc`。命令以 CMake preset 固化
（`cmake --preset fw && cmake --build --preset fw`，见 `CMakePresets.json`）；等价的手工命令为：

```bash
cmake -S . -B build/fw -DROBOTCAR01_BUILD_FW=ON \
  -DCMAKE_TOOLCHAIN_FILE="$PWD/cmake/arm-none-eabi.cmake"
cmake --build build/fw
```

## 协议与 host 调试工具（Iteration 003 起）

外部控制边界的线格式、错误码与上位机契约详见
`docs/design docs/robotcar01_chassis_control/review_exec/iterations/003-command-and-feedback/command-and-feedback_detailed_design.md`。
要点速览（**版式冻结，改版必须升 `version`**）：

```text
offset 0   magic 'R''C''0''1'（逐字节比较，禁止对整数常量 memcmp）
offset 4   version = 1
offset 5   msg_type: 0x01 Motion | 0x02 Stop | 0x81 FeedbackStatus | 0x82 FeedbackDiag
offset 6   flags（保留，必须为 0；非 0 按 kRejectedFlag 拒绝）
offset 7   payload_len（Motion=13, Stop=0, Status=64, Diag=200）
offset 8   seq（uint32 LE；会话内必须严格递增，半程规则见详设）
offset 12  payload
...        crc16 = CRC-16/CCITT-FALSE(poly 0x1021, init 0xFFFF, 不反射, 无末异或)，uint32 LE 线序
```

Motion 载荷（13 B，小端）：`v_mps`(4) | `omega_radps`(4) | `run_requested`(1) | `reserved0`(1=0) |
`lease_ms`(2) | `send_age_ms`(1)。`run_requested` **不是运行许可**：会话首帧永不使能，必须由
上位机先发一帧"不请求运行"的有效命令（或显式 Stop 帧）完成 arm，最终启动裁决属 Iteration 007。

### 调试工具

```bash
# 每行一个十六进制命令帧；stdout 出二进制反馈帧，stderr 出可解析统计行
printf '524330310101000d01000000cdcc4c3e0000000000002c0100<b5><a6>\n' | \
  ./build/host/tools/robotcar01_loopback --frames 5 > feedback.bin 2> stats.txt

# 端到端自检（也被注册为 ctest 用例 loopback_end_to_end）
python3 scripts/check_feedback_stream.py ./build/host/tools/robotcar01_loopback
```

工具使用**内部虚拟时钟**（每行输入推进 1 ms），输出帧数只由输入行数决定，不依赖墙钟、不随机器负载变化；
不提供墙钟模式（真机联调由 008 的 USB 接线条承担）。工具是 host 证据，**不是** USB 真机链路证据。

## 分支与 PR 流程（GitHub stacked PR，自 Iteration 002.5 起）

**硬规则：不得把迭代分支直接合并/推送到 `main`。** 每轮迭代一条分支 + 一个 PR，PR 的 base 指向
**上一轮迭代的分支**，形成一条 stack；只有最底层的 PR 才以 `main` 为 base。

当前 stack（GitHub stack #3）：

```text
main
└── iteration-002-clock-and-scheduler   → PR #1
    └── iteration-002.5-minimal-bringup → PR #2   ← 当前
```

分支命名：`iteration-<序号>-<owner-module>`（如 `iteration-003-command-and-feedback`）。

常用命令（需 `gh` 已认证；`gh extension install github/gh-stack` 一次即可）：

```bash
gh stack view                      # 查看当前 stack
gh stack checkout <PR号|分支名>     # 检出并挂载本地 tracking
gh stack add <新分支>               # 在当前栈顶再叠一层（自动置于当前分支之上）
gh stack submit                    # 推送并创建/更新整条 stack 的 PR
gh stack sync                      # 与远端同步（底层合并后自动 rebase/retarget）
gh stack rebase                    # 手动重排/变基
```

约定：新迭代分支应从**上一轮分支的 head** 切出（不要从 main 切），并设置
`git config branch.<新分支>.gh-merge-base <上一轮分支>`，这样 `gh pr create` 会自动使用正确的 base。
合并顺序自底向上；底层 PR 合并后，上层 PR 的 base 由 GitHub/gh stack 自动改指 `main`。

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
