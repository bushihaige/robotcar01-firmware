# third_party：vendor 依赖（只读区，Iteration 002.5）

> **只读约定（INV-2.5-2）**：本目录下所有文件均为第三方原始文件，**不得手工修改**。
> `.gitattributes` 对其设置 `-text`（禁止行尾转换），并以 sha256 清单自证"未被改动"。
> 需要修改 vendor 行为时，改为在 `platform_stm32/` 侧覆写（例如自写 `stm32f4xx_hal_conf.h`、
> 自写 MSP 回调），而不是改本目录。

## 组件清单（版本锚定）

| 组件 | 来源仓库 | 锚定 commit | 许可证 | 说明 |
|---|---|---|---|---|
| `stm32f4-hal/` | `STMicroelectronics/stm32f4xx-hal-driver` | `1f6451c3e07728b4c830744de380e56bf5bc0026` | **BSD-3-Clause**（见 `stm32f4-hal/LICENSE.md`） | STM32F4 HAL 驱动源与头（含 `stm32f4xx_hal_conf_template.h` 作为参考模板） |
| `cmsis-device-f4/` | `STMicroelectronics/cmsis-device-f4` | `a833f4af71410f25b01468f976560d7ff63a2fc9` | **Apache-2.0**（见 `cmsis-device-f4/LICENSE.md`） | 器件头 `stm32f4xx.h`/`stm32f407xx.h`、`system_stm32f4xx.c`、`startup_stm32f407xx.s` |
| `cmsis-core/` | `ARM-software/CMSIS_5`（`CMSIS/Core/Include`） | `55b19837f5703e418ca37894d5745b1dc05e4c91` | **Apache-2.0**（见 `cmsis-core/LICENSE.txt`） | Cortex-M 核心头 `core_cm4.h`、`cmsis_gcc.h` 等 |

许可证原文随组件目录一并保留（分别位于上表括号中的路径）。

## 获取方式（可复现）

```bash
# 组件按 commit 锚定，逐文件从 raw.githubusercontent.com 拉取（避免整包 STM32CubeF4）
curl -sSLf "https://raw.githubusercontent.com/STMicroelectronics/stm32f4xx-hal-driver/1f6451c3e07728b4c830744de380e56bf5bc0026/Src/stm32f4xx_hal.c" -o stm32f4-hal/Src/stm32f4xx_hal.c
# 其余文件同理；完整文件清单与 sha256 见下表
```

升级流程：改锚定 commit → 重新拉取 → 用 `shasum -a 256` 更新下表 → 在对应迭代的
`implementation_log.md` 记录升级原因与影响面。

## 体积

约 3.1 MB（45 个文件），可直接入库，不使用 submodule（Windows 侧无需额外配置）。

## 非 vendor 文件（本项目自写，不属本目录）

- `../linker/STM32F407ZGTx_FLASH.ld`：本项目自写链接脚本（ST 官方仓库不存在 F407ZG*_FLASH.ld，
  仅有 IGHX/VGTX 版本）；以官方 `F407IGHX_FLASH.ld` 为结构参照，依据 RM0090 存储映射编写。**允许修改**。

## sha256 清单（vendor 自证）

| 文件 | sha256 |
|---|---|
| `cmsis-core/Include/cmsis_compiler.h` | `747c5edd04a88d4e228c7856750ec475908545c77464a162a305b38d9e362d82` |
| `cmsis-core/Include/cmsis_gcc.h` | `2f92cd832b6f58e4d4803b28d7340443f5e46b0f357a307cda76a42a0e530108` |
| `cmsis-core/Include/cmsis_version.h` | `184c19fd3ee73632edf35a0b4d49cd48be75fbf49e6ccb19d9db05fa83bea4b3` |
| `cmsis-core/Include/core_cm4.h` | `6b355b3974b13104dac0c987aff54795a2c64e6e268c9a0e058fb3443b3406a3` |
| `cmsis-core/Include/mpu_armv7.h` | `29206b52ee02290ed6f5a5415ebd4187de802cf176d9b3cb844390d8e5571372` |
| `cmsis-core/LICENSE.txt` | `b40930bbcf80744c86c46a12bc9da056641d722716c378f5659b9e555ef833e1` |
| `cmsis-device-f4/Include/stm32f407xx.h` | `ae1ab57d8ab750f64f04fd73476bc2760c761e18e45bde648e0bc350c9a1d9ac` |
| `cmsis-device-f4/Include/stm32f4xx.h` | `a19edea6b2f4df2ab3c567782c4d313a52319c4303bd56ad7dd1d0e077014657` |
| `cmsis-device-f4/Include/system_stm32f4xx.h` | `02c067d5a135f540c03215dc834b1e60a97bbd4e55bfd3994b0c82a15ea3012a` |
| `cmsis-device-f4/LICENSE.md` | `dadb755f51d36614173b28c5790cb4a991e8f4cc822e5b634fd66a4f4145824d` |
| `cmsis-device-f4/Source/Templates/gcc/startup_stm32f407xx.s` | `430f14fd00db0f7aa88f354aee54064efb582ddc4cc264fb8e435dc7d7a2f53d` |
| `cmsis-device-f4/Source/Templates/system_stm32f4xx.c` | `2207561f907415c4bd91d640755679f8d2feb9b609bd0a6d521e6e76ee463b85` |
| `stm32f4-hal/Inc/Legacy/stm32_hal_legacy.h` | `8c2d2feb90e8ed88fe5756f3f1616a2b67aa1cd0858da8b130ed69607688c8c0` |
| `stm32f4-hal/Inc/stm32f4xx_hal.h` | `6942f40453fee331e026179cfac4c0f24ffe6313fedcb68af9c3382f702dbaf5` |
| `stm32f4-hal/Inc/stm32f4xx_hal_conf_template.h` | `fea2676745945e8cf5ded3e1ea32000c68c5a3e33018578703024964336ca243` |
| `stm32f4-hal/Inc/stm32f4xx_hal_cortex.h` | `d96dba22903abda49d66cad29ac0c49357e6a27cf9cb03dfd8c6db71b61a7fca` |
| `stm32f4-hal/Inc/stm32f4xx_hal_def.h` | `c8eb3f7b393f8c5a0c32e16ccfa101c83ddb59e96bd42239b8a5c5d4da83e97f` |
| `stm32f4-hal/Inc/stm32f4xx_hal_dma.h` | `71c38dfd65c5415b388375147e5317c7ac018b39893d6fd0ac1c897f4313329c` |
| `stm32f4-hal/Inc/stm32f4xx_hal_dma_ex.h` | `3ee15423d73ac394de39e7b65821775966389b13749ff0849894548ab6968676` |
| `stm32f4-hal/Inc/stm32f4xx_hal_exti.h` | `61e8f76bdb45dedf38fef592504e10418ca410952c7a209cd271b45ac835833a` |
| `stm32f4-hal/Inc/stm32f4xx_hal_flash.h` | `10be06c6e06fcf4eb11e236641948aed861898fea235c606ae221fa72bc46638` |
| `stm32f4-hal/Inc/stm32f4xx_hal_flash_ex.h` | `7c206f5dbed3c147a6f090ab808933132a1b06b8339623ad7174fd0a222e981c` |
| `stm32f4-hal/Inc/stm32f4xx_hal_flash_ramfunc.h` | `f18a6eabc99f56483566f9231ca33c780afa98bc93929e60bb419b5e544eee2a` |
| `stm32f4-hal/Inc/stm32f4xx_hal_gpio.h` | `0b40fc83363b64d3c7396a202be7a914ceaa7735ab0fa81d566ed320102b23fb` |
| `stm32f4-hal/Inc/stm32f4xx_hal_gpio_ex.h` | `c01a38c283c48ed68e23ed3ef067c8610a46a98409c16b11cb6a43fd7c650634` |
| `stm32f4-hal/Inc/stm32f4xx_hal_pwr.h` | `d6f6abc4af3d5a1ca59d886f5a9154e44f257a43af77c7f18173f160825ed6b5` |
| `stm32f4-hal/Inc/stm32f4xx_hal_pwr_ex.h` | `1794b88c5e080858c0205a2e94340c0250ac9171b3a3f01cad6d050005a1b09d` |
| `stm32f4-hal/Inc/stm32f4xx_hal_rcc.h` | `eb6911d964b1cf8a83993b19c8feb9845e8793d7c0e5d643390b53207c8b078e` |
| `stm32f4-hal/Inc/stm32f4xx_hal_rcc_ex.h` | `49f0c8068524e1de248738530cff3bbe630dc4784b6e8b5568fbc7e1a154f772` |
| `stm32f4-hal/Inc/stm32f4xx_hal_uart.h` | `42528bc9caa6e2fe96b7e6d7b3fcdf35d8d85ae38d33195a233e18a072f157a1` |
| `stm32f4-hal/LICENSE.md` | `376c82bcef0be563bf59b93d05cfdd4b6295e4660b55b31dd8a883bf1d6637f7` |
| `stm32f4-hal/Src/stm32f4xx_hal.c` | `97e715cda2a34bd564b59ce4ba9b16bd29651f8f229161105aaec666d76ea91a` |
| `stm32f4-hal/Src/stm32f4xx_hal_cortex.c` | `9aaa0f19137d700616bec01a5a806786c24e3f3386120365c5d2addaa4055505` |
| `stm32f4-hal/Src/stm32f4xx_hal_dma.c` | `a345a1b1fe0221c827d5ad21d5a0f5387296bc2c075e06e5b8539c23da5678e4` |
| `stm32f4-hal/Src/stm32f4xx_hal_dma_ex.c` | `90cafb5a5b17a46b478770ef4abdc9a641cb395419abbb80f288138a63d44582` |
| `stm32f4-hal/Src/stm32f4xx_hal_exti.c` | `f56651f86e5d89e8bba3bcb331353e00f291db15c902927a205d81bbe8951e6d` |
| `stm32f4-hal/Src/stm32f4xx_hal_flash.c` | `8bf12eeb6e6cfc12a17587ed6d761724e0835a4a3dc0f6b14a8d1ca068bcb55b` |
| `stm32f4-hal/Src/stm32f4xx_hal_flash_ex.c` | `51a70deb9b19dd43f6f202430fae052f2deb26c39232c09c79230c8d8a61f773` |
| `stm32f4-hal/Src/stm32f4xx_hal_flash_ramfunc.c` | `d3b426e8cb9d617f9064d2b0cea2a1d02b7bb6a74cba3f094779d6e922e477e3` |
| `stm32f4-hal/Src/stm32f4xx_hal_gpio.c` | `408908b15477b6ce2b0b9703caabcb053293d98f2f109d6e6940e076a6a3902b` |
| `stm32f4-hal/Src/stm32f4xx_hal_pwr.c` | `d779468a95df34421bf6c8417136a79aa910753381a644afa9ed10b185ae18b5` |
| `stm32f4-hal/Src/stm32f4xx_hal_pwr_ex.c` | `4409dddc0d2019f918d0ddebecf659f06d4de3f9aa28566621a16ea47203331f` |
| `stm32f4-hal/Src/stm32f4xx_hal_rcc.c` | `f3ac468e2f415adb74281107c0dec1448008950b64a7bb11c14cbe3ee1a936de` |
| `stm32f4-hal/Src/stm32f4xx_hal_rcc_ex.c` | `526324811fa113c3eadfa84cd24ea5e607ad8e8bf003a2085456c04b734ca25e` |
| `stm32f4-hal/Src/stm32f4xx_hal_uart.c` | `a74ed3a9a13a4fcbaada9e6f88f7d4fcabe2a75852a6ce4c09f8d4314a2e851a` |
