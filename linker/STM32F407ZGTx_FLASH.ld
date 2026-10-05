/* RobotCar01 链接脚本：STM32F407ZGT6（144pin, 1MB Flash, 128KB SRAM + 64KB CCM）
 *
 * 本项目自写（B-2.5-6）：ST 官方仓库不存在 F407ZG*_FLASH.ld（仅有 IGHX/VGTX 版本），
 * 以官方 F407IGHX_FLASH.ld 为结构参照，依据 RM0090 存储映射编写。
 * 非 vendor 文件：允许修改，但需在 third_party/README.md 的"非 vendor"区登记。
 *
 * 符号契约（startup_stm32f407xx.s 硬依赖，评审 C-3）：
 *   _estack（MSP 初值与向量表首字）、_sidata/_sdata/_edata、_sbss/_ebss
 * .data 拷贝与 .bss 清零由 startup 承担，本脚本只提供符号与布局。
 * CCMRAM 段声明但本迭代不使用（启用需修改 startup 以拷贝初值）。
 */

ENTRY(Reset_Handler)

_estack = ORIGIN(RAM) + LENGTH(RAM);

_Min_Stack_Size = 0x800; /* 2 KB */
_Min_Heap_Size = 0x200;  /* 0.5 KB（本迭代无堆使用，留作 newlib 兜底） */

MEMORY
{
  CCMRAM (xrw) : ORIGIN = 0x10000000, LENGTH = 64K
  RAM    (xrw) : ORIGIN = 0x20000000, LENGTH = 128K
  FLASH  (rx)  : ORIGIN = 0x08000000, LENGTH = 1024K
}

SECTIONS
{
  /* 中断向量表必须位于 Flash 起始 */
  .isr_vector :
  {
    . = ALIGN(4);
    KEEP(*(.isr_vector))
    . = ALIGN(4);
  } >FLASH

  .text :
  {
    . = ALIGN(4);
    *(.text)
    *(.text*)
    *(.glue_7)
    *(.glue_7t)
    *(.eh_frame)
    KEEP(*(.init))
    KEEP(*(.fini))
    . = ALIGN(4);
    _etext = .;
  } >FLASH

  .rodata :
  {
    . = ALIGN(4);
    *(.rodata)
    *(.rodata*)
    . = ALIGN(4);
  } >FLASH

  .ARM.extab :
  {
    *(.ARM.extab* .gnu.linkonce.armextab.*)
  } >FLASH

  .ARM :
  {
    __exidx_start = .;
    *(.ARM.exidx*)
    __exidx_end = .;
  } >FLASH

  .preinit_array :
  {
    PROVIDE_HIDDEN(__preinit_array_start = .);
    KEEP(*(.preinit_array*))
    PROVIDE_HIDDEN(__preinit_array_end = .);
  } >FLASH

  .init_array :
  {
    PROVIDE_HIDDEN(__init_array_start = .);
    KEEP(*(SORT(.init_array.*)))
    KEEP(*(.init_array*))
    PROVIDE_HIDDEN(__init_array_end = .);
  } >FLASH

  .fini_array :
  {
    PROVIDE_HIDDEN(__fini_array_start = .);
    KEEP(*(SORT(.fini_array.*)))
    KEEP(*(.fini_array*))
    PROVIDE_HIDDEN(__fini_array_end = .);
  } >FLASH

  /* 已初始化数据：运行时地址在 RAM，加载地址在 Flash（_sidata 为加载地址） */
  _sidata = LOADADDR(.data);

  .data :
  {
    . = ALIGN(4);
    _sdata = .;
    *(.data)
    *(.data*)
    . = ALIGN(4);
    _edata = .;
  } >RAM AT> FLASH

  /* 未初始化数据（startup 负责清零） */
  .bss :
  {
    . = ALIGN(4);
    _sbss = .;
    __bss_start__ = _sbss;
    *(.bss)
    *(.bss*)
    *(COMMON)
    . = ALIGN(4);
    _ebss = .;
    __bss_end__ = _ebss;
  } >RAM

  /* 堆栈检查区：保证 RAM 至少留出 _Min_Heap_Size + _Min_Stack_Size */
  ._user_heap_stack :
  {
    . = ALIGN(8);
    PROVIDE(end = .);
    PROVIDE(_end = .);
    . = . + _Min_Heap_Size;
    . = . + _Min_Stack_Size;
    . = ALIGN(8);
  } >RAM

  /* CCMRAM：声明但本迭代不使用（NOLOAD，不占用 Flash 初值） */
  .ccmram (NOLOAD) :
  {
    . = ALIGN(4);
    _sccmram = .;
    *(.ccmram)
    *(.ccmram*)
    . = ALIGN(4);
    _eccmram = .;
  } >CCMRAM

  /* 未使用段的丢弃（保留 libc/libgcc 中真正被引用的部分，不用 ST 模板的激进 DISCARD） */
  /DISCARD/ :
  {
    *(.note.GNU-stack)
    *(.gnu_debuglink)
    *(.gnu.lto_*)
  }
}
