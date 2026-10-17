# host_fakes：测试替身 fake 外设（仅供 host 单测，不进入固件）

按迭代递增填充：
- ~~`fake_output.h`~~：Iteration 001 的速度域 sink，已随 walking_skeleton 于 **Iteration 006 退役**（由 `fake_pwm_sink.h` 取代）。
- `fake_pwm_sink.h`：**Iteration 006** — 记录仲裁后 PWM 请求（方向/占空比/模式），用于证明唯一写入口。
- `fake_motor_plant.h`：**Iteration 006** — 单轮一阶电机模型（独立参数，含死区与带载换向检测）。
- `fake_clock.h`：**Iteration 002** — 可步进单调微秒时钟（`SetUs`/`AdvanceUs`），驱动
  `robotcar01::mcu_os_lite::CooperativeScheduler` 的 tick 累计/回绕/任务耗时模拟。
- fake PWM（PWM 域）：**Iteration 006** 起按需填充。
- fake encoder：**Iteration 004** 起按需填充。

约束：fake 只依赖 `app/` 公共头与标准库；不得引入 HAL。
