# host_fakes：测试替身 fake 外设（仅供 host 单测，不进入固件）

按迭代递增填充：
- `fake_output.h`：**Iteration 001** — 记录仲裁后输出请求（`WheelSpeedRequest`）写入序列的 sink。
- fake clock / fake PWM（PWM 域）：**Iteration 002/006** 起按需填充。
- fake encoder：**Iteration 004** 起按需填充。

约束：fake 只依赖 `app/` 公共头与标准库；不得引入 HAL。
