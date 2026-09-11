// MCU OS Lite 协作调度器测试（Iteration 002）
//
// 覆盖（映射 detailed design 验收 1/2/4 与测试策略表）：
//   - 周期 1/5/100 tick 在激活点网格上执行（phase=0 → tick0 起每 period 一次）；
//   - 相位 1/2/period-1 的逐 tick 轨迹与冻结语义一致；
//   - tick 跨 2^32 回绕后到期语义不变；
//   - 一次延迟跨多周期不追赶（每循环至多一次）；
//   - overrun 任务顺延：不立即再次调度；
//   - 空闲运行不崩、stats 更新；
//   - Register 校验：非法参数/满员/Seal 后拒绝。
// fake clock 推进与断言直接使用 host_fakes/fake_clock.h。

#include <cstdint>

#include <gtest/gtest.h>

#include "app/mcu_os_lite/clock.h"
#include "app/mcu_os_lite/health_monitor.h"
#include "app/mcu_os_lite/scheduler.h"
#include "app/mcu_os_lite/task_descriptor.h"
#include "host_fakes/fake_clock.h"

namespace robotcar01::mcu_os_lite {
namespace {

using host_fakes::FakeClock;

constexpr uint32_t kBaseTickUs = 1000;  // 1 ms

struct TaskContext {
  uint32_t run_count = 0;
};

void CountingRun(void* opaque) {
  auto* ctx = static_cast<TaskContext*>(opaque);
  ++ctx->run_count;
}

// 固定耗时任务：在 run_fn 内推进 fake clock（模拟"长任务"，host 卡死/overrun 模型）。
struct SlowContext {
  uint32_t run_count = 0;
  host_fakes::FakeClock* clock = nullptr;
  uint64_t consume_us = 0;
};

void SlowRun(void* opaque) {
  auto* ctx = static_cast<SlowContext*>(opaque);
  ++ctx->run_count;
  if (ctx->clock != nullptr && ctx->consume_us > 0u) {
    ctx->clock->AdvanceUs(ctx->consume_us);
  }
}

TaskConfig MakeConfig(uint32_t period_ticks, uint32_t phase_ticks = 0,
                      uint32_t wcet_budget_us = 0,
                      TaskCriticality criticality = TaskCriticality::kNonCritical) {
  TaskConfig cfg;
  cfg.name = "test_task";
  cfg.period_ticks = period_ticks;
  cfg.phase_ticks = phase_ticks;
  cfg.wcet_budget_us = wcet_budget_us;
  cfg.criticality = criticality;
  return cfg;
}

// 从当前 tick 起再推进 n 个基础 tick（每次 1 tick + 一次 TickLoopOnce）。
void AdvanceTicks(host_fakes::FakeClock& clock, CooperativeScheduler& scheduler, uint32_t n) {
  for (uint32_t i = 0; i < n; ++i) {
    ASSERT_TRUE(clock.AdvanceUs(kBaseTickUs));
    scheduler.TickLoopOnce();
  }
}

TEST(SchedulerTest, RegisterRejectsInvalidConfig) {
  FakeClock clock;
  TaskSet task_set;
  TaskContext ctx;
  constexpr uint32_t kWindow = 100;

  TaskConfig bad_period = MakeConfig(0);
  EXPECT_FALSE(task_set.Register(bad_period, CountingRun, &ctx, kWindow));

  TaskConfig bad_phase = MakeConfig(5, 5);  // phase >= period
  EXPECT_FALSE(task_set.Register(bad_phase, CountingRun, &ctx, kWindow));

  TaskConfig bad_fn = MakeConfig(5);
  EXPECT_FALSE(task_set.Register(bad_fn, nullptr, &ctx, kWindow));

  TaskConfig bad_critical =
      MakeConfig(200, 0, 0, TaskCriticality::kCritical);  // period > window（INV-OS-7）
  EXPECT_FALSE(task_set.Register(bad_critical, CountingRun, &ctx, kWindow));

  TaskConfig ok = MakeConfig(5);
  EXPECT_TRUE(task_set.Register(ok, CountingRun, &ctx, kWindow));
  EXPECT_EQ(task_set.size(), 1u);
}

TEST(SchedulerTest, SealPreventsFurtherRegistration) {
  FakeClock clock;
  TaskSet task_set;
  HealthMonitor health;
  health.Init(HealthConfig{}, task_set);
  TaskContext ctx;
  CooperativeScheduler scheduler;
  scheduler.Init(TickConfig{kBaseTickUs}, clock, task_set, health);

  // 生命周期：Init 后、首次 TickLoopOnce 前仍允许注册。
  TaskConfig pre_cfg = MakeConfig(5);
  EXPECT_TRUE(task_set.Register(pre_cfg, CountingRun, &ctx, 100));
  scheduler.TickLoopOnce();  // 首次调用 → Seal

  TaskConfig cfg = MakeConfig(5);
  EXPECT_FALSE(task_set.Register(cfg, CountingRun, &ctx, 100));  // Seal 后拒绝
}

TEST(SchedulerTest, PeriodicTaskRunsOnActivationGrid) {
  FakeClock clock;
  TaskSet task_set;
  TaskContext ctx;
  TaskConfig cfg = MakeConfig(5);  // phase=0, period=5
  ASSERT_TRUE(task_set.Register(cfg, CountingRun, &ctx, 100));
  HealthMonitor health;
  health.Init(HealthConfig{}, task_set);
  CooperativeScheduler scheduler;
  scheduler.Init(TickConfig{kBaseTickUs}, clock, task_set, health);

  // tick0 首轮：phase=0 首激活点即 tick0（冻结语义 D-OS-003）。
  scheduler.TickLoopOnce();
  EXPECT_EQ(ctx.run_count, 1u);

  // tick1..4 不到期。
  AdvanceTicks(clock, scheduler, 4);
  EXPECT_EQ(ctx.run_count, 1u);
  EXPECT_EQ(scheduler.tick_count(), 4u);

  // tick5 到期。
  AdvanceTicks(clock, scheduler, 1);
  EXPECT_EQ(ctx.run_count, 2u);
  EXPECT_EQ(scheduler.tick_count(), 5u);

  // tick10 到期（第 3 次）。
  AdvanceTicks(clock, scheduler, 5);
  EXPECT_EQ(ctx.run_count, 3u);
}

TEST(SchedulerTest, PeriodOneRunsEveryTick) {
  FakeClock clock;
  TaskSet task_set;
  TaskContext ctx;
  TaskConfig cfg = MakeConfig(1);  // period=1 → 每 tick 激活（含 tick0）
  ASSERT_TRUE(task_set.Register(cfg, CountingRun, &ctx, 100));
  HealthMonitor health;
  health.Init(HealthConfig{}, task_set);
  CooperativeScheduler scheduler;
  scheduler.Init(TickConfig{kBaseTickUs}, clock, task_set, health);

  scheduler.TickLoopOnce();  // tick0
  AdvanceTicks(clock, scheduler, 4);
  EXPECT_EQ(ctx.run_count, 5u);  // tick0..4 共 5 次
  EXPECT_EQ(scheduler.tick_count(), 4u);
}

TEST(SchedulerTest, PeriodHundredRunsEveryHundredTicks) {
  FakeClock clock;
  TaskSet task_set;
  TaskContext ctx;
  TaskConfig cfg = MakeConfig(100);  // period=100 tick（对应 arch 健康/监督节奏）
  ASSERT_TRUE(task_set.Register(cfg, CountingRun, &ctx, 100));
  HealthMonitor health;
  health.Init(HealthConfig{}, task_set);
  CooperativeScheduler scheduler;
  scheduler.Init(TickConfig{kBaseTickUs}, clock, task_set, health);

  scheduler.TickLoopOnce();  // tick0 首激活
  AdvanceTicks(clock, scheduler, 99);
  EXPECT_EQ(scheduler.tick_count(), 99u);
  EXPECT_EQ(ctx.run_count, 1u);  // tick99 未到期

  AdvanceTicks(clock, scheduler, 1);  // tick100
  EXPECT_EQ(ctx.run_count, 2u);

  AdvanceTicks(clock, scheduler, 99);  // tick199 未到期
  EXPECT_EQ(ctx.run_count, 2u);
  AdvanceTicks(clock, scheduler, 1);  // tick200
  EXPECT_EQ(ctx.run_count, 3u);
}

TEST(SchedulerTest, PhaseShiftsActivationGrid) {
  FakeClock clock;
  TaskSet task_set;
  HealthMonitor health;
  health.Init(HealthConfig{}, task_set);

  // 三个任务：phase=1/2/4（period=5），验证各自网格。
  TaskContext ctx1, ctx2, ctx4;
  TaskConfig cfg1 = MakeConfig(5, 1);
  TaskConfig cfg2 = MakeConfig(5, 2);
  TaskConfig cfg4 = MakeConfig(5, 4);
  ASSERT_TRUE(task_set.Register(cfg1, CountingRun, &ctx1, 100));
  ASSERT_TRUE(task_set.Register(cfg2, CountingRun, &ctx2, 100));
  ASSERT_TRUE(task_set.Register(cfg4, CountingRun, &ctx4, 100));

  CooperativeScheduler scheduler;
  scheduler.Init(TickConfig{kBaseTickUs}, clock, task_set, health);

  // 逐 tick 走到 tick12：phase1 → {1,6,11}（3 次）；phase2 → {2,7,12}（3 次）；
  // phase4 → {4,9}（2 次）。首跑点由 Register 的 (phase-period) 初值保证。
  // 每 tick 后断言累计执行次数 == 到该 tick 为止的激活点个数。
  for (uint32_t tick = 1; tick <= 12; ++tick) {
    clock.AdvanceUs(kBaseTickUs);
    scheduler.TickLoopOnce();
    const uint32_t expect1 = (tick >= 1) + (tick >= 6) + (tick >= 11);
    const uint32_t expect2 = (tick >= 2) + (tick >= 7) + (tick >= 12);
    const uint32_t expect4 = (tick >= 4) + (tick >= 9);
    EXPECT_EQ(ctx1.run_count, expect1) << "phase1 @ tick " << tick;
    EXPECT_EQ(ctx2.run_count, expect2) << "phase2 @ tick " << tick;
    EXPECT_EQ(ctx4.run_count, expect4) << "phase4 @ tick " << tick;
  }
}

TEST(SchedulerTest, TickWrapAroundKeepsPeriodicity) {
  FakeClock clock;
  TaskSet task_set;
  TaskContext ctx;
  TaskConfig cfg = MakeConfig(5, 0);
  ASSERT_TRUE(task_set.Register(cfg, CountingRun, &ctx, 100));
  HealthMonitor health;
  health.Init(HealthConfig{}, task_set);
  CooperativeScheduler scheduler;
  scheduler.Init(TickConfig{kBaseTickUs}, clock, task_set, health);

  // 白盒：任务上次执行在 tick = 2^32-2（回绕边界前）。
  TaskEntry& entry = task_set.at(0);
  entry.stats.last_run_tick = 0xFFFFFFFEu;  // = 2^32-2

  // tick 计数器到 1：diff=(1 - 0xFFFFFFFE) mod 2^32 = 3 < 5 → 不到期。
  clock.AdvanceUs(1u * kBaseTickUs);
  scheduler.TickLoopOnce();
  EXPECT_EQ(scheduler.tick_count(), 1u);
  EXPECT_EQ(ctx.run_count, 0u);

  // tick 到 2：diff=(2 - 0xFFFFFFFE) mod 2^32 = 4 < 5 → 仍不到期。
  clock.AdvanceUs(1u * kBaseTickUs);
  scheduler.TickLoopOnce();
  EXPECT_EQ(scheduler.tick_count(), 2u);
  EXPECT_EQ(ctx.run_count, 0u);

  // tick 到 3：diff=(3 - 0xFFFFFFFE) mod 2^32 = 5 >= 5 → 恰好到期一次（跨回绕后首个激活点）。
  clock.AdvanceUs(1u * kBaseTickUs);
  scheduler.TickLoopOnce();
  EXPECT_EQ(scheduler.tick_count(), 3u);
  EXPECT_EQ(ctx.run_count, 1u);
}

TEST(SchedulerTest, NoCatchUpOnMultiPeriodLag) {
  FakeClock clock;
  TaskSet task_set;
  TaskContext ctx;
  TaskConfig cfg = MakeConfig(5);
  ASSERT_TRUE(task_set.Register(cfg, CountingRun, &ctx, 100));
  HealthMonitor health;
  health.Init(HealthConfig{}, task_set);
  CooperativeScheduler scheduler;
  scheduler.Init(TickConfig{kBaseTickUs}, clock, task_set, health);

  scheduler.TickLoopOnce();  // tick0 首激活
  // 主循环滞留：一次性跳过 30 tick（6 个周期）→ 只应执行一次（最新到期点），不补跑。
  clock.AdvanceUs(30u * kBaseTickUs);
  scheduler.TickLoopOnce();
  EXPECT_EQ(scheduler.tick_count(), 30u);
  EXPECT_EQ(ctx.run_count, 2u);  // tick0 + tick30 各一次，不是 7 次

  // 再滞留 60 tick → tick90：距上次(last=30)差 60 >= 5 → 再执行一次。
  clock.AdvanceUs(60u * kBaseTickUs);
  scheduler.TickLoopOnce();
  EXPECT_EQ(scheduler.tick_count(), 90u);
  EXPECT_EQ(ctx.run_count, 3u);
}

TEST(SchedulerTest, OverrunDeferredNotImmediatelyRerun) {
  FakeClock clock;
  TaskSet task_set;
  SlowContext slow_ctx;
  slow_ctx.clock = &clock;
  slow_ctx.consume_us = 2400;  // 单次执行耗时 2.4 ms > 预算 1 ms
  TaskConfig cfg = MakeConfig(5, 0, /*wcet_budget_us=*/1000);
  ASSERT_TRUE(task_set.Register(cfg, SlowRun, &slow_ctx, 100));
  HealthMonitor health;
  health.Init(HealthConfig{}, task_set);
  CooperativeScheduler scheduler;
  scheduler.Init(TickConfig{kBaseTickUs}, clock, task_set, health);

  scheduler.TickLoopOnce();  // tick0 激活；任务把时钟推前 2400us
  EXPECT_EQ(slow_ctx.run_count, 1u);
  TaskEntry& entry = task_set.at(0);
  EXPECT_EQ(entry.stats.overrun_count, 1u);
  EXPECT_GE(entry.stats.max_elapsed_us, 2400u);
  // 顺延 floor(2400/1000)=2 tick：last_run_tick = 0+2 = 2（不超前 tick 计数）。
  EXPECT_EQ(entry.stats.last_run_tick, 2u);

  // 下一循环吸收 2400us → tick=2；diff=(2-2)=0 < 5 → 不立即重跑。
  scheduler.TickLoopOnce();
  EXPECT_EQ(scheduler.tick_count(), 2u);
  EXPECT_EQ(slow_ctx.run_count, 1u);

  // tick3..6：diff 未达 5 → 不跑。
  AdvanceTicks(clock, scheduler, 4);
  EXPECT_EQ(scheduler.tick_count(), 6u);
  EXPECT_EQ(slow_ctx.run_count, 1u);

  // tick7：diff=(7-2)=5 → 到期再跑一次。
  AdvanceTicks(clock, scheduler, 1);
  EXPECT_EQ(scheduler.tick_count(), 7u);
  EXPECT_EQ(slow_ctx.run_count, 2u);
}

TEST(SchedulerTest, IdleRunIsStable) {
  FakeClock clock;
  TaskSet task_set;
  HealthMonitor health;
  health.Init(HealthConfig{}, task_set);
  CooperativeScheduler scheduler;
  scheduler.Init(TickConfig{kBaseTickUs}, clock, task_set, health);

  for (uint32_t i = 0; i < 1000; ++i) {
    clock.AdvanceUs(kBaseTickUs);  // 先推进再跑，确保每次累计恰好 1 tick
    scheduler.TickLoopOnce();
  }
  EXPECT_EQ(scheduler.stats().loop_count, 1000u);
  EXPECT_EQ(scheduler.tick_count(), 1000u);
}

}  // namespace
}  // namespace robotcar01::mcu_os_lite
