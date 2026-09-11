// MCU OS Lite 健康监控测试（Iteration 002）
//
// 覆盖（映射 detailed design 验收 3/5 与测试策略表）：
//   - 非关键单次 overrun 仅统计（feed 不变）；
//   - 关键单次 overrun → snapshot() 立即 feed_allowed=false + stop_active（不等 Check）；
//   - 窗口内第 2 次 overrun（任意任务）→ 立即 fault_latched（D-002-4）；
//   - 关键任务卡死（窗口内 run_count 无增量）→ Check 后锁存；
//   - 首窗宽限：注册后第一个监督窗口不判 stale（Check 只建基线）；
//   - ClearFault 复位 stop/fault；恢复需下一 Check 心跳全新鲜；
//   - wcet_budget_us=0 不判定 overrun；
//   - 跨度式 Check 触发（scheduler 集成：窗口边界清 overrun 计数，含回绕）。

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

constexpr uint32_t kBaseTickUs = 1000;

struct TaskContext {
  uint32_t run_count = 0;
};

void CountingRun(void* opaque) {
  auto* ctx = static_cast<TaskContext*>(opaque);
  ++ctx->run_count;
}

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

TaskConfig MakeConfig(uint32_t period_ticks, uint32_t phase_ticks, uint32_t wcet_budget_us,
                      TaskCriticality criticality) {
  TaskConfig cfg;
  cfg.name = "test_task";
  cfg.period_ticks = period_ticks;
  cfg.phase_ticks = phase_ticks;
  cfg.wcet_budget_us = wcet_budget_us;
  cfg.criticality = criticality;
  return cfg;
}

HealthConfig MakeHealth(uint32_t window_ticks = 100, uint32_t max_overruns = 2) {
  HealthConfig cfg;
  cfg.supervision_window_ticks = window_ticks;
  cfg.max_overruns_per_window = max_overruns;
  return cfg;
}

// --- 白盒单元测试：直接驱动 HealthMonitor，run_count 增量模拟"调度器执行" ---

TEST(HealthMonitorTest, FirstCheckIsGraceNoFalseLatch) {
  TaskSet task_set;
  TaskContext ctx;
  TaskConfig cfg = MakeConfig(100, 0, 0, TaskCriticality::kCritical);  // period==window
  ASSERT_TRUE(task_set.Register(cfg, CountingRun, &ctx, 100));
  HealthMonitor health;
  health.Init(MakeHealth(100), task_set);

  // 关键任务从未执行，但首窗 Check 是宽限：只建基线，不锁存。
  health.Check(100);
  HealthSnapshot snap = health.snapshot();
  EXPECT_FALSE(snap.fault_latched);
  EXPECT_TRUE(snap.critical_heartbeat_ok);  // 宽限期按"未判定"保持 ok（无 stale 计数）
  EXPECT_EQ(snap.stale_critical_count, 0u);
}

TEST(HealthMonitorTest, StaleCriticalHeartbeatLatchesOnCheck) {
  TaskSet task_set;
  TaskContext ctx;
  TaskConfig cfg = MakeConfig(100, 0, 0, TaskCriticality::kCritical);
  ASSERT_TRUE(task_set.Register(cfg, CountingRun, &ctx, 100));
  HealthMonitor health;
  health.Init(MakeHealth(100), task_set);

  // 首窗宽限（基线）。
  health.Check(100);
  // 第二窗：run_count 无增量（卡死模型：任务未被执行）。
  health.Check(200);
  HealthSnapshot snap = health.snapshot();
  EXPECT_TRUE(snap.fault_latched);
  EXPECT_FALSE(snap.watchdog_feed_allowed);
  EXPECT_FALSE(snap.critical_heartbeat_ok);
  EXPECT_EQ(snap.stale_critical_count, 1u);
}

TEST(HealthMonitorTest, HealthyCriticalHeartbeatStaysOkAcrossWindows) {
  TaskSet task_set;
  TaskContext ctx;
  TaskConfig cfg = MakeConfig(100, 0, 0, TaskCriticality::kCritical);
  ASSERT_TRUE(task_set.Register(cfg, CountingRun, &ctx, 100));
  HealthMonitor health;
  health.Init(MakeHealth(100), task_set);

  health.Check(100);  // 宽限基线（此刻 stats.run_count=0）
  ++task_set.at(0).stats.run_count;  // 模拟调度器在第一窗执行了一次任务
  health.Check(200);
  // 又执行一次。
  ++task_set.at(0).stats.run_count;
  health.Check(300);
  HealthSnapshot snap = health.snapshot();
  EXPECT_FALSE(snap.fault_latched);
  EXPECT_TRUE(snap.critical_heartbeat_ok);
  EXPECT_TRUE(snap.watchdog_feed_allowed);
  EXPECT_EQ(snap.stale_critical_count, 0u);
}

TEST(HealthMonitorTest, NonCriticalStaleDoesNotLatch) {
  TaskSet task_set;
  TaskContext ctx;
  TaskConfig cfg = MakeConfig(100, 0, 0, TaskCriticality::kNonCritical);
  ASSERT_TRUE(task_set.Register(cfg, CountingRun, &ctx, 100));
  HealthMonitor health;
  health.Init(MakeHealth(100), task_set);

  health.Check(100);  // 宽限基线
  health.Check(200);  // 非关键任务无心跳不锁存
  HealthSnapshot snap = health.snapshot();
  EXPECT_FALSE(snap.fault_latched);
  EXPECT_TRUE(snap.critical_heartbeat_ok);
  EXPECT_TRUE(snap.watchdog_feed_allowed);
}

TEST(HealthMonitorTest, ClearFaultClearsLatchAndNeedsFreshWindowToRecover) {
  TaskSet task_set;
  TaskContext ctx;
  TaskConfig cfg = MakeConfig(100, 0, 0, TaskCriticality::kCritical);
  ASSERT_TRUE(task_set.Register(cfg, CountingRun, &ctx, 100));
  HealthMonitor health;
  health.Init(MakeHealth(100), task_set);

  health.Check(100);  // 宽限
  health.Check(200);  // 无心跳 → 锁存
  EXPECT_TRUE(health.snapshot().fault_latched);
  EXPECT_FALSE(health.snapshot().watchdog_feed_allowed);

  health.ClearFault();
  HealthSnapshot after_clear = health.snapshot();
  EXPECT_FALSE(after_clear.fault_latched);
  EXPECT_FALSE(after_clear.stop_active);
  // critical_heartbeat_ok 仍为 false（上一 Check 判定），喂狗许可保持 false 直到新鲜窗口。
  EXPECT_FALSE(after_clear.watchdog_feed_allowed);

  // 恢复：心跳恢复 + 下一 Check 心跳新鲜。
  ++task_set.at(0).stats.run_count;
  health.Check(300);
  HealthSnapshot recovered = health.snapshot();
  EXPECT_TRUE(recovered.critical_heartbeat_ok);
  EXPECT_TRUE(recovered.watchdog_feed_allowed);
  EXPECT_FALSE(recovered.fault_latched);
}

TEST(HealthMonitorTest, RepeatedOverrunLatchesFaultImmediately) {
  TaskSet task_set;
  SlowContext slow_ctx;
  slow_ctx.clock = nullptr;  // 白盒：不走 scheduler，只驱动 health 事件
  TaskConfig noncritical = MakeConfig(5, 0, 1000, TaskCriticality::kNonCritical);
  ASSERT_TRUE(task_set.Register(noncritical, SlowRun, &slow_ctx, 100));
  HealthMonitor health;
  health.Init(MakeHealth(100, /*max_overruns=*/2), task_set);

  health.OnTaskOverrun(0);  // 第 1 次（非关键：只计数）
  EXPECT_FALSE(health.snapshot().fault_latched);
  health.OnTaskOverrun(0);  // 第 2 次 → 窗口累计 >=2 → 立即锁存
  HealthSnapshot snap = health.snapshot();
  EXPECT_TRUE(snap.fault_latched);
  EXPECT_FALSE(snap.watchdog_feed_allowed);
  EXPECT_EQ(snap.overruns_in_window, 2u);
}

TEST(HealthMonitorTest, CriticalSingleOverrunStopsImmediately) {
  TaskSet task_set;
  SlowContext slow_ctx;
  slow_ctx.clock = nullptr;
  TaskConfig critical = MakeConfig(5, 0, 1000, TaskCriticality::kCritical);
  ASSERT_TRUE(task_set.Register(critical, SlowRun, &slow_ctx, 100));
  HealthMonitor health;
  health.Init(MakeHealth(100), task_set);

  health.OnTaskOverrun(0);  // 关键单次 → 立即 stop
  HealthSnapshot snap = health.snapshot();
  EXPECT_TRUE(snap.stop_active);
  EXPECT_FALSE(snap.watchdog_feed_allowed);
  EXPECT_FALSE(snap.fault_latched);  // 单次 overrun 只 stop，不锁存
}

TEST(HealthMonitorTest, NonCriticalSingleOverrunDoesNotStop) {
  TaskSet task_set;
  SlowContext slow_ctx;
  slow_ctx.clock = nullptr;
  TaskConfig noncritical = MakeConfig(5, 0, 1000, TaskCriticality::kNonCritical);
  ASSERT_TRUE(task_set.Register(noncritical, SlowRun, &slow_ctx, 100));
  HealthMonitor health;
  health.Init(MakeHealth(100), task_set);

  health.OnTaskOverrun(0);
  HealthSnapshot snap = health.snapshot();
  EXPECT_FALSE(snap.stop_active);
  EXPECT_FALSE(snap.fault_latched);
  EXPECT_TRUE(snap.watchdog_feed_allowed);
}

TEST(HealthMonitorTest, ClearFaultClearsStopFromCriticalOverrun) {
  TaskSet task_set;
  SlowContext slow_ctx;
  slow_ctx.clock = nullptr;
  TaskConfig critical = MakeConfig(5, 0, 1000, TaskCriticality::kCritical);
  ASSERT_TRUE(task_set.Register(critical, SlowRun, &slow_ctx, 100));
  HealthMonitor health;
  health.Init(MakeHealth(100), task_set);

  health.OnTaskOverrun(0);
  EXPECT_TRUE(health.snapshot().stop_active);

  health.ClearFault();
  HealthSnapshot snap = health.snapshot();
  EXPECT_FALSE(snap.stop_active);
  EXPECT_TRUE(snap.critical_heartbeat_ok);  // 无 Check 判定历史 → ok 保持 true → 许可恢复
  EXPECT_TRUE(snap.watchdog_feed_allowed);
}

// --- scheduler 集成测试 ---

TEST(HealthMonitorTest, WcetBudgetZeroDoesNotTriggerOverrun) {
  FakeClock clock;
  TaskSet task_set;
  SlowContext slow_ctx;
  slow_ctx.clock = &clock;
  slow_ctx.consume_us = 2400;
  TaskConfig cfg = MakeConfig(5, 0, /*wcet_budget_us=*/0, TaskCriticality::kNonCritical);
  ASSERT_TRUE(task_set.Register(cfg, SlowRun, &slow_ctx, 100));
  HealthMonitor health;
  health.Init(MakeHealth(100), task_set);
  CooperativeScheduler scheduler;
  scheduler.Init(TickConfig{kBaseTickUs}, clock, task_set, health);

  scheduler.TickLoopOnce();
  TaskEntry& entry = task_set.at(0);
  EXPECT_EQ(entry.stats.overrun_count, 0u);       // 未标定不判定
  EXPECT_GE(entry.stats.max_elapsed_us, 2400u);   // 但记录 WCET 证据
  EXPECT_TRUE(health.snapshot().watchdog_feed_allowed);
}

TEST(HealthMonitorTest, SchedulerReportsOverrunAndWindowCheckResetsCounter) {
  FakeClock clock;
  TaskSet task_set;
  SlowContext slow_ctx;
  slow_ctx.clock = &clock;
  slow_ctx.consume_us = 2400;  // 单次 2.4ms > 预算 1ms
  TaskConfig cfg = MakeConfig(5, 0, /*wcet_budget_us=*/1000, TaskCriticality::kNonCritical);
  ASSERT_TRUE(task_set.Register(cfg, SlowRun, &slow_ctx, 100));
  // max_overruns 放大到 1000：本测试只验证"窗口 Check 清零计数"，不做锁存。
  HealthMonitor health;
  health.Init(MakeHealth(100, /*max_overruns=*/1000), task_set);
  CooperativeScheduler scheduler;
  scheduler.Init(TickConfig{kBaseTickUs}, clock, task_set, health);

  scheduler.TickLoopOnce();  // tick0 overrun #1
  EXPECT_EQ(health.snapshot().overruns_in_window, 1u);

  // 窗口=100 tick：推进跨过窗口边界 → scheduler 应触发 Check 并把窗口计数清零。
  // 任务在 tick0 已把时钟推到 2400us；目标 tick 100 → 一次 TickLoopOnce 快照后触发 Check。
  const uint64_t now_us = clock.now_us();
  const uint64_t target_us = 100u * kBaseTickUs;
  if (target_us > now_us) {
    clock.AdvanceUs(target_us - now_us);
  }
  scheduler.TickLoopOnce();
  EXPECT_GE(scheduler.tick_count(), 100u);
  EXPECT_EQ(health.snapshot().overruns_in_window, 0u);  // Check 已把窗口计数清零
}

TEST(HealthMonitorTest, WindowCheckFiresAfterTickWrap) {
  // 用很小的窗口（5 tick）验证跨度式判定在 tick 跨 2^32 回绕后仍触发。
  FakeClock clock;
  TaskSet task_set;
  TaskContext ctx;
  TaskConfig cfg = MakeConfig(5, 0, 0, TaskCriticality::kCritical);
  ASSERT_TRUE(task_set.Register(cfg, CountingRun, &ctx, 5));  // 关键 period<=window(5)
  HealthMonitor health;
  health.Init(MakeHealth(5), task_set);
  CooperativeScheduler scheduler;
  scheduler.Init(TickConfig{kBaseTickUs}, clock, task_set, health);

  // 白盒：把任务上次执行置于回绕边界附近。
  TaskEntry& entry = task_set.at(0);
  entry.stats.last_run_tick = 0xFFFFFFFEu;  // 2^32-2

  // 一次性推进到 tick=2^32-2（注意：乘法先转 uint64 防 32 位溢出）。
  clock.AdvanceUs(static_cast<uint64_t>(0xFFFFFFFEu) * kBaseTickUs);
  scheduler.TickLoopOnce();
  EXPECT_EQ(scheduler.tick_count(), 0xFFFFFFFEu);
  // diff=(0xFFFFFFFE - 0xFFFFFFFE)=0 <5 → 不跑。
  EXPECT_EQ(ctx.run_count, 0u);

  // 推进过回绕到 tick=3：diff=(3 - 0xFFFFFFFE) mod 2^32 = 5 → 到期执行一次。
  clock.AdvanceUs(static_cast<uint64_t>(5u) * kBaseTickUs);
  scheduler.TickLoopOnce();
  EXPECT_EQ(scheduler.tick_count(), 3u);
  EXPECT_EQ(ctx.run_count, 1u);
}

}  // namespace
}  // namespace robotcar01::mcu_os_lite
