#include "app/control/control_tasks.h"

#include "app/chassis/safety_gate.h"

namespace robotcar01::control {

namespace {

constexpr uint32_t kControlPeriodTicks = WheelControllerConfig::kNominalPeriodUs / 1000u;
static_assert(kControlPeriodTicks == 5u, "control period must be 5 ms at 1 ms base tick (arch §6)");

const mcu_os_lite::TaskConfig kWheelControlConfig{
    "WheelControlTask", kControlPeriodTicks, 2u, 400u, mcu_os_lite::TaskCriticality::kCritical};
const mcu_os_lite::TaskConfig kOutputArbitrationConfig{
    "OutputArbitrationTask", 1u, 0u, 100u, mcu_os_lite::TaskCriticality::kCritical};

bool AllowMotion(const ControlContext& ctx) {
  return ctx.safety != nullptr && ctx.safety->allow_motion;
}

}  // namespace

const mcu_os_lite::TaskConfig& WheelControlTaskConfig() { return kWheelControlConfig; }
const mcu_os_lite::TaskConfig& OutputArbitrationTaskConfig() { return kOutputArbitrationConfig; }

void WheelControlTaskRun(void* context) {
  auto* ctx = static_cast<ControlContext*>(context);
  if (ctx == nullptr || ctx->clock == nullptr || ctx->left == nullptr || ctx->right == nullptr ||
      ctx->arbitration == nullptr) {
    return;
  }
  const uint64_t now_us = ctx->clock->Now().us;
  const bool allow = AllowMotion(*ctx);

  chassis::WheelTarget target{};
  if (ctx->target != nullptr) {
    target = *ctx->target;
  }
  const float left_target = target.valid ? target.left_mps : 0.0f;
  const float right_target = target.valid ? target.right_mps : 0.0f;

  encoder::WheelState wheel{};
  if (ctx->wheel_state != nullptr) {
    wheel = *ctx->wheel_state;  // nullptr ⇒ 默认不可信 ⇒ 控制器零输出
  }

  MotorOutputPair pair{};
  pair.left = ctx->left->Update(left_target, wheel.left, wheel.timestamp.us, allow, now_us);
  pair.right = ctx->right->Update(right_target, wheel.right, wheel.timestamp.us, allow, now_us);
  pair.produced_at_us = now_us;
  ctx->last_pair = pair;
  ctx->arbitration->Submit(pair);

  // 反馈速度域投影：复用 001 冻结组合规则（D-006-6）
  chassis::SafetyStatus safety{};
  safety.allow_motion = allow;
  ctx->speed_request = chassis::RunSafetyGate(target, safety);
}

void OutputArbitrationTaskRun(void* context) {
  auto* ctx = static_cast<ControlContext*>(context);
  if (ctx == nullptr || ctx->clock == nullptr || ctx->arbitration == nullptr) {
    return;
  }
  ctx->arbitration->Tick(AllowMotion(*ctx), ctx->clock->Now().us);
}

bool RegisterControlTasks(mcu_os_lite::TaskSet& task_set, ControlContext& context,
                          uint32_t supervision_window_ticks) {
  if (!task_set.Register(kWheelControlConfig, &WheelControlTaskRun, &context,
                         supervision_window_ticks)) {
    return false;
  }
  return task_set.Register(kOutputArbitrationConfig, &OutputArbitrationTaskRun, &context,
                           supervision_window_ticks);
}

}  // namespace robotcar01::control
