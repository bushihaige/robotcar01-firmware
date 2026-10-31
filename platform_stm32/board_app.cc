#include "platform_stm32/board_app.h"

#include "platform_stm32/platform_config.h"
#include "platform_stm32/uart_console.h"

namespace robotcar01::platform {
namespace {

constexpr uint32_t kEncoderTaskPeriodTicks = 5U;
constexpr uint32_t kEncoderTaskWcetUs = 200U;
constexpr uint32_t kSupervisionWindowTicks = 100U;
constexpr uint32_t kMaxCommandsPerStep = 4U;

int32_t ToMilli(float value) { return static_cast<int32_t>(value * 1000.0f); }

}  // namespace

namespace {
// ISR 访问的当前 sink（Init 期设置一次，运行期不再变）；单核 Cortex-M 上指针赋值是原子的。
robotcar01::platform::Ibt2PwmSink* g_watchdog_sink = nullptr;
}  // namespace

void MotorWatchdogTickFromIsr() {
  Ibt2PwmSink* sink = g_watchdog_sink;
  if (sink != nullptr) {
    (void)sink->ServiceWatchdogFromIsr(SysTickClockSource::MillisSinceBoot());
  }
}

bool BoardApp::Init() {
  clock_.Reset();

  // ---- 1) 配置（fail-closed：任一 IsValid 失败都不进入主循环） ----
  encoder_config_.location = encoder::EncoderLocation::kOutputShaft;  // D-006.5-2 输出轴模型
  encoder_config_.counts_per_encoder_rev = kEncoderCountsPerWheelRev;
  encoder_config_.gear_ratio = kGearRatio;
  encoder_config_.wheel_radius_m = kWheelRadiusM;
  encoder_config_.left.counter_bits = kEncoderCounterBits;
  encoder_config_.right.counter_bits = kEncoderCounterBits;
  encoder_config_.left.polarity =
      kEncoderLeftInverted ? encoder::EncoderPolarity::kInverted : encoder::EncoderPolarity::kNormal;
  encoder_config_.right.polarity =
      kEncoderRightInverted ? encoder::EncoderPolarity::kInverted : encoder::EncoderPolarity::kNormal;
  encoder_config_.low_speed_zero_threshold_mps = kMeasurementFloorMps;
  encoder_config_.max_estimation_delay_us = 20000U;
  encoder_config_.stall_windows_threshold = 8U;
  encoder_config_.low_speed_accumulate_counts = 4U;
  encoder_config_.max_wheel_speed_mps = 0.5f;
  if (!encoder_config_.IsValid()) {
    SetError("encoder config invalid");
    return false;
  }

  controller_config_ = control::WheelControllerConfig{};  // 006 冻结默认（占位非标定）
  controller_config_.measurement_floor_mps = encoder_config_.low_speed_zero_threshold_mps;
  if (!controller_config_.IsValid()) {
    SetError("controller config invalid");
    return false;
  }

  // ---- 2) 算法对象 ----
  if (!estimator_.Init(encoder_config_)) {
    SetError("estimator init failed");
    return false;
  }
  if (!left_.Init(controller_config_)) {
    SetError("left controller init failed");
    return false;
  }
  if (!right_.Init(controller_config_)) {
    SetError("right controller init failed");
    return false;
  }

  // ---- 3) 平台外设（EN 保持失能直到首次 Apply） ----
  EncoderInputConfig encoder_input_config{};
  encoder_input_config.counter_bits = kEncoderCounterBits;
  encoder_input_config.invert_left = kEncoderLeftInverted;
  encoder_input_config.invert_right = kEncoderRightInverted;
  if (!encoder_input_.Init(encoder_input_config)) {
    SetError("encoder hardware init failed");
    return false;
  }
  if (!sink_.Init()) {
    SetError("pwm sink init failed");
    return false;
  }
  g_watchdog_sink = &sink_;  // ISR 监护从此刻起生效（EN 仍为失能，直到首次 Apply）

  control::ArbitrationConfig arbitration_config{};
  if (!arbitration_config.IsValid()) {
    SetError("arbitration config invalid");
    return false;
  }
  if (!arbitration_.Init(sink_, arbitration_config)) {
    SetError("arbitration init failed");
    return false;
  }

  // ---- 4) 数据端口 ----
  safety_.allow_motion = true;  // 007 之前恒允许；急停靠硬件（详设 §3）
  ctx_.clock = &clock_;
  ctx_.target = &target_;
  ctx_.wheel_state = &wheel_state_;
  ctx_.safety = &safety_;
  ctx_.left = &left_;
  ctx_.right = &right_;
  ctx_.arbitration = &arbitration_;
  mode_ = MotorMode::kClosed;
  wheels_ = WheelSelection::kBoth;
  target_mps_ = 0.0f;
  open_duty_permille_ = 0U;
  RefreshTarget();

  // ---- 5) 任务注册（顺序 = 同 tick 内执行序：编码器 → 控制 → 仲裁由周期/相位错开） ----
  encoder_task_config_.name = "EncoderTask";
  encoder_task_config_.period_ticks = kEncoderTaskPeriodTicks;
  encoder_task_config_.phase_ticks = 0U;
  encoder_task_config_.wcet_budget_us = kEncoderTaskWcetUs;
  encoder_task_config_.criticality = mcu_os_lite::TaskCriticality::kCritical;

  mcu_os_lite::HealthConfig health_config{};
  health_config.supervision_window_ticks = kSupervisionWindowTicks;
  health_config.max_overruns_per_window = 2U;
  health_.Init(health_config, task_set_);

  const uint32_t window = health_.supervision_window_ticks();
  if (!task_set_.Register(encoder_task_config_, &EncoderTaskEntry, this, window)) {
    SetError("register encoder task failed");
    return false;
  }
  if (!task_set_.Register(control::WheelControlTaskConfig(), &ControlTaskEntry, this, window)) {
    SetError("register control task failed");
    return false;
  }
  if (!task_set_.Register(control::OutputArbitrationTaskConfig(), &ArbitrationTaskEntry, this,
                          window)) {
    SetError("register arbitration task failed");
    return false;
  }

  mcu_os_lite::TickConfig tick_config{};
  tick_config.base_tick_us = 1000U;
  scheduler_.Init(tick_config, clock_, task_set_, health_);

  // ---- 6) 命令通道（RX 失败不致命：仍可观测，只是收不到命令） ----
  if (!console_.Init()) {
    SetError("console rx not started (tx-only)");
  }

  last_status_ms_ = SysTickClockSource::MillisSinceBoot();
  return true;
}

void BoardApp::EncoderTaskEntry(void* arg) {
  auto* self = static_cast<BoardApp*>(arg);
  if (self != nullptr) {
    self->OnEncoderTick();
  }
}

void BoardApp::ControlTaskEntry(void* arg) {
  auto* self = static_cast<BoardApp*>(arg);
  if (self != nullptr) {
    self->OnControlTick();
  }
}

void BoardApp::ArbitrationTaskEntry(void* arg) {
  auto* self = static_cast<BoardApp*>(arg);
  if (self != nullptr) {
    control::OutputArbitrationTaskRun(&self->ctx_);
  }
}

void BoardApp::OnEncoderTick() {
  const uint64_t now_us = clock_.Now().us;
  const encoder::EncoderSnapshot snapshot = encoder_input_.Read(now_us);
  // 006.5 简化：无命令租约（无上位机），motion_desired = 允许运动 ∧ 闭环 ∧ 目标非零。
  // 004 的"唯一构造式"在 003 协议路径上仍由 protocol::MakeMotionDesired 承担（D-006.5-6）。
  const bool motion_desired =
      safety_.allow_motion && (mode_ == MotorMode::kClosed) && (target_mps_ != 0.0f);
  estimator_.OnSample(snapshot, motion_desired);
  wheel_state_ = estimator_.state();
}

control::MotorOutputRequest BoardApp::MakeOpenLoopRequest(bool active) const {
  if (!active || open_duty_permille_ == 0U || !safety_.allow_motion) {
    return control::CoastRequest(control::kOutputZeroTarget);
  }
  control::MotorOutputRequest request{};
  request.mode = control::OutputMode::kDrive;
  request.direction = open_direction_;
  request.duty_permille = open_duty_permille_;
  request.reason = control::kOutputNone;
  return request;
}

void BoardApp::OnControlTick() {
  if (mode_ == MotorMode::kOpen) {
    const uint64_t now_us = clock_.Now().us;
    control::MotorOutputPair pair{};
    pair.left = MakeOpenLoopRequest(wheels_ != WheelSelection::kRight);
    pair.right = MakeOpenLoopRequest(wheels_ != WheelSelection::kLeft);
    pair.produced_at_us = now_us;
    ctx_.last_pair = pair;
    arbitration_.Submit(pair);  // D-006-10：开环同样只经仲裁写 PWM
    return;
  }
  control::WheelControlTaskRun(&ctx_);  // 复用 006 任务体（含 SafetyGate 投影）
}

void BoardApp::RefreshTarget() {
  const bool allow_left = (wheels_ != WheelSelection::kRight);
  const bool allow_right = (wheels_ != WheelSelection::kLeft);
  const float left = allow_left && (mode_ == MotorMode::kClosed) ? target_mps_ : 0.0f;
  const float right = allow_right && (mode_ == MotorMode::kClosed) ? target_mps_ : 0.0f;
  target_.raw_left_mps = left;
  target_.raw_right_mps = right;
  target_.left_mps = left;
  target_.right_mps = right;
  target_.scale = 1.0f;
  target_.limit_reason = chassis::LimitReason::kNone;
  target_.valid = (mode_ == MotorMode::kClosed);
}

void BoardApp::Apply(ConsoleCommand command) {
  switch (command) {
    case ConsoleCommand::kStop:
      // 只把目标置零：停止确认（方向锁存 + dwell）由 006 控制器自己走完。
      // 这里**不能** Reset()，否则会清掉 applied_dir_ 与 STOPPING 状态，绕过 INV-006-8 的停止流程。
      target_mps_ = 0.0f;
      open_duty_permille_ = 0U;
      break;
    case ConsoleCommand::kTargetUp:
      // 调目标同样不 Reset：Reset 会清积分与方向锁存，导致每次调档都从零重来并掩盖停止语义。
      mode_ = MotorMode::kClosed;
      target_mps_ += kTargetStepMps;
      if (target_mps_ > kTargetMaxMps) {
        target_mps_ = kTargetMaxMps;
      }
      break;
    case ConsoleCommand::kTargetDown:
      mode_ = MotorMode::kClosed;
      target_mps_ -= kTargetStepMps;
      if (target_mps_ < -kTargetMaxMps) {
        target_mps_ = -kTargetMaxMps;
      }
      break;
    case ConsoleCommand::kOpenDutyUp:
      mode_ = MotorMode::kOpen;
      open_duty_permille_ =
          static_cast<uint16_t>((open_duty_permille_ + kOpenLoopStepPermille) > kOpenLoopMaxPermille
                                    ? kOpenLoopMaxPermille
                                    : (open_duty_permille_ + kOpenLoopStepPermille));
      break;
    case ConsoleCommand::kOpenDutyDown:
      mode_ = MotorMode::kOpen;
      open_duty_permille_ = static_cast<uint16_t>(
          (open_duty_permille_ < kOpenLoopStepPermille) ? 0U : (open_duty_permille_ - kOpenLoopStepPermille));
      break;
    case ConsoleCommand::kModeToggle:
      mode_ = (mode_ == MotorMode::kClosed) ? MotorMode::kOpen : MotorMode::kClosed;
      if (mode_ == MotorMode::kOpen) {
        target_mps_ = 0.0f;
      } else {
        open_duty_permille_ = 0U;
      }
      left_.Reset();
      right_.Reset();
      break;
    case ConsoleCommand::kGpioScan:
      PrintGpioScan();
      return;
    case ConsoleCommand::kEncoderRaw:
      PrintEncoderRaw();
      return;
    case ConsoleCommand::kInfoPrint:
      PrintInfo();
      return;
    case ConsoleCommand::kDebugStall: {
      // 仅允许在零输出（目标 0 / 开环占空比 0）时使用：验证 INV-006.5-6
      // —— 主循环被卡住 120 ms 期间，1 ms ISR 应在 50 ms 时把 EN 拉低并让 wd 计数 +1。
      if (target_mps_ != 0.0f || open_duty_permille_ != 0U) {
        UartWriteLine("x: refused (set target 0 first)");
        return;
      }
      UartWriteLine("x: stalling main loop 120 ms ...");
      const uint32_t start_ms = SysTickClockSource::MillisSinceBoot();
      while ((SysTickClockSource::MillisSinceBoot() - start_ms) < 120U) {
        // 忙等：中断照常运行（SysTick/看门狗/串口 RX）
      }
      UartWriteLine("x: resumed");
      return;
    }
    case ConsoleCommand::kPrintOnce:
      status_line_index_ = 0U;
      status_active_ = true;  // 由 Step 分帧输出（避免一次性 8–10 ms 阻塞）
      return;
    case ConsoleCommand::kOpenDirectionToggle:
      open_direction_ = (open_direction_ == control::MotorDirection::kForward)
                            ? control::MotorDirection::kReverse
                            : control::MotorDirection::kForward;
      open_duty_permille_ = 0U;  // 先归零，避免带载瞬间反向（与 INV-006.5-3 同精神）
      mode_ = MotorMode::kOpen;
      break;
    case ConsoleCommand::kZeroBaseline:
      encoder_input_.Restart();
      estimator_.Reset();
      left_.Reset();
      right_.Reset();
      break;
    case ConsoleCommand::kWheelBoth:
      wheels_ = WheelSelection::kBoth;
      break;
    case ConsoleCommand::kWheelLeft:
      wheels_ = WheelSelection::kLeft;
      break;
    case ConsoleCommand::kWheelRight:
      wheels_ = WheelSelection::kRight;
      break;
    case ConsoleCommand::kNone:
      return;
  }
  RefreshTarget();
}

void BoardApp::Step() {
  scheduler_.TickLoopOnce();

  for (uint32_t index = 0U; index < kMaxCommandsPerStep; ++index) {
    const ConsoleCommand command = console_.Poll();
    if (command == ConsoleCommand::kNone) {
      break;
    }
    Apply(command);
  }

  const uint32_t now_ms = SysTickClockSource::MillisSinceBoot();
  if ((now_ms - last_status_ms_) >= kStatusPeriodMs) {
    last_status_ms_ = now_ms;
    status_line_index_ = 0U;
    status_active_ = true;
  }
  // 分帧输出（D-006.5-7 实现修订）：每次 Step 只发一行（约 1-2 ms 阻塞），
  // 相邻行之间必然回到调度器，避免一次打印 8-10 ms 让 1 ms 仲裁停摆并诱发 stale_zero。
  if (status_active_) {
    PrintStatusLine(status_line_index_);
    ++status_line_index_;
    if (status_line_index_ >= kStatusLineCount) {
      status_active_ = false;
    }
  }
}

void BoardApp::PrintHelp() {
  PrintCommandHelp();
  UartWrite("cfg: cpr=");
  UartWriteU32(static_cast<uint32_t>(kEncoderCountsPerWheelRev));
  UartWrite(" radius_mm=");
  UartWriteU32(static_cast<uint32_t>(kWheelRadiusM * 1000.0f));
  UartWrite(" pwm_hz=");
  UartWriteU32(kPwmFrequencyHz);
  UartWrite(" target_max_mmps=");
  UartWriteU32(static_cast<uint32_t>(kTargetMaxMps * 1000.0f));
  UartWriteLine("");
}

void BoardApp::PrintGpioScan() {
  // 一次性打两行：第一行原始 32 位，第二行按位展开成 16 个 0/1，便于肉眼对比"哪一位在动"。
  const uint32_t a = encoder_input_.ReadGpioA();
  const uint32_t b = encoder_input_.ReadGpioB();
  const uint32_t odr_b = encoder_input_.ReadOdrB();
  UartWrite("gpio: A=");
  UartWriteHex32(a);
  UartWrite(" B(IDR)=");
  UartWriteHex32(b);
  UartWrite(" B(ODR)=");
  UartWriteHex32(odr_b);
  UartWriteLine("");
  // EN 是 PB0/PB1（推挽输出）。IDR 与 ODR 不一致 ⇒ 外部把引脚拉到了别的电平。
  UartWrite("gpioEN: (PB0=en_right PB1=en_left) en_l_idr=");
  UartWriteU32(b & 0x1U);
  UartWrite(" en_l_odr=");
  UartWriteU32(odr_b & 0x1U);
  UartWrite(" en_r_idr=");
  UartWriteU32((b >> 1U) & 0x1U);
  UartWrite(" en_r_odr=");
  UartWriteU32((odr_b >> 1U) & 0x1U);
  UartWriteLine("  (IDR!=ODR => pin externally pulled)");
  UartWrite("gpioA15-0: ");
  for (int32_t bit = 15; bit >= 0; --bit) {
    UartWrite(((a >> static_cast<uint32_t>(bit)) & 1U) != 0U ? "1" : "0");
    if (bit % 4 == 0) {
      UartWrite(" ");
    }
  }
  UartWriteLine("  (bit15..bit0)");
  UartWrite("gpioB15-0: ");
  for (int32_t bit = 15; bit >= 0; --bit) {
    UartWrite(((b >> static_cast<uint32_t>(bit)) & 1U) != 0U ? "1" : "0");
    if (bit % 4 == 0) {
      UartWrite(" ");
    }
  }
  UartWriteLine("  (bit15..bit0)");
}

void BoardApp::PrintEncoderRaw() {
  const EncoderRawDiagnostics raw = encoder_input_.ReadRaw();
  UartWrite("enc-raw: tim2=");
  UartWriteU32(raw.tim2_count);
  UartWrite(" tim3=");
  UartWriteU32(raw.tim3_count);
  UartWrite(" pa0=");
  UartWriteU32(raw.pa0 ? 1U : 0U);
  UartWrite(" pa1=");
  UartWriteU32(raw.pa1 ? 1U : 0U);
  UartWrite(" pa6=");
  UartWriteU32(raw.pa6 ? 1U : 0U);
  UartWrite(" pa7=");
  UartWriteU32(raw.pa7 ? 1U : 0U);
  UartWriteLine("");
  UartWrite("enc-raw2: tim2_cr1=");
  UartWriteHex32(raw.tim2_cr1);
  UartWrite(" smcr=");
  UartWriteHex32(raw.tim2_smcr);
  UartWrite(" ccmr1=");
  UartWriteHex32(raw.tim2_ccmr1);
  UartWrite(" ccer=");
  UartWriteHex32(raw.tim2_ccer);
  UartWriteLine("");
  // 判读：smcr SMS[2:0] 应=3（0x3 mask），ccer CC1E|CC2E (bit0|bit4) 应置位，cr1 CEN(bit0) 应置位
  UartWrite("enc-raw2: (tim2=RIGHT wheel) sms=");
  UartWriteU32(raw.tim2_smcr & 0x7U);
  UartWrite(" cen=");
  UartWriteU32(raw.tim2_cr1 & 0x1U);
  UartWrite(" cc1e=");
  UartWriteU32(raw.tim2_ccer & 0x1U);
  UartWrite(" cc2e=");
  UartWriteU32((raw.tim2_ccer >> 4U) & 0x1U);
  UartWriteLine("  (expect sms=3 cen=1 cc1e=1 cc2e=1)");
  // 判读提示（写进日志，现场不用记）：两相恒为同一电平且 CNT 不变 ⇒ 很可能悬空/未接。
  UartWrite("enc-raw: cum_l=");
  UartWriteI32(static_cast<int32_t>(encoder_input_.cumulative_left()));
  UartWrite(" cum_r=");
  UartWriteI32(static_cast<int32_t>(encoder_input_.cumulative_right()));
  UartWriteLine("  (since boot; divide by turn count to get CPR)");
  UartWriteLine("enc-raw: pa0/pa1=RIGHT wheel enc, pa6/pa7=LEFT wheel enc; static+both-high => likely unconnected");
}

void BoardApp::PrintInfo() {
  UartWrite("info: fw=006.5 build=");
  UartWrite(__DATE__);
  UartWrite(" ");
  UartWriteLine(__TIME__);
  UartWrite("info: cpr=");
  UartWriteU32(static_cast<uint32_t>(kEncoderCountsPerWheelRev));
  UartWrite(" radius_mm=");
  UartWriteU32(static_cast<uint32_t>(kWheelRadiusM * 1000.0f));
  UartWrite(" pwm_hz=");
  UartWriteU32(kPwmFrequencyHz);
  UartWrite(" pwm_arr=");
  UartWriteU32(kPwmPeriodCounts - 1U);
  UartWrite(" wdt_ms=");
  UartWriteU32(kMotorWatchdogTimeoutMs);
  UartWriteLine("");
  UartWrite("info: ctrl kff=");
  UartWriteU32(static_cast<uint32_t>(controller_config_.kff_permille_per_mps));
  UartWrite(" kp=");
  UartWriteU32(static_cast<uint32_t>(controller_config_.kp_permille_per_mps));
  UartWrite(" ki=");
  UartWriteU32(static_cast<uint32_t>(controller_config_.ki_permille_per_mps_s));
  UartWrite(" max_duty=");
  UartWriteU32(controller_config_.max_duty_permille);
  UartWrite(" min_eff=");
  UartWriteU32(controller_config_.min_effective_duty_permille);
  UartWrite(" floor_mmps=");
  UartWriteU32(static_cast<uint32_t>(controller_config_.measurement_floor_mps * 1000.0f));
  UartWriteLine("");
  UartWrite("info: enc loc=output_shaft bits=");
  UartWriteU32(kEncoderCounterBits);
  UartWrite(" invert_l=");
  UartWriteU32(kEncoderLeftInverted ? 1U : 0U);
  UartWrite(" invert_r=");
  UartWriteU32(kEncoderRightInverted ? 1U : 0U);
  UartWrite(" motor_inv_l=");
  UartWriteU32(kMotorLeftDirectionInverted ? 1U : 0U);
  UartWrite(" motor_inv_r=");
  UartWriteU32(kMotorRightDirectionInverted ? 1U : 0U);
  UartWrite(" tick_us=1000 tasks=3");
  UartWriteLine("");
}

void BoardApp::PrintStatusLine(uint8_t index) {
  const control::ArbitrationStats& arb = arbitration_.stats();
  switch (index) {
    case 0: {
      UartWrite("t=");
      UartWriteU32(SysTickClockSource::MillisSinceBoot());
      UartWrite(" enc: L c=");
      UartWriteI32(wheel_state_.left.count_delta);
      UartWrite(" v=");
      UartWriteI32(ToMilli(wheel_state_.left.speed_mps));
      UartWrite(" q=");
      UartWriteHex32(wheel_state_.left.quality);
      UartWrite(" pm=");
      UartWriteI32(ToMilli(wheel_state_.left.position_m));  // 累计位移 mm：手转一圈直接读 CPR
      UartWrite(" R c=");
      UartWriteI32(wheel_state_.right.count_delta);
      UartWrite(" v=");
      UartWriteI32(ToMilli(wheel_state_.right.speed_mps));
      UartWrite(" q=");
      UartWriteHex32(wheel_state_.right.quality);
      UartWrite(" pm=");
      UartWriteI32(ToMilli(wheel_state_.right.position_m));
      UartWriteLine("");
      return;
    }
    case 1: {
      UartWrite("tgt: L ");
      UartWriteI32(ToMilli(target_.left_mps));
      UartWrite(" R ");
      UartWriteI32(ToMilli(target_.right_mps));
      UartWrite(" mode=");
      UartWrite(mode_ == MotorMode::kClosed ? "closed" : "open");
      UartWrite(" wheel=");
      UartWrite(wheels_ == WheelSelection::kBoth
                    ? "both"
                    : (wheels_ == WheelSelection::kLeft ? "left" : "right"));
      UartWrite(" od=");
      UartWriteU32(open_duty_permille_);
      UartWrite(" dir=");
      UartWrite(open_direction_ == control::MotorDirection::kForward ? "fwd" : "rev");
      UartWriteLine("");
      return;
    }
    case 2: {
      UartWrite("out: L m=");
      UartWriteU32(static_cast<uint32_t>(ctx_.last_pair.left.mode));
      UartWrite(" d=");
      UartWriteU32(static_cast<uint32_t>(ctx_.last_pair.left.direction));
      UartWrite(" p=");
      UartWriteU32(ctx_.last_pair.left.duty_permille);
      UartWrite(" R m=");
      UartWriteU32(static_cast<uint32_t>(ctx_.last_pair.right.mode));
      UartWrite(" d=");
      UartWriteU32(static_cast<uint32_t>(ctx_.last_pair.right.direction));
      UartWrite(" p=");
      UartWriteU32(ctx_.last_pair.right.duty_permille);
      UartWriteLine("");
      return;
    }
    case 3: {
      UartWrite("arb: t=");
      UartWriteU32(arb.ticks);
      UartWrite(" a=");
      UartWriteU32(arb.apply_calls);
      UartWrite(" drv=");
      UartWriteU32(arb.drive_applies);
      UartWrite(" st=");
      UartWriteU32(arb.stale_zero);
      UartWrite(" sf=");
      UartWriteU32(arb.safety_zero);
      UartWrite(" ft=");
      UartWriteU32(arb.fault_zero);
      UartWrite(" hw=");
      UartWriteU32(arb.platform_errors);
      UartWriteLine("");
      return;
    }
    case 4: {
      UartWrite("est: smp=");
      UartWriteU32(estimator_.stats().samples_submitted);
      UartWrite(" acc=");
      UartWriteU32(estimator_.stats().samples_accepted);
      UartWrite(" inv=");
      UartWriteU32(estimator_.stats().snapshot_invalid);
      UartWrite(" cmd=");
      UartWriteU32(console_.stats().bytes);
      UartWrite(" unk=");
      UartWriteU32(console_.stats().unknown);
      UartWrite(" ovf=");
      UartWriteU32(UartConsoleRxOverflowCount());
      UartWrite(" sw=");
      UartWriteU32(sink_.switch_zero_windows());
      UartWrite(" rej=");
      UartWriteU32(sink_.rejected_requests());  // 平台拒绝的非法请求（与 arb.hw 区分）
      UartWrite(" wd=");
      UartWriteU32(sink_.watchdog_trips());  // 主循环卡死触发强制失能的次数（INV-006.5-6）
      UartWriteLine("");
      return;
    }
    default:
      return;
  }
}

}  // namespace robotcar01::platform
