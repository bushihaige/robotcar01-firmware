// 命令管理与租约（Iteration 003）
//
// 职责：把"已通过线格式校验的帧"变成唯一的命令事实来源——数值/范围校验、时序/迟到判定、
// 序号回绕安全、会话代次、显式 arm 运行门、租约有效期。不做安全裁决（007 职责）。
//
// 关键契约（003 详细设计 §会话与运行语义 / §租约语义）：
//   - INV-003-1：任何拒绝路径（迟到、挂起、数值非法、方向错、序号倒退）都不改变已发布快照；
//   - D-003-1：run_requested **不是运行许可**。会话首帧永不使能；armed 只能由"未请求运行的
//     有效命令"或显式 Stop 帧置位；SAFE_IDLE→RUNNING 的唯一裁决在 007；
//   - D-003-4：接受条件 (seq - last_seq) != 0 && diff < 0x80000000（新会话基线 0xFFFFFFFF）；
//   - D-003-5：生效租约 = clamp(声明或默认, min_lease_ms, max_lease_ms)；
//   - D-003-12：迟到帧（send_age_ms > max_send_age_ms）拒绝且不刷新时间戳；
//   - INV-003-8：租约判定唯一实现为 CommandValidAt；snapshot().command.valid 恒表示
//     "会话内已接受过合法命令"，消费者禁止直读它判定新鲜度（必须用 CommandValidAt /
//     EffectiveSnapshot）。
//
// 无动态内存、无 HAL 依赖、不持有 ClockSource（时间戳由调用方传入，便于注入 fake 时钟）。

#ifndef ROBOTCAR01_APP_PROTOCOL_COMMAND_MANAGER_H_
#define ROBOTCAR01_APP_PROTOCOL_COMMAND_MANAGER_H_

#include <cstdint>

#include "app/chassis/chassis_types.h"
#include "app/protocol/decoder.h"
#include "app/protocol/frame.h"

namespace robotcar01::protocol {

// 租约与迟到阈值（注入用值对象；取值来自应用配置，协议库不内建生效值，D-003-9）。
struct CommandLeaseConfig {
  uint32_t default_lease_ms = 300;  // 占位非标定（上位机 20–50Hz ⇒ 300ms ≈ 6–15 帧丢失）
  uint32_t min_lease_ms = 50;       // 下限：防上位机用极短租约持续刷新绕过停止预算
  uint32_t max_lease_ms = 2000;     // 上限：防上位机声明无限租约
  uint32_t max_send_age_ms = 100;   // 迟到帧上限；0 = 不启用迟到判定
};

// 唯一命令快照。除 session_generation 外，所有字段在拒绝路径上保持不变。
struct CommandSnapshot {
  chassis::MotionCommand command{};  // command.valid = "会话内已接受过合法命令"（不随时间变化）
  uint64_t received_at_us = 0;       // 帧被接受的时刻（单调 µs）
  uint64_t valid_until_us = 0;       // = received_at_us + 生效租约（µs）
  uint32_t session_generation = 0;   // 会话代次（重连 +1；回绕仅作诊断，不参与判定）
  uint32_t lease_ms = 0;             // 生效租约
  bool armed = false;                // 显式 arm 门（D-003-1）
  bool run_requested = false;        // 客户端运行请求；**非运行许可**
  bool stop_requested = false;       // 最近一次被接受的命令是 Stop
  bool has_command = false;          // 本会话是否收到过合法命令
};

struct CommandManagerStats {
  uint32_t accepted = 0;
  uint32_t rejected = 0;              // 数值/范围/迟到/方向/挂起等业务拒绝合计
  uint32_t stops = 0;
  uint32_t seq_rejected = 0;          // 序号重复/倒退/恰好半程（协议层判定，计数器在本模块）
  uint32_t run_requests_ignored = 0;  // 会话首帧携带 run 请求（被忽略，A4）
  uint32_t stale_frames = 0;
  uint32_t suspended_rejected = 0;
  uint32_t wrong_direction = 0;
  uint32_t session_resets = 0;

  void Reset() { *this = CommandManagerStats{}; }
};

class CommandManager {
 public:
  // 注入配置（值对象来自应用配置，D-003-9）并复位到"无会话、无命令"状态。
  void Init(const CommandLeaseConfig& lease, const CommandLimits& limits);

  // 会话重置语义：必须经 ResetCommandSession() 调用本函数（唯一入口，见 session.h）。
  void ResetSession();

  bool suspended() const { return suspended_; }

  // 消费一帧（前置条件：帧为 DecodeStatus::kFrameReady，即线格式已通过）。
  // 返回 true 表示快照被更新；任何拒绝路径返回 false 且不改动快照（INV-003-1）。
  bool OnFrame(const DecodedFrame& frame, uint64_t now_us);

  // 急停/安全禁止期间调用（D-003-11）：运动帧一律拒绝、不刷新时间戳、不置 arm。
  void Suspend();
  // 恢复：armed 保持 false，必须重新收到一帧"未请求运行"的有效命令才可再请求运行。
  void Resume();

  const CommandSnapshot& snapshot() const { return snapshot_; }

  // 租约唯一判定（INV-003-8）。
  bool CommandValidAt(uint64_t now_us) const;
  // 无有效命令时返回 0；有命令则返回 now - received_at_us（单调时基保证 >= 0）。
  uint32_t AgeUs(uint64_t now_us) const;
  // 消费方使用的快照：command.valid 按 now 计算（不用改写原始快照）。
  CommandSnapshot EffectiveSnapshot(uint64_t now_us) const;

  const CommandManagerStats& stats() const { return stats_; }

 private:
  bool ValidatePayload(const DecodedFrame& frame) const;
  bool IsSequenceAcceptable(uint32_t seq) const;
  void RejectBusiness();
  uint32_t EffectiveLeaseMs(uint16_t declared_lease_ms) const;

  CommandLeaseConfig lease_{};
  CommandLimits limits_{};
  CommandSnapshot snapshot_{};
  CommandManagerStats stats_{};
  uint32_t last_seq_ = 0xFFFFFFFFu;  // 新会话基线（D-003-4）
  bool has_last_seq_ = false;
  bool suspended_ = false;
  // arm 门是否已建立："尚未建立"时下一帧运动命令的 run 请求被忽略（保证显式 arm，D-003-1）。
  // 会话开始与 Resume 后均为 false。
  bool armed_gate_open_ = false;
};

}  // namespace robotcar01::protocol

#endif  // ROBOTCAR01_APP_PROTOCOL_COMMAND_MANAGER_H_
