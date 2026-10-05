#include "app/protocol/command_manager.h"

#include <cmath>
#include <cstdint>
#include <limits>

namespace robotcar01::protocol {

namespace {

constexpr bool IsFinite(float value) {
  return value == value && value <= std::numeric_limits<float>::max() &&
         value >= -std::numeric_limits<float>::max();
}

uint32_t ClampU32(uint32_t value, uint32_t low, uint32_t high) {
  if (low > high) {
    return high;
  }
  if (value < low) {
    return low;
  }
  if (value > high) {
    return high;
  }
  return value;
}

}  // namespace

void CommandManager::Init(const CommandLeaseConfig& lease, const CommandLimits& limits) {
  lease_ = lease;
  limits_ = limits;
  stats_ = CommandManagerStats{};
  suspended_ = false;
  has_last_seq_ = false;
  last_seq_ = 0xFFFFFFFFu;
  snapshot_ = CommandSnapshot{};
  snapshot_.session_generation = 1;
}

void CommandManager::ResetSession() {
  ++snapshot_.session_generation;
  ++stats_.session_resets;
  has_last_seq_ = false;
  last_seq_ = 0xFFFFFFFFu;  // D-003-4
  armed_gate_open_ = false;
  const uint32_t generation = snapshot_.session_generation;
  snapshot_ = CommandSnapshot{};
  snapshot_.session_generation = generation;
  suspended_ = false;
}

void CommandManager::Suspend() { suspended_ = true; }

void CommandManager::Resume() {
  suspended_ = false;
  // arm 门复位：恢复后必须先有一帧"未请求运行"的有效命令（D-003-11）
  snapshot_.armed = false;
  armed_gate_open_ = false;
}

bool CommandManager::ValidatePayload(const DecodedFrame& frame) const {
  if (frame.header.msg_type != static_cast<uint8_t>(MessageType::kMotionCommand)) {
    return true;  // 停止帧：无载荷字段
  }
  const MotionPayload payload = DecodeMotionPayload(frame.payload, frame.header.payload_len);
  if (!IsFinite(payload.v_mps) || !IsFinite(payload.omega_radps)) {
    return false;
  }
  if (limits_.max_body_speed_mps > 0.0f &&
      std::fabs(payload.v_mps) > limits_.max_body_speed_mps) {
    return false;
  }
  if (limits_.max_yaw_rate_radps > 0.0f &&
      std::fabs(payload.omega_radps) > limits_.max_yaw_rate_radps) {
    return false;
  }
  return true;
}

bool CommandManager::IsSequenceAcceptable(uint32_t seq) const {
  if (!has_last_seq_) {
    return true;  // 新会话首个序号（基线 0xFFFFFFFF ⇒ 0 与非零均被接受）
  }
  const uint32_t diff = seq - last_seq_;  // uint32 模差（回绕安全）
  return diff != 0u && diff < 0x80000000u;
}

void CommandManager::RejectBusiness() { ++stats_.rejected; }

uint32_t CommandManager::EffectiveLeaseMs(uint16_t declared_lease_ms) const {
  const uint32_t declared = (declared_lease_ms == 0u) ? lease_.default_lease_ms : declared_lease_ms;
  return ClampU32(declared, lease_.min_lease_ms, lease_.max_lease_ms);
}

bool CommandManager::OnFrame(const DecodedFrame& frame, uint64_t now_us) {
  if (!frame.valid) {
    return false;
  }
  const bool is_motion = frame.header.msg_type == static_cast<uint8_t>(MessageType::kMotionCommand);
  const bool is_stop = frame.header.msg_type == static_cast<uint8_t>(MessageType::kStopCommand);
  if (!is_motion && !is_stop) {
    ++stats_.wrong_direction;  // 反馈方向帧到达命令路径（A3.4）
    return false;
  }

  const uint32_t seq = frame.header.seq;

  if (suspended_) {
    if (is_motion) {
      // 挂起期间运动帧一律拒绝：不推进序号基线、不刷新时间戳、不置 arm（D-003-11/A3.8）
      ++stats_.suspended_rejected;
      RejectBusiness();
      return false;
    }
    if (!IsSequenceAcceptable(seq)) {
      ++stats_.seq_rejected;
      RejectBusiness();
      return false;
    }
    // 停止帧在挂起期间仍被接受，但**不刷新时间戳、不置 arm**（D-003-11）
    has_last_seq_ = true;
    last_seq_ = seq;
    ++stats_.accepted;
    ++stats_.stops;
    snapshot_.run_requested = false;
    snapshot_.stop_requested = true;
    snapshot_.command.v_mps = 0.0f;
    snapshot_.command.omega_radps = 0.0f;
    snapshot_.command.valid = false;
    return true;
  }

  if (!IsSequenceAcceptable(seq)) {
    ++stats_.seq_rejected;
    RejectBusiness();
    return false;
  }

  if (is_motion && lease_.max_send_age_ms > 0u &&
      static_cast<uint32_t>(frame.payload[kMotionPayloadBytes - 1]) > lease_.max_send_age_ms) {
    ++stats_.stale_frames;
    RejectBusiness();
    return false;  // 迟到帧：不刷新 received_at_us（D-003-12）
  }

  if (!ValidatePayload(frame)) {
    RejectBusiness();
    return false;
  }

  const uint32_t effective_lease_ms =
      EffectiveLeaseMs(is_motion ? DecodeMotionPayload(frame.payload, frame.header.payload_len).lease_ms
                                 : 0u);
  const bool is_first_frame = !armed_gate_open_;

  // --- 提交：此后不允许再走任何拒绝路径（保持"接受即完整更新"的原子语义）---
  has_last_seq_ = true;
  last_seq_ = seq;
  ++stats_.accepted;
  snapshot_.has_command = true;
  snapshot_.command.seq = seq;
  snapshot_.received_at_us = now_us;
  snapshot_.valid_until_us = now_us + static_cast<uint64_t>(effective_lease_ms) * 1000ull;
  snapshot_.lease_ms = effective_lease_ms;

  if (is_stop) {
    ++stats_.stops;
    snapshot_.command.v_mps = 0.0f;
    snapshot_.command.omega_radps = 0.0f;
    snapshot_.command.valid = false;
    snapshot_.run_requested = false;
    snapshot_.stop_requested = true;
    snapshot_.armed = true;      // 显式停止构成 arm（D-003-1）
    armed_gate_open_ = true;
    return true;
  }

  const MotionPayload payload = DecodeMotionPayload(frame.payload, frame.header.payload_len);
  snapshot_.command.v_mps = payload.v_mps;
  snapshot_.command.omega_radps = payload.omega_radps;
  snapshot_.command.valid = true;
  snapshot_.stop_requested = false;

  if (is_first_frame) {
    // 会话首帧（或 Resume 后首帧）：永不使能（arch §10）；若带 run 请求则忽略并计数
    armed_gate_open_ = true;
    snapshot_.armed = true;
    snapshot_.run_requested = false;
    if (payload.run_requested) {
      ++stats_.run_requests_ignored;
    }
    return true;
  }

  if (!payload.run_requested) {
    snapshot_.armed = true;  // 未请求运行的命令即构成 arm
    armed_gate_open_ = true;
  }
  snapshot_.run_requested = payload.run_requested;
  return true;
}

bool CommandManager::CommandValidAt(uint64_t now_us) const {
  return snapshot_.has_command && now_us <= snapshot_.valid_until_us;  // 含边界：valid_until 时刻仍有效
}

uint32_t CommandManager::AgeUs(uint64_t now_us) const {
  if (!snapshot_.has_command) {
    return 0u;
  }
  const uint64_t age = now_us - snapshot_.received_at_us;  // 单调时基 ⇒ 非负
  return (age > 0xFFFFFFFFull) ? 0xFFFFFFFFu : static_cast<uint32_t>(age);
}

CommandSnapshot CommandManager::EffectiveSnapshot(uint64_t now_us) const {
  CommandSnapshot result = snapshot_;
  result.command.valid = CommandValidAt(now_us);
  return result;
}

}  // namespace robotcar01::protocol
