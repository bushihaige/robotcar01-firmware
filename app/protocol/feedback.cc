#include "app/protocol/feedback.h"

#include "app/protocol/crc16.h"

namespace robotcar01::protocol {

namespace {

// 状态帧载荷字段偏移（小端）
enum StatusOffset : size_t {
  kStatusFlags = 0,
  kLimitReason = 4,
  kFlagsValid = 8,
  kCommandSeq = 12,
  kCommandAgeUs = 16,
  kSessionGeneration = 20,
  kCommandFlags = 24,
  kTargetLeft = 28,
  kTargetRight = 32,
  kOutputLeft = 36,
  kOutputRight = 40,
  kMeasuredLeft = 44,
  kMeasuredRight = 48,
  kStatusReserved = 52,
};

// 诊断帧载荷字段偏移（小端）：104 B 统计与装配 + 6×12 B 任务槽 = 176 B
enum DiagOffset : size_t {
  kUptimeMs = 0,
  kProtocolAccepted = 4,
  kProtocolRejected = 8,
  kBadMagic = 12,
  kBadVersion = 16,
  kBadFlag = 20,
  kBadType = 24,
  kBadLength = 28,
  kBadCrc = 32,
  kBadReserved = 36,
  kCommandAccepted = 40,
  kCommandRejected = 44,
  kCommandStops = 48,
  kSeqRejected = 52,
  kRunRequestsIgnored = 56,
  kStaleFrames = 60,
  kSuspendedRejected = 64,
  kWrongDirection = 68,
  kSessionResets = 72,
  kRingOverflow = 76,
  kRingHighWater = 80,
  kLimitsConfigValid = 84,
  kTaskCount = 88,
  kDroppedTaskCount = 92,
  kDiagFlagBits = 96,
  kDiagReserved = 100,
  kTasksFirst = 104,
};

// 编译期校验：偏移表与字段总长必须恰好等于声明的诊断载荷长度（防止再次出现长度/偏移不一致）。
static_assert(kTasksFirst + kMaxDiagTaskSlots * kDiagTaskEntryBytes == kDiagPayloadBytes,
              "diag payload offsets must exactly fill kDiagPayloadBytes");
static_assert(kStatusReserved + 3 * sizeof(uint32_t) == kStatusPayloadBytes,
              "status payload offsets must exactly fill kStatusPayloadBytes");

constexpr uint32_t kDiagFlagTruncated = 1u << 0;

// 命令标志位（FeedbackStatusPayload::command_flags 的位语义）
constexpr uint32_t kCommandFlagRunRequested = 1u << 0;
constexpr uint32_t kCommandFlagStopRequested = 1u << 1;

// 组装完整反馈帧（与命令帧共用外层格式）。返回总字节数；容量不足返回 0。
size_t AssembleFeedbackFrame(uint8_t msg_type, const uint8_t* payload, size_t payload_len,
                             uint32_t seq, uint8_t* out, size_t out_capacity) {
  const size_t total = kFrameHeaderBytes + payload_len + kCrcBytes;
  if (out == nullptr || total > out_capacity) {
    return 0;
  }
  out[0] = kMagicByte0;
  out[1] = kMagicByte1;
  out[2] = kMagicByte2;
  out[3] = kMagicByte3;
  FrameHeader header{};
  header.msg_type = msg_type;
  header.payload_len = static_cast<uint8_t>(payload_len);
  header.seq = seq;
  EncodeHeader(header, out + 4);
  for (size_t i = 0; i < payload_len; ++i) {
    out[kFrameHeaderBytes + i] = payload[i];
  }
  PutLe16(out + kFrameHeaderBytes + payload_len,
          Crc16CcittFalse(out, kFrameHeaderBytes + payload_len));
  return total;
}

}  // namespace

FeedbackStatusPayload AssembleStatusPayload(const FeedbackInputs& inputs, uint64_t now_us) {
  FeedbackStatusPayload payload{};
  const bool has_command = inputs.command != nullptr;
  const bool has_target = inputs.target != nullptr;
  const bool has_output = inputs.output != nullptr;

  // 1) 基础状态位（装配方提供的输入，恒有效）
  uint32_t flags = 0;
  uint32_t valid = 0;
  if (inputs.initialized) {
    flags |= kFlagInitialized;
  }
  if (inputs.fault_latched) {
    flags |= kFlagFaultLatched;
  }
  valid |= kFlagInitialized | kFlagFaultLatched;

  // 2) 命令维度
  if (has_command) {
    const CommandSnapshot& snapshot = *inputs.command;
    if (snapshot.armed) {
      flags |= kFlagArmed;
    }
    if (snapshot.run_requested) {
      flags |= kFlagRunRequested;
    }
    if (snapshot.stop_requested) {
      flags |= kFlagStopRequested;
    }
    payload.command_seq = snapshot.command.seq;
    payload.session_generation = snapshot.session_generation;
    payload.command_flags = (snapshot.run_requested ? kCommandFlagRunRequested : 0u) |
                            (snapshot.stop_requested ? kCommandFlagStopRequested : 0u);
    const bool fresh = snapshot.has_command && now_us <= snapshot.valid_until_us;
    if (fresh) {
      flags |= kFlagCommandFresh;
      const uint64_t age = now_us - snapshot.received_at_us;
      payload.command_age_us = (age > 0xFFFFFFFFull) ? 0xFFFFFFFFu : static_cast<uint32_t>(age);
    } else {
      payload.command_age_us = 0u;  // 无有效命令时不回传可能误导的 age
    }
    valid |= kFlagArmed | kFlagRunRequested | kFlagStopRequested | kFlagCommandFresh;
  }

  // 3) 控制维度（004/006 前由调用方传 nullptr）
  if (has_target) {
    flags |= kFlagTargetValid;
    valid |= kFlagTargetValid;
    payload.target_left_mps = inputs.target->left_mps;
    payload.target_right_mps = inputs.target->right_mps;
  }
  if (has_output) {
    if (inputs.output->force_disable) {
      flags |= kFlagForceDisable;
    }
    valid |= kFlagForceDisable;
    payload.output_left_mps = inputs.output->left_mps;
    payload.output_right_mps = inputs.output->right_mps;
    payload.limit_reason = static_cast<uint32_t>(inputs.output->limit_reason);
  } else if (has_target) {
    payload.limit_reason = static_cast<uint32_t>(inputs.target->limit_reason);
  }
  if (inputs.safety != nullptr && !inputs.safety->allow_motion) {
    payload.limit_reason |= static_cast<uint32_t>(chassis::LimitReason::kSafety);
  }
  // 004 之前无轮速/编码器数据源：kFlagSpeedValid / kFlagEncoderValid 恒不置位

  payload.status_flags = flags;
  payload.flags_valid = valid;
  return payload;
}

FeedbackDiagPayload AssembleDiagPayload(const mcu_os_lite::TaskSet* tasks, const ProtocolStats& stats,
                                       const CommandManagerStats& command_stats,
                                       const ByteRingStatistics& ring, bool limits_config_valid,
                                       uint64_t now_ms) {
  FeedbackDiagPayload payload{};
  payload.uptime_ms = (now_ms > 0xFFFFFFFFull) ? 0xFFFFFFFFu : static_cast<uint32_t>(now_ms);
  // 线格式类计数（解码器）
  payload.protocol_frames_accepted = stats.frames_accepted;
  payload.protocol_frames_rejected = stats.frames_rejected;
  payload.bad_magic = stats.bad_magic;
  payload.bad_version = stats.bad_version;
  payload.bad_flag = stats.bad_flag;
  payload.bad_type = stats.bad_type;
  payload.bad_length = stats.bad_length;
  payload.bad_crc = stats.bad_crc;
  payload.bad_reserved = stats.bad_reserved;
  // 会话/序号/迟到/挂起类计数（命令管理器，评审 H7：单一 writer）
  payload.command_accepted = command_stats.accepted;
  payload.command_rejected = command_stats.rejected;
  payload.command_stops = command_stats.stops;
  payload.seq_rejected = command_stats.seq_rejected;
  payload.run_requests_ignored = command_stats.run_requests_ignored;
  payload.stale_frames = command_stats.stale_frames;
  payload.suspended_rejected = command_stats.suspended_rejected;
  payload.wrong_direction_frames = command_stats.wrong_direction;
  payload.session_resets = command_stats.session_resets;
  // 字节环
  payload.ring_overflow_bytes = ring.overflow_drop_bytes;  // 单一真源（INV-003-9）
  payload.ring_high_water = ring.high_water_bytes;
  payload.limits_config_valid = limits_config_valid ? 1u : 0u;

  uint32_t copied = 0;
  uint32_t dropped = 0;
  if (tasks != nullptr) {
    for (size_t i = 0; i < tasks->size(); ++i) {
      if (copied < kMaxDiagTaskSlots) {
        const mcu_os_lite::TaskEntry& entry = tasks->at(i);
        FeedbackDiagTaskEntry& slot = payload.tasks[copied];
        // uint64 → uint32 窄化必须饱和，不得静默回绕
        slot.run_count = (entry.stats.run_count > 0xFFFFFFFFull)
                             ? 0xFFFFFFFFu
                             : static_cast<uint32_t>(entry.stats.run_count);
        slot.overrun_count = entry.stats.overrun_count;
        slot.max_elapsed_us = (entry.stats.max_elapsed_us > 0xFFFFFFFFull)
                                  ? 0xFFFFFFFFu
                                  : static_cast<uint32_t>(entry.stats.max_elapsed_us);
        ++copied;
      } else {
        ++dropped;
      }
    }
  }
  payload.task_count = copied;
  payload.dropped_task_count = dropped;
  payload.flag_bits = (dropped == 0u) ? 0u : kDiagFlagTruncated;
  return payload;
}

size_t EncodeStatusFrame(const FeedbackStatusPayload& payload, uint32_t seq, uint8_t* out,
                         size_t out_capacity) {
  uint8_t body[kStatusPayloadBytes] = {};
  PutLe32(body + kStatusFlags, payload.status_flags);
  PutLe32(body + kLimitReason, payload.limit_reason);
  PutLe32(body + kFlagsValid, payload.flags_valid);
  PutLe32(body + kCommandSeq, payload.command_seq);
  PutLe32(body + kCommandAgeUs, payload.command_age_us);
  PutLe32(body + kSessionGeneration, payload.session_generation);
  PutLe32(body + kCommandFlags, payload.command_flags);
  PutLe32(body + kTargetLeft, FloatBits(payload.target_left_mps));
  PutLe32(body + kTargetRight, FloatBits(payload.target_right_mps));
  PutLe32(body + kOutputLeft, FloatBits(payload.output_left_mps));
  PutLe32(body + kOutputRight, FloatBits(payload.output_right_mps));
  PutLe32(body + kMeasuredLeft, FloatBits(payload.measured_left_mps));
  PutLe32(body + kMeasuredRight, FloatBits(payload.measured_right_mps));
  for (size_t i = 0; i < 3; ++i) {
    PutLe32(body + kStatusReserved + i * 4, payload.reserved[i]);
  }
  return AssembleFeedbackFrame(static_cast<uint8_t>(MessageType::kFeedbackStatus), body,
                               kStatusPayloadBytes, seq, out, out_capacity);
}

size_t EncodeDiagFrame(const FeedbackDiagPayload& payload, uint32_t seq, uint8_t* out,
                       size_t out_capacity) {
  uint8_t body[kDiagPayloadBytes] = {};
  PutLe32(body + kUptimeMs, payload.uptime_ms);
  PutLe32(body + kProtocolAccepted, payload.protocol_frames_accepted);
  PutLe32(body + kProtocolRejected, payload.protocol_frames_rejected);
  PutLe32(body + kBadMagic, payload.bad_magic);
  PutLe32(body + kBadVersion, payload.bad_version);
  PutLe32(body + kBadFlag, payload.bad_flag);
  PutLe32(body + kBadType, payload.bad_type);
  PutLe32(body + kBadLength, payload.bad_length);
  PutLe32(body + kBadCrc, payload.bad_crc);
  PutLe32(body + kBadReserved, payload.bad_reserved);
  PutLe32(body + kCommandAccepted, payload.command_accepted);
  PutLe32(body + kCommandRejected, payload.command_rejected);
  PutLe32(body + kCommandStops, payload.command_stops);
  PutLe32(body + kSeqRejected, payload.seq_rejected);
  PutLe32(body + kRunRequestsIgnored, payload.run_requests_ignored);
  PutLe32(body + kStaleFrames, payload.stale_frames);
  PutLe32(body + kSuspendedRejected, payload.suspended_rejected);
  PutLe32(body + kWrongDirection, payload.wrong_direction_frames);
  PutLe32(body + kSessionResets, payload.session_resets);
  PutLe32(body + kRingOverflow, payload.ring_overflow_bytes);
  PutLe32(body + kRingHighWater, payload.ring_high_water);
  PutLe32(body + kLimitsConfigValid, payload.limits_config_valid);
  PutLe32(body + kTaskCount, payload.task_count);
  PutLe32(body + kDroppedTaskCount, payload.dropped_task_count);
  PutLe32(body + kDiagFlagBits, payload.flag_bits);
  PutLe32(body + kDiagReserved, payload.reserved);
  for (size_t i = 0; i < kMaxDiagTaskSlots; ++i) {
    const size_t base = kTasksFirst + i * kDiagTaskEntryBytes;
    PutLe32(body + base + 0, payload.tasks[i].run_count);
    PutLe32(body + base + 4, payload.tasks[i].overrun_count);
    PutLe32(body + base + 8, payload.tasks[i].max_elapsed_us);
  }
  return AssembleFeedbackFrame(static_cast<uint8_t>(MessageType::kFeedbackDiag), body,
                               kDiagPayloadBytes, seq, out, out_capacity);
}

bool DecodeStatusPayload(const DecodedFrame& frame, FeedbackStatusPayload& out) {
  if (!frame.valid || frame.header.msg_type != static_cast<uint8_t>(MessageType::kFeedbackStatus) ||
      frame.header.payload_len != kStatusPayloadBytes) {
    return false;
  }
  const uint8_t* body = frame.payload;
  FeedbackStatusPayload result{};
  result.status_flags = GetLe32(body + kStatusFlags);
  result.limit_reason = GetLe32(body + kLimitReason);
  result.flags_valid = GetLe32(body + kFlagsValid);
  result.command_seq = GetLe32(body + kCommandSeq);
  result.command_age_us = GetLe32(body + kCommandAgeUs);
  result.session_generation = GetLe32(body + kSessionGeneration);
  result.command_flags = GetLe32(body + kCommandFlags);
  result.target_left_mps = BitsToFloat(GetLe32(body + kTargetLeft));
  result.target_right_mps = BitsToFloat(GetLe32(body + kTargetRight));
  result.output_left_mps = BitsToFloat(GetLe32(body + kOutputLeft));
  result.output_right_mps = BitsToFloat(GetLe32(body + kOutputRight));
  result.measured_left_mps = BitsToFloat(GetLe32(body + kMeasuredLeft));
  result.measured_right_mps = BitsToFloat(GetLe32(body + kMeasuredRight));
  for (size_t i = 0; i < 3; ++i) {
    result.reserved[i] = GetLe32(body + kStatusReserved + i * 4);
  }
  out = result;
  return true;
}

bool DecodeDiagPayload(const DecodedFrame& frame, FeedbackDiagPayload& out) {
  if (!frame.valid || frame.header.msg_type != static_cast<uint8_t>(MessageType::kFeedbackDiag) ||
      frame.header.payload_len != kDiagPayloadBytes) {
    return false;
  }
  const uint8_t* body = frame.payload;
  FeedbackDiagPayload result{};
  result.uptime_ms = GetLe32(body + kUptimeMs);
  result.protocol_frames_accepted = GetLe32(body + kProtocolAccepted);
  result.protocol_frames_rejected = GetLe32(body + kProtocolRejected);
  result.bad_magic = GetLe32(body + kBadMagic);
  result.bad_version = GetLe32(body + kBadVersion);
  result.bad_flag = GetLe32(body + kBadFlag);
  result.bad_type = GetLe32(body + kBadType);
  result.bad_length = GetLe32(body + kBadLength);
  result.bad_crc = GetLe32(body + kBadCrc);
  result.bad_reserved = GetLe32(body + kBadReserved);
  result.command_accepted = GetLe32(body + kCommandAccepted);
  result.command_rejected = GetLe32(body + kCommandRejected);
  result.command_stops = GetLe32(body + kCommandStops);
  result.seq_rejected = GetLe32(body + kSeqRejected);
  result.run_requests_ignored = GetLe32(body + kRunRequestsIgnored);
  result.stale_frames = GetLe32(body + kStaleFrames);
  result.suspended_rejected = GetLe32(body + kSuspendedRejected);
  result.wrong_direction_frames = GetLe32(body + kWrongDirection);
  result.session_resets = GetLe32(body + kSessionResets);
  result.ring_overflow_bytes = GetLe32(body + kRingOverflow);
  result.ring_high_water = GetLe32(body + kRingHighWater);
  result.limits_config_valid = GetLe32(body + kLimitsConfigValid);
  result.task_count = GetLe32(body + kTaskCount);
  result.dropped_task_count = GetLe32(body + kDroppedTaskCount);
  result.flag_bits = GetLe32(body + kDiagFlagBits);
  result.reserved = GetLe32(body + kDiagReserved);
  for (size_t i = 0; i < kMaxDiagTaskSlots; ++i) {
    const size_t base = kTasksFirst + i * kDiagTaskEntryBytes;
    result.tasks[i].run_count = GetLe32(body + base + 0);
    result.tasks[i].overrun_count = GetLe32(body + base + 4);
    result.tasks[i].max_elapsed_us = GetLe32(body + base + 8);
  }
  out = result;
  return true;
}

}  // namespace robotcar01::protocol
