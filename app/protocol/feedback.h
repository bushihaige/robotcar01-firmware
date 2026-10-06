// 反馈契约与帧编解码（Iteration 003）
//
// 字段来源（003 详细设计 §API / arch §13）：原始命令序号、会话代次、命令标志（run/stop 请求）、
// 实际 WheelTarget、实际输出 WheelSpeedRequest（含 force_disable）、合成后的限幅/安全原因、
// 数据年龄与命令新鲜度。
//
// 有效性语义（评审 H6）：flags_valid 是 status_flags 的**子集**——只有对应数据源可用时才允许
// 置位；缺失数据源时对应数值字段必须为 0 且不得被上位机读作"测得为零"。
// 发送失败不得影响控制路径（INV-003-6），本模块不含任何发送/阻塞逻辑。

#ifndef ROBOTCAR01_APP_PROTOCOL_FEEDBACK_H_
#define ROBOTCAR01_APP_PROTOCOL_FEEDBACK_H_

#include <cstddef>
#include <cstdint>

#include "app/chassis/chassis_types.h"
#include "app/encoder/wheel_state.h"
#include "app/mcu_os_lite/task_descriptor.h"
#include "app/protocol/byte_ring.h"
#include "app/protocol/command_manager.h"
#include "app/protocol/decoder.h"
#include "app/protocol/frame.h"
#include "app/protocol/protocol_constants.h"
#include "app/protocol/stats.h"

namespace robotcar01::protocol {

// 状态位。每个位在 flags_valid 中有对应位（子集语义见 §API 的对应表）。
enum FeedbackFlags : uint32_t {
  kFlagInitialized = 1u << 0,
  kFlagArmed = 1u << 1,
  kFlagRunRequested = 1u << 2,
  kFlagStopRequested = 1u << 3,
  kFlagFaultLatched = 1u << 4,
  kFlagCommandFresh = 1u << 5,
  kFlagTargetValid = 1u << 6,
  kFlagSpeedValid = 1u << 7,    // 004 起（轮速测量数据源）
  kFlagEncoderValid = 1u << 8,  // 004 起（编码器质量位）
  kFlagForceDisable = 1u << 9,
};

// 状态帧载荷（定长 64 B：52 B 字段 + 8 B 字段 + 4 B 保留，保留必须写 0）。
struct FeedbackStatusPayload {
  uint32_t status_flags = 0;
  uint32_t limit_reason = 0;
  uint32_t flags_valid = 0;
  uint32_t command_seq = 0;
  uint32_t command_age_us = 0;
  uint32_t session_generation = 0;
  uint32_t command_flags = 0;  // bit0 = run_requested（生效值），bit1 = stop_requested
  float target_left_mps = 0.0f;
  float target_right_mps = 0.0f;
  float output_left_mps = 0.0f;
  float output_right_mps = 0.0f;
  float measured_left_mps = 0.0f;   // 004 起有效
  float measured_right_mps = 0.0f;  // 004 起有效
  // 005 起：WheelTarget 的**域限幅比例**（不含加减速斜坡，D-005-6），仅 kFlagTargetValid
  // 置位时有意义；无 target 数据源时写 0（不得让上位机把默认值读成"未限幅"）。
  // 该字段占用 003 的 reserved[0]（长度不变，同一 v1 内的线格式语义变更，详设 §10）。
  float target_scale = 1.0f;
  // 005 起：逐侧编码器质量位（004 详设 §可观测性字段映射表的指派，005 落地补齐）。
  // 与 kFlagEncoderValid 的"两侧无阻塞位"语义配合：上位机可区分"整体不可信"与"哪一侧不可信"。
  uint32_t encoder_quality_left = 0;   // encoder::EncoderQuality 位组合
  uint32_t encoder_quality_right = 0;  // 无 wheel_state 数据源时写 0
};

// 诊断帧任务槽（线格式，逐字段定长；不含指针与名称字符串）。
// 说明：run_count / max_elapsed_us 在 mcu_os_lite 为 uint64，此处窄化为 uint32 并**饱和**
// 到 0xFFFFFFFF（不静默回绕）。
struct FeedbackDiagTaskEntry {
  uint32_t run_count = 0;
  uint32_t overrun_count = 0;
  uint32_t max_elapsed_us = 0;
};

// 诊断帧载荷（定长 192 B：104 B 统计与装配信息 + 4×4 B 编码器统计 + 6×12 B 任务槽）。
// 统计来源分工（评审 H7 的单一 writer 原则）：线格式类来自 ProtocolStats（解码器持有），
// 会话/序号/迟到/挂起类来自 CommandManagerStats（命令管理器持有），字节环来自 ByteRingStatistics。
struct FeedbackDiagPayload {
  // --- ProtocolStats（解码器）---
  uint32_t uptime_ms = 0;
  uint32_t protocol_frames_accepted = 0;
  uint32_t protocol_frames_rejected = 0;
  uint32_t bad_magic = 0;
  uint32_t bad_version = 0;
  uint32_t bad_flag = 0;
  uint32_t bad_type = 0;
  uint32_t bad_length = 0;
  uint32_t bad_crc = 0;
  uint32_t bad_reserved = 0;
  // --- CommandManagerStats（命令管理器）---
  uint32_t command_accepted = 0;
  uint32_t command_rejected = 0;
  uint32_t command_stops = 0;
  uint32_t seq_rejected = 0;
  uint32_t run_requests_ignored = 0;
  uint32_t stale_frames = 0;
  uint32_t suspended_rejected = 0;
  uint32_t wrong_direction_frames = 0;
  uint32_t session_resets = 0;
  // --- ByteRingStatistics（字节环）---
  uint32_t ring_overflow_bytes = 0;
  uint32_t ring_high_water = 0;
  // --- 装配信息 ---
  uint32_t limits_config_valid = 0;  // 1 = CommandLimits 已由应用配置填充（0 = 范围校验未启用）
  uint32_t task_count = 0;
  uint32_t dropped_task_count = 0;
  uint32_t flag_bits = 0;  // bit0 = truncated（dropped_task_count > 0）
  uint32_t reserved = 0;
  // --- 编码器统计（004；唯一 writer = EncoderEstimatorStats，映射表见 004 详设）---
  uint32_t encoder_samples = 0;
  uint32_t encoder_out_of_range = 0;
  uint32_t encoder_timestamp_invalid = 0;
  uint32_t encoder_hardware_faults = 0;
  FeedbackDiagTaskEntry tasks[kMaxDiagTaskSlots] = {};
};

// 组装输入：缺失数据源以 nullptr 表达（不以 0 冒充测量值）。
// 说明：measured_* 的数据源（WheelState）在 003 尚无类型；到 004 之前
// kFlagSpeedValid / kFlagEncoderValid 必须恒不置位。
struct FeedbackInputs {
  const CommandSnapshot* command = nullptr;
  const chassis::WheelTarget* target = nullptr;
  const chassis::WheelSpeedRequest* output = nullptr;
  const chassis::SafetyStatus* safety = nullptr;
  // 轮速测量数据源（004 起）。nullptr ⇒ 无数据源：kFlagSpeedValid/kFlagEncoderValid 清位、
  // measured_* 写 0（不得让上位机把陈旧值读成测量值）。
  const encoder::WheelState* wheel_state = nullptr;
  bool initialized = true;
  bool fault_latched = false;
};

FeedbackStatusPayload AssembleStatusPayload(const FeedbackInputs& inputs, uint64_t now_us);
// encoder_stats 为 004 的可选输入：nullptr 表示本轮未接入编码器数据源（四个计数写 0）。
struct EncoderStatsView {
  uint32_t samples = 0;
  uint32_t out_of_range = 0;
  uint32_t timestamp_invalid = 0;
  uint32_t hardware_faults = 0;
};
FeedbackDiagPayload AssembleDiagPayload(const mcu_os_lite::TaskSet* tasks, const ProtocolStats& stats,
                                       const CommandManagerStats& command_stats,
                                       const ByteRingStatistics& ring, bool limits_config_valid,
                                       const EncoderStatsView* encoder_stats, uint64_t now_ms);

size_t EncodeStatusFrame(const FeedbackStatusPayload& payload, uint32_t seq, uint8_t* out,
                         size_t out_capacity);
size_t EncodeDiagFrame(const FeedbackDiagPayload& payload, uint32_t seq, uint8_t* out,
                       size_t out_capacity);

bool DecodeStatusPayload(const DecodedFrame& frame, FeedbackStatusPayload& out);
bool DecodeDiagPayload(const DecodedFrame& frame, FeedbackDiagPayload& out);

}  // namespace robotcar01::protocol

#endif  // ROBOTCAR01_APP_PROTOCOL_FEEDBACK_H_
