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

// 状态帧载荷（定长 64 B：52 B 字段 + 12 B 保留，保留必须写 0）。
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
  uint32_t reserved[3] = {};
};

// 诊断帧任务槽：mcu_os_lite::TaskSet 的紧凑投影（不拥有 name 字符串）。
struct FeedbackDiagTaskEntry {
  const char* name = "";
  uint32_t run_count = 0;
  uint32_t overrun_count = 0;
  uint32_t max_elapsed_us = 0;
};

// 诊断帧载荷（定长 64 B：28 B 统计 + 8 B 任务元信息 + 4 B 保留 + 8×4 B 任务槽）。
struct FeedbackDiagPayload {
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
  uint32_t seq_rejected = 0;
  uint32_t ring_overflow_bytes = 0;
  uint32_t ring_high_water = 0;
  uint32_t wrong_direction_frames = 0;
  uint32_t task_count = 0;
  uint32_t dropped_task_count = 0;
  uint32_t reserved = 0;
  uint32_t flag_bits = 0;  // bit0 = truncated
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
  bool initialized = true;
  bool fault_latched = false;
};

FeedbackStatusPayload AssembleStatusPayload(const FeedbackInputs& inputs, uint64_t now_us);
FeedbackDiagPayload AssembleDiagPayload(const mcu_os_lite::TaskSet* tasks, const ProtocolStats& stats,
                                       const ByteRingStatistics& ring, uint64_t now_ms);

size_t EncodeStatusFrame(const FeedbackStatusPayload& payload, uint32_t seq, uint8_t* out,
                         size_t out_capacity);
size_t EncodeDiagFrame(const FeedbackDiagPayload& payload, uint32_t seq, uint8_t* out,
                       size_t out_capacity);

bool DecodeStatusPayload(const DecodedFrame& frame, FeedbackStatusPayload& out);
bool DecodeDiagPayload(const DecodedFrame& frame, FeedbackDiagPayload& out);

}  // namespace robotcar01::protocol

#endif  // ROBOTCAR01_APP_PROTOCOL_FEEDBACK_H_
