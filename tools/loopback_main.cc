// robotcar01_loopback —— host 字节流回环调试工具（Iteration 003 A7）
//
// 输入（stdin）：每行一个命令帧的十六进制字节串（如 "52433031..."），空行与 '#' 注释忽略。
// 输出（stdout）：反馈帧二进制流（外层帧格式与命令帧一致：magic/version/type/len/seq/CRC）。
// 诊断（stderr）：每行处理后的可解析统计行（key=value），以及汇总行。
//
// 确定性契约（评审 H11）：默认使用**内部虚拟时钟**（每处理一行推进 1 ms），因此输出帧数
// 只由输入行数决定，不依赖墙钟、不随机器负载变化。`--realtime` 才使用真实时钟。
//
// 边界声明：本工具是 host 证据，**不是** USB 真机链路证据（USB CDC 接线属 Iteration 008）。

#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <iostream>
#include <string>
#include <vector>

#include "app/protocol/byte_ring.h"
#include "app/protocol/command_manager.h"
#include "app/protocol/decoder.h"
#include "app/protocol/feedback.h"
#include "app/protocol/feedback_limiter.h"
#include "app/protocol/protocol_constants.h"
#include "app/protocol/session.h"

namespace {

using robotcar01::protocol::AssembleDiagPayload;
using robotcar01::protocol::AssembleStatusPayload;
using robotcar01::protocol::ByteRing;
using robotcar01::protocol::CommandLeaseConfig;
using robotcar01::protocol::CommandLimits;
using robotcar01::protocol::CommandManager;
using robotcar01::protocol::DecodeStatus;
using robotcar01::protocol::EncodeDiagFrame;
using robotcar01::protocol::EncodeStatusFrame;
using robotcar01::protocol::FeedbackInputs;
using robotcar01::protocol::FeedbackLimiter;
using robotcar01::protocol::FeedbackRateConfig;
using robotcar01::protocol::ProtocolStats;
using robotcar01::protocol::StreamDecoder;

struct Options {
  uint32_t frames = 0;  // 0 = 读到 EOF
  uint32_t status_period_ms = 20;
  uint32_t diag_period_ms = 100;
  uint32_t lease_ms = 300;
};

int HexNibble(char c) {
  if (c >= '0' && c <= '9') return c - '0';
  if (c >= 'a' && c <= 'f') return c - 'a' + 10;
  if (c >= 'A' && c <= 'F') return c - 'A' + 10;
  return -1;
}

// 把一行中的十六进制字符解析为字节（忽略空白、'x'、','、':'、'-'、'#' 注释）。
std::vector<uint8_t> ParseHexLine(const std::string& line) {
  std::vector<uint8_t> bytes;
  int high = -1;
  for (size_t i = 0; i < line.size(); ++i) {
    const char c = line[i];
    if (c == '#') break;
    const int nibble = HexNibble(c);
    if (nibble < 0) continue;
    if (high < 0) {
      high = nibble;
    } else {
      bytes.push_back(static_cast<uint8_t>((high << 4) | nibble));
      high = -1;
    }
  }
  return bytes;
}

void WriteFrames(const uint8_t* data, size_t size) {
  if (size != 0) {
    std::fwrite(data, 1, size, stdout);
  }
}

}  // namespace

int main(int argc, char** argv) {
  Options options{};
  bool realtime = false;
  for (int i = 1; i < argc; ++i) {
    const std::string arg = argv[i];
    const bool has_next = (i + 1) < argc;
    if (arg == "--frames" && has_next) {
      options.frames = static_cast<uint32_t>(std::strtoul(argv[++i], nullptr, 10));
    } else if (arg == "--status-period-ms" && has_next) {
      options.status_period_ms = static_cast<uint32_t>(std::strtoul(argv[++i], nullptr, 10));
    } else if (arg == "--diag-period-ms" && has_next) {
      options.diag_period_ms = static_cast<uint32_t>(std::strtoul(argv[++i], nullptr, 10));
    } else if (arg == "--lease-ms" && has_next) {
      options.lease_ms = static_cast<uint32_t>(std::strtoul(argv[++i], nullptr, 10));
    } else if (arg == "--realtime") {
      realtime = true;
    } else if (arg == "--help" || arg == "-h") {
      std::fprintf(stderr,
                   "usage: robotcar01_loopback [--frames N] [--status-period-ms X] "
                   "[--diag-period-ms Y] [--lease-ms Z] [--realtime]\n"
                   "stdin: one hex-encoded command frame per line\n"
                   "stdout: binary feedback frames\n");
      return 0;
    } else {
      std::fprintf(stderr, "unknown argument: %s\n", arg.c_str());
      return 2;
    }
  }

  ByteRing ring;
  StreamDecoder decoder;
  CommandManager manager;
  FeedbackLimiter limiter;
  ProtocolStats stats;

  CommandLeaseConfig lease{};
  lease.default_lease_ms = options.lease_ms;
  CommandLimits limits{};  // 未配置范围上限：只校验有限性（数值校验归 CommandManager）
  decoder.Init();
  manager.Init(lease, limits);
  FeedbackRateConfig rate{};
  rate.status_period_ms = options.status_period_ms;
  rate.diag_period_ms = options.diag_period_ms;
  limiter.Init(rate, 0);
  (void)realtime;  // 保留：真机/交互调试可由调用方切换时钟源（D-003-9 注入点）

  uint64_t now_us = 0;
  uint32_t feedback_seq = 0;
  uint32_t lines = 0;
  uint32_t frames_processed = 0;
  uint8_t feedback_buffer[robotcar01::protocol::kMaxFrameBytes] = {};

  std::string line;
  while (std::getline(std::cin, line)) {
    const std::vector<uint8_t> bytes = ParseHexLine(line);
    if (bytes.empty()) {
      continue;
    }
    ++lines;
    now_us += 1000;  // 虚拟时钟：每行 = 1 ms（确定性契约）

    ring.Write(bytes.data(), bytes.size());
    // 逐字节消费（工具场景：单线程、输入已定；有界预算消费见协议测试）
    uint8_t byte = 0;
    while (ring.ReadByte(byte)) {
      const DecodeStatus status = decoder.ConsumeByte(byte);
      if (status == DecodeStatus::kFrameReady) {
        manager.OnFrame(decoder.last_frame(), now_us);
        decoder.ClearLastFrame();
        stats = decoder.stats();  // 镜像解码器统计
        ++frames_processed;
      }
    }

    // 反馈发送（限频由 FeedbackLimiter 决定；失败只计数，不影响命令路径）
    FeedbackInputs inputs{};
    inputs.command = &manager.snapshot();
    if (limiter.ShouldSendStatus(now_us)) {
      const auto payload = AssembleStatusPayload(inputs, now_us);
      const size_t size = EncodeStatusFrame(payload, feedback_seq++, feedback_buffer,
                                           sizeof(feedback_buffer));
      if (size == 0) {
        limiter.OnSendFailed(true);
      } else {
        WriteFrames(feedback_buffer, size);
      }
    }
    if (limiter.ShouldSendDiag(now_us)) {
      const auto payload = AssembleDiagPayload(nullptr, stats, ring.statistics(), now_us / 1000);
      const size_t size =
          EncodeDiagFrame(payload, feedback_seq++, feedback_buffer, sizeof(feedback_buffer));
      if (size == 0) {
        limiter.OnSendFailed(false);
      } else {
        WriteFrames(feedback_buffer, size);
      }
    }

    std::fprintf(stderr,
                 "line=%u accepted=%u rejected=%u frames=%u bad_magic=%u bad_crc=%u bad_len=%u "
                 "bad_type=%u bad_flag=%u bad_reserved=%u seq_rejected=%u wrong_direction=%u "
                 "cmd_accepted=%u cmd_rejected=%u cmd_stale=%u cmd_suspended=%u "
                 "cmd_ignored_run=%u armed=%u run_requested=%u ring_overflow=%u\n",
                 lines, stats.frames_accepted, stats.frames_rejected, frames_processed,
                 stats.bad_magic, stats.bad_crc, stats.bad_length, stats.bad_type, stats.bad_flag,
                 stats.bad_reserved, manager.stats().seq_rejected, manager.stats().wrong_direction,
                 manager.stats().accepted, manager.stats().rejected, manager.stats().stale_frames,
                 manager.stats().suspended_rejected, manager.stats().run_requests_ignored,
                 manager.snapshot().armed ? 1u : 0u, manager.snapshot().run_requested ? 1u : 0u,
                 ring.statistics().overflow_drop_bytes);
    std::fflush(stdout);
    std::fflush(stderr);

    if (options.frames != 0 && lines >= options.frames) {
      break;
    }
  }

  std::fprintf(stderr,
               "summary lines=%u frames=%u feedback_accepted=%u status_failed=%u diag_failed=%u "
               "exit=0\n",
               lines, frames_processed, stats.frames_accepted, limiter.status_dropped(),
               limiter.diag_dropped());
  std::fflush(stdout);
  return 0;
}
