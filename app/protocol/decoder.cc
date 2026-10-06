#include "app/protocol/decoder.h"

#include "app/protocol/crc16.h"

namespace robotcar01::protocol {

namespace {
constexpr uint8_t kMagicBytes[4] = {kMagicByte0, kMagicByte1, kMagicByte2, kMagicByte3};
}  // namespace

void StreamDecoder::Init() {
  stats_ = ProtocolStats{};
  Reset();
}

void StreamDecoder::Reset() {
  state_ = State::kSearchMagic;
  magic_index_ = 0;
  header_index_ = 0;
  payload_index_ = 0;
  crc_index_ = 0;
  frame_len_ = 0;
  frame_ = DecodedFrame{};
}

void StreamDecoder::Reject(DecodeStatus status) {
  ++stats_.frames_rejected;
  switch (status) {
    case DecodeStatus::kRejectedMagic:
      ++stats_.bad_magic;
      break;
    case DecodeStatus::kRejectedVersion:
      ++stats_.bad_version;
      break;
    case DecodeStatus::kRejectedFlag:
      ++stats_.bad_flag;
      break;
    case DecodeStatus::kRejectedType:
      ++stats_.bad_type;
      break;
    case DecodeStatus::kRejectedLength:
      ++stats_.bad_length;
      break;
    case DecodeStatus::kRejectedCrc:
      ++stats_.bad_crc;
      break;
    case DecodeStatus::kRejectedReserved:
      ++stats_.bad_reserved;
      break;
    case DecodeStatus::kNeedMoreData:
    case DecodeStatus::kFrameReady:
      break;  // 非拒绝路径
  }
}

void StreamDecoder::BackToSearch(bool restart_with_current_byte, uint8_t byte) {
  state_ = State::kSearchMagic;
  magic_index_ = 0;
  header_index_ = 0;
  payload_index_ = 0;
  crc_index_ = 0;
  if (restart_with_current_byte && byte == kMagicBytes[0]) {
    raw_[0] = byte;
    magic_index_ = 1;
    state_ = State::kCheckMagic;
  }
}

DecodeStatus StreamDecoder::ConsumeByte(uint8_t byte) {
  switch (state_) {
    case State::kSearchMagic:
      if (byte == kMagicBytes[0]) {
        raw_[0] = byte;
        magic_index_ = 1;
        state_ = State::kCheckMagic;
      }
      return DecodeStatus::kNeedMoreData;

    case State::kCheckMagic:
      if (byte == kMagicBytes[magic_index_]) {
        raw_[magic_index_] = byte;
        ++magic_index_;
        if (magic_index_ == 4u) {
          header_index_ = 4u;
          state_ = State::kReadHeader;
        }
        return DecodeStatus::kNeedMoreData;
      }
      // 重同步（D-003-2）：当前字节重当候选首字节，不吞字节
      Reject(DecodeStatus::kRejectedMagic);
      BackToSearch(true, byte);
      return DecodeStatus::kRejectedMagic;

    case State::kReadHeader: {
      raw_[header_index_] = byte;
      ++header_index_;
      if (header_index_ < kFrameHeaderBytes) {
        return DecodeStatus::kNeedMoreData;
      }
      const FrameHeader header{raw_[4], raw_[5], raw_[6], raw_[7], GetLe32(raw_ + 8)};
      DecodeStatus failure = DecodeStatus::kNeedMoreData;
      if (header.version != kProtocolVersion) {
        failure = DecodeStatus::kRejectedVersion;
      } else if (header.flags != 0u) {
        failure = DecodeStatus::kRejectedFlag;
      } else if (!IsKnownMessageType(header.msg_type)) {
        failure = DecodeStatus::kRejectedType;
      } else if (static_cast<size_t>(header.payload_len) > kMaxPayloadBytes ||
                 static_cast<int>(header.payload_len) != RequiredPayloadBytes(header.msg_type)) {
        failure = DecodeStatus::kRejectedLength;
      }
      if (failure != DecodeStatus::kNeedMoreData) {
        Reject(failure);
        BackToSearch(false, byte);
        return failure;
      }
      frame_ = DecodedFrame{};
      frame_.header = header;
      payload_index_ = 0;
      crc_index_ = 0;
      frame_len_ = static_cast<uint16_t>(kFrameHeaderBytes + header.payload_len + kCrcBytes);
      state_ = (header.payload_len == 0u) ? State::kReadCrc : State::kReadPayload;
      return DecodeStatus::kNeedMoreData;
    }

    case State::kReadPayload: {
      raw_[kFrameHeaderBytes + payload_index_] = byte;
      if (payload_index_ < kMaxPayloadBytes) {
        frame_.payload[payload_index_] = byte;
      }
      ++payload_index_;
      if (payload_index_ < frame_.header.payload_len) {
        return DecodeStatus::kNeedMoreData;
      }
      // 保留字段校验（线格式的一部分）：运动帧 payload[9] 必须为 0
      if (frame_.header.msg_type == static_cast<uint8_t>(MessageType::kMotionCommand) &&
          frame_.payload[9] != 0u) {
        Reject(DecodeStatus::kRejectedReserved);
        BackToSearch(false, byte);
        return DecodeStatus::kRejectedReserved;
      }
      state_ = State::kReadCrc;
      return DecodeStatus::kNeedMoreData;
    }

    case State::kReadCrc: {
      raw_[kFrameHeaderBytes + frame_.header.payload_len + crc_index_] = byte;
      ++crc_index_;
      if (crc_index_ < kCrcBytes) {
        return DecodeStatus::kNeedMoreData;
      }
      const DecodeStatus result = FinishFrame();
      BackToSearch(false, byte);
      return result;
    }
  }
  return DecodeStatus::kNeedMoreData;  // 不可达（switch 覆盖全部枚举值）
}

DecodeStatus StreamDecoder::FinishFrame() {
  const uint16_t crc_received = GetLe16(raw_ + kFrameHeaderBytes + frame_.header.payload_len);
  const uint16_t crc_calculated =
      Crc16CcittFalse(raw_, static_cast<size_t>(kFrameHeaderBytes + frame_.header.payload_len));
  if (crc_received != crc_calculated) {
    Reject(DecodeStatus::kRejectedCrc);
    frame_.valid = false;
    return DecodeStatus::kRejectedCrc;
  }
  frame_.valid = true;
  ++stats_.frames_accepted;
  return DecodeStatus::kFrameReady;
}

DecodeStatus StreamDecoder::Consume(const uint8_t* data, size_t size, size_t* out_consumed,
                                   const CommandParseConfig& budget) {
  if (out_consumed != nullptr) {
    *out_consumed = 0;
  }
  if (data == nullptr || size == 0) {
    return DecodeStatus::kNeedMoreData;
  }
  const size_t limit = (size < budget.max_bytes_per_cycle) ? size : budget.max_bytes_per_cycle;
  for (size_t i = 0; i < limit; ++i) {
    const DecodeStatus status = ConsumeByte(data[i]);
    if (out_consumed != nullptr) {
      *out_consumed = i + 1;
    }
    if (status == DecodeStatus::kFrameReady) {
      return status;  // 成帧即返回，剩余字节留待下一次调用（A2.1）
    }
  }
  return DecodeStatus::kNeedMoreData;
}

}  // namespace robotcar01::protocol
