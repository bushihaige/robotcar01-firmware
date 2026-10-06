// 会话重置唯一入口（Iteration 003，评审 H9 / D-003-?）
//
// USB 断连或重新枚举时，**唯一允许**的重置动作：
//   ByteRing::Reset()（写者必须已停止）→ StreamDecoder::Reset() → CommandManager::ResetSession()
// 分别调用三者是错误用法：漏掉任一项都会让残留字节被按新会话接受。
// 协议的 ProtocolStats 属于"启动以来累计"诊断量，不随会话重置归零（解码器内部实现细节），
// 会话级计数由 CommandManagerStats.session_resets 记录。

#ifndef ROBOTCAR01_APP_PROTOCOL_SESSION_H_
#define ROBOTCAR01_APP_PROTOCOL_SESSION_H_

#include "app/protocol/byte_ring.h"
#include "app/protocol/command_manager.h"
#include "app/protocol/decoder.h"

namespace robotcar01::protocol {

// 前置条件：ByteRing 的写者已停止（重新枚举已发生），本函数由主循环侧调用。
void ResetCommandSession(ByteRing& ring, StreamDecoder& decoder, CommandManager& manager);

}  // namespace robotcar01::protocol

#endif  // ROBOTCAR01_APP_PROTOCOL_SESSION_H_
