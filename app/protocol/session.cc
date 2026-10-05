#include "app/protocol/session.h"

namespace robotcar01::protocol {

void ResetCommandSession(ByteRing& ring, StreamDecoder& decoder, CommandManager& manager) {
  ring.Reset();
  decoder.Reset();
  manager.ResetSession();
}

}  // namespace robotcar01::protocol
