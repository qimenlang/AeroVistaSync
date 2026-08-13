#pragma once

#include <cstdint>

/// Handshake-plane wire types (TCP HELLO / UDP_SYNC).
/// Data-plane IGCtrl / eye / SOF use CIGI V4 — see CigiWire.h.
namespace aerovista::sync
{
    namespace sync_proto
    {
        constexpr uint32_t kMagic = 0x41565359u; // 'AVSY'

        enum class MsgType : uint32_t
        {
            HELLO = 1,
            HELLO_ACK = 2,
            UDP_SYNC = 3,
            UDP_SYNC_ACK = 4,
            // 5 / 6 reserved (former custom IG_CTRL / SOF; data plane is CIGI now)
        };

#pragma pack(push, 1)
        struct WireMsg
        {
            uint32_t magic = kMagic;
            uint32_t type = 0;
            uint32_t udpRecvPort = 0;
        };
#pragma pack(pop)

        static_assert(sizeof(WireMsg) == 12, "WireMsg size");
    } // namespace sync_proto
} // namespace aerovista::sync
