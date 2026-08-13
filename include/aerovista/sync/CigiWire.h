#pragma once

#include <cstdint>
#include <functional>
#include <optional>
#include <vector>

/// CIGI V4 data-plane pack/unpack for Host↔IG sync.
/// Handshake (HELLO / UDP_SYNC) stays on sync_proto::WireMsg — see SyncProtocol.h.
namespace aerovista::sync
{
    namespace cigi_wire
    {
        /// Wire position semantics from EntityPosition AttachState (lla设计 §5).
        enum class EyeFrame : std::uint8_t
        {
            WORLD_LOCAL = 0, ///< Attach + X/Y/Z off
            LLA = 1          ///< Detach + Lat/Lon/Alt
        };

        struct EyePose
        {
            double x = 0.0; ///< WORLD_LOCAL: X off m; LLA: lat°
            double y = 0.0; ///< WORLD_LOCAL: Y off m; LLA: lon°
            double z = 0.0; ///< WORLD_LOCAL: Z off m; LLA: alt m
            double yawDeg = 0.0;
            double pitchDeg = 0.0;
            double rollDeg = 0.0;
            EyeFrame frame = EyeFrame::WORLD_LOCAL;
            std::uint16_t entityId = 0;
            std::uint16_t parentId = 0;
        };

        struct HostFrame
        {
            std::uint32_t frameCntr = 0;
            std::uint32_t timeStamp = 0;
            bool timeStampValid = false;
            std::optional<EyePose> eye;
        };

        /// Command-plane message（状态同步设计初版.md §2）：msgId 指令/回执码，seq 全局递增，payload 不含 seq。
        struct CommandMsg
        {
            std::uint16_t msgId = 0;
            std::uint16_t seq = 0;
            std::vector<std::uint8_t> payload;
        };

        // 指令码（状态同步设计初版.md §2.2）：Host→IG 命令类型。
        // 枚举值遵循 SCREAMING_SNAKE（.clang-tidy EnumConstantCase=UPPER_CASE / cpp-vsg-style.mdc）。
        enum class Command : std::uint16_t
        {
            LOAD_MODEL = 0x0001,
            PLACE_MODEL = 0x0002,
            MOVE_MODEL = 0x0003
        };

        // 回执基码（编码规则 RECEIVED=0x7000|cmd、RESULT-ACK=0x8000|cmd、RESULT-NACK=0x9000|cmd）：
        // 回执 MsgID = 基码 | 指令码，是线格式计算常量（不属于命令枚举，回执码 = base | cmd）。
        inline constexpr std::uint16_t kReceivedReplyBase = 0x7000;
        inline constexpr std::uint16_t kResultAckBase = 0x8000;
        inline constexpr std::uint16_t kResultNackBase = 0x9000;

        /// Pack CommandMsg into a CIGI V4 IGMsg wire frame（初版 §3.2 / CigiIGMsgV4::Pack）：
        /// [PacketSize(2,LE)][PacketID=0x0ff0(2,LE)][MsgID(2,LE)][reserved(2)][Msg=seq(2)+payload，8 对齐]。
        bool packCommandMsg(const CommandMsg& msg, std::vector<unsigned char>& out);

        /// Unpack one complete CIGI V4 IGMsg wire frame. False on malformed / incomplete buffer.
        bool unpackCommandMsg(const unsigned char* data, int n, CommandMsg& out);

        /// TCP 流分帧器（初版 §3.2）：任意分块喂入，按 PacketSize 切出完整 IGMsg 并回调。粘包/拆包均覆盖。
        class CommandFrameAssembler
        {
        public:
            /// 喂入一段 recv 字节；对每条切出的完整报文调用 onMsg。
            void feed(const unsigned char* data, int n, const std::function<void(const CommandMsg&)>& onMsg);

            bool bufferEmpty() const { return _buf.empty(); }

        private:
            std::vector<unsigned char> _buf;
        };

        /// True if buffer starts with sync_proto AVSY magic (handshake plane).
        bool isAvsyMagic(const unsigned char* data, int n);

        /// Count of LLA eyes dropped for lat/pitch out of range (lla设计 §5).
        std::uint64_t eyePoseRejectedByRange();

        /// Pack Host→IG: IGCtrlV4 [+ EntityPositionCtrlV4 when eye != nullptr].
        /// WorldLocal → Attach+XYZ ParentID=1; Lla → Detach+LLA ParentID=0.
        bool packHostFrame(std::uint32_t frameCntr, double simTimeMs, const EyePose* eye,
                           std::vector<unsigned char>& out);

        /// Pack IG→Host: SOFV4 with FrameCntr echo.
        bool packSof(std::uint32_t frameCntr, std::vector<unsigned char>& out);

        /// Unpack Host→IG datagram. Requires IGCtrl; eye optional.
        bool unpackHostFrame(const unsigned char* data, int n, HostFrame& out);

        /// Unpack IG→Host SOF datagram.
        bool unpackSof(const unsigned char* data, int n, std::uint32_t& frameCntrOut);

        /// simTimeMs → CIGI TimeStamp (10 µs steps).
        /// 自然回绕：超出 uint32 上限后取模（时钟同步方案.md §3 决策——第一版直接跨 12h 自然回绕，
        /// IG 侧相位展开平滑跨过回绕点；不做饱和，否则跨 12h 时间戳停住）。
        inline std::uint32_t simTimeMsToTimeStamp(double simTimeMs)
        {
            if (simTimeMs <= 0.0)
                return 0;
            const double ticks = simTimeMs * 100.0; // ms → 10 µs
            return static_cast<std::uint32_t>(ticks);
        }
    } // namespace cigi_wire
} // namespace aerovista::sync
