#pragma once

#include <cstdint>
#include <functional>
#include <optional>
#include <vector>

/// CIGI V4 数据面 Host↔IG 同步的 pack/unpack。
/// 握手（HELLO / UDP_SYNC）仍在 sync_proto::WireMsg 上——见 SyncProtocol.h。
namespace aerovista::sync
{
    namespace cigi_wire
    {
        /// 线上位置语义来自 EntityPosition AttachState（lla设计 §5）。
        enum class EyeFrame : std::uint8_t
        {
            WORLD_LOCAL = 0, ///< Attach + X/Y/Z 偏移
            LLA = 1          ///< Detach + 纬度/经度/海拔
        };

        struct EyePose
        {
            double x = 0.0; ///< WORLD_LOCAL: X 偏移 米; LLA: 纬度°
            double y = 0.0; ///< WORLD_LOCAL: Y 偏移 米; LLA: 经度°
            double z = 0.0; ///< WORLD_LOCAL: Z 偏移 米; LLA: 海拔 米
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

        /// 把 CommandMsg 打包成 CIGI V4 IGMsg 线上帧（初版 §3.2 / CigiIGMsgV4::Pack）：
        /// [PacketSize(2,LE)][PacketID=0x0ff0(2,LE)][MsgID(2,LE)][reserved(2)][Msg=seq(2)+payload，8 对齐]。
        bool packCommandMsg(const CommandMsg& msg, std::vector<unsigned char>& out);

        /// 解包一个完整的 CIGI V4 IGMsg 线上帧。畸形 / 不完整缓冲返回 false。
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

        /// 缓冲区以 sync_proto AVSY 魔数开头（握手面）则返回 true。
        bool isAvsyMagic(const unsigned char* data, int n);

        /// 因纬度/俯仰超出范围而被丢弃的 LLA 眼点数（lla设计 §5）。
        std::uint64_t eyePoseRejectedByRange();

        /// 打包 Host→IG：IGCtrlV4 [+ 眼点非空时 EntityPositionCtrlV4]。
        /// WorldLocal → Attach+XYZ ParentID=1；Lla → Detach+LLA ParentID=0。
        bool packHostFrame(std::uint32_t frameCntr, double simTimeMs, const EyePose* eye,
                           std::vector<unsigned char>& out);

        /// 打包 IG→Host：SOFV4 回显 FrameCntr。
        bool packSof(std::uint32_t frameCntr, std::vector<unsigned char>& out);

        /// 解包 Host→IG 数据报。要求有 IGCtrl；眼点可选。
        bool unpackHostFrame(const unsigned char* data, int n, HostFrame& out);

        /// 解包 IG→Host SOF 数据报。
        bool unpackSof(const unsigned char* data, int n, std::uint32_t& frameCntrOut);

        /// simTimeMs → CIGI TimeStamp（10 µs 步进）。
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
