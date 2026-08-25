#pragma once

#include <cstdint>
#include <functional>
#include <optional>
#include <vector>

class CigiOutgoingMsg;

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

        /// 通用 CIGI 分帧器：按 PacketSize 切出完整报文字节并回调（不解析、不解包）。
        /// 供命令面 I/O 线程使用；主线程拿完整报文字节喂 CigiIncomingMsg::ProcessIncomingMsg。
        class CigiFrameAssembler
        {
        public:
            void feed(const unsigned char* data, int n,
                      const std::function<void(const std::vector<unsigned char>&)>& onFrame);

            bool bufferEmpty() const { return _buf.empty(); }

        private:
            std::vector<unsigned char> _buf;
        };

        /// 缓冲区以 sync_proto AVSY 魔数开头（握手面）则返回 true。
        bool isAvsyMagic(const unsigned char* data, int n);

        /// 因纬度/俯仰超出范围而被丢弃的 LLA 眼点数（lla设计 §5）。
        std::uint64_t eyePoseRejectedByRange();

        /// 把 ownship 眼点（EntityPositionCtrlV4）组装进 omsg（IGCtrl 已由 outMsgWithIgCtrlUdp() 自动前置）。
        /// WorldLocal → Attach+XYZ ParentID=1；Lla → Detach+LLA ParentID=0。
        /// 业务侧（矛盾 A + IGCtrl 自动填充）用 host.outMsgWithIgCtrlUdp() 拿到 omsg 后调本函数追加眼点，再 flushUdp()。
        /// LLA 越界丢弃逻辑在内（eyePoseRejectedByRange 计数）。eye 为空则只发 IGCtrl（无眼点帧）。
        void appendEye(CigiOutgoingMsg& omsg, const EyePose* eye);

        /// 打包 Host→IG：IGCtrlV4 [+ 眼点非空时 EntityPositionCtrlV4]（线格式测试锚定用）。
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
