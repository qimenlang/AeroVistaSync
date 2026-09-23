#pragma once

#include <cstdint>
#include <functional>
#include <optional>
#include <vector>

class CigiOutgoingMsg;

/// CIGI V4 组包辅助：数据面 Host↔IG，以及握手面 HELLO / UDP_SYNC。
/// Host/IG 业务收发仍走 session（outMsgWithIgCtrl* / drainIncoming）。
namespace aerovista::sync
{
    namespace cigi_wire
    {
        /// TCP HELLO：`CigiIGMsgV4::MsgID`（平台同步设计.md §6.3）。
        constexpr std::uint16_t helloMsgId = 1;

        struct HelloIdentity
        {
            std::uint32_t udpRecvPort = 0;
            int channelId = 0;
        };

        struct EyePose
        {
            double x = 0.0; ///< 纬度°
            double y = 0.0; ///< 经度°
            double z = 0.0; ///< 海拔 米
            double yawDeg = 0.0;
            double pitchDeg = 0.0;
            double rollDeg = 0.0;
            std::uint16_t entityId = 0;
            std::uint16_t parentId = 0;
        };

        /// 通用 CIGI 分帧器：按 PacketSize 切出完整报文字节并回调（不解析、不解包）。
        /// 供命令面 I/O 线程使用；主线程拿完整报文字节喂 CigiIncomingMsg::ProcessIncomingMsg。
        /// `startPacketId`：消息起点。Host→IG / 缺省 = IGCtrl `0x0000`；IG→Host TCP = SOF `0xffff`。
        class CigiFrameAssembler
        {
        public:
            explicit CigiFrameAssembler(std::uint16_t startPacketId = 0)
                : _startPacketId(startPacketId)
            {
            }

            void feed(const unsigned char* data, int n,
                      const std::function<void(const std::vector<unsigned char>&)>& onFrame);

            bool bufferEmpty() const { return _buf.empty(); }

        private:
            std::vector<unsigned char> _buf;
            std::uint16_t _startPacketId = 0;
        };

        /// 因纬度/俯仰超出范围而被丢弃的 LLA 眼点数（lla设计 §5）。
        std::uint64_t eyePoseRejectedByRange();

        /// 把 ownship 眼点（EntityPositionCtrlV4）组装进 omsg（IGCtrl 已由 outMsgWithIgCtrlUdp() 自动前置）。
        /// 恒为 Detach + LLA + ParentID=0（同步层只支持 LLA）。
        /// 调用顺序：`host.outMsgWithIgCtrlUdp()` 拿到已前置 IGCtrl 的 omsg → 本函数追加眼点 → `flushUdp()`。
        /// LLA 越界丢弃逻辑在内（eyePoseRejectedByRange 计数）。eye 为空则只发 IGCtrl（无眼点帧）。
        void appendEye(CigiOutgoingMsg& omsg, const EyePose* eye);

        /// 打包 IG→Host：SOFV4 回显 FrameCntr（生产：`IgSync::sendSofPacket` / UDP_SYNC 探测）。
        bool packSof(std::uint32_t frameCntr, std::vector<unsigned char>& out);

        /// 打包 TCP HELLO：SOF + IGMsg（`MsgID=1`，`Msg`=`udpRecvPort channelId`）。
        bool packHello(std::uint32_t udpRecvPort, int channelId, std::vector<unsigned char>& out);

        /// 解一条 SOF+IGMsg HELLO。MsgID 非 1 或正文不是两个整数则空。
        std::optional<HelloIdentity> parseHello(const unsigned char* data, int n);

        /// 打包 Host→IG：单包 IGCtrl（UDP_SYNC_ACK；FrameCntr 无握手语义）。
        bool packIgCtrl(std::uint32_t frameCntr, std::vector<unsigned char>& out);

        bool isSofPacket(const unsigned char* data, int n);
        bool isIgCtrlPacket(const unsigned char* data, int n);

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
