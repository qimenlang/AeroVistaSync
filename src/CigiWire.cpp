#include <aerovista/sync/CigiIncludes.h>

#include <aerovista/sync/CigiWire.h>

#include "CigiBaseEntityPositionCtrl.h"
#include "CigiBaseIGCtrl.h"
#include "CigiBaseIGMsg.h"
#include "CigiBaseSOF.h"
#include "CigiEntityPositionCtrlV4.h"
#include "CigiHostSession.h"
#include "CigiIGCtrlV4.h"
#include "CigiIGMsgV4.h"
#include "CigiIGSession.h"
#include "CigiOutgoingMsg.h"
#include "CigiSOFV4.h"

#include <cmath>
#include <mutex>
#include <optional>
#include <sstream>
#include <string>

namespace aerovista::sync
{
    namespace cigi_wire
    {
        namespace
        {
            // CCL 非线程安全；packSof / packHello / packIgCtrl 共用内部 session，用这把锁串行化。
            // appendEye 追加到调用方已有的 omsg，不经过本锁。
            std::mutex gCigiMutex;
            std::uint64_t gEyePoseRejectedByRange = 0;

            /// 归一化经度到 (-180, 180]（lla设计 §5）。
            double normalizeLonDeg(double lon)
            {
                double x = std::fmod(lon, 360.0);
                if (x <= -180.0)
                    x += 360.0;
                if (x > 180.0)
                    x -= 360.0;
                return x;
            }

            bool llaEyeInRange(double lat, double /*lon*/, double pitchDeg)
            {
                return lat >= -90.0 && lat <= 90.0 && pitchDeg >= -90.0 && pitchDeg <= 90.0;
            }

            constexpr int kCigiBufCount = 1;
            constexpr int kCigiBufLen = 4096;

            /// packSof / packHello 用的一次性 CCL 会话（IG 出站：SOF 开头）。
            struct IgPackRuntime
            {
                IgPackRuntime() : ig(kCigiBufCount, kCigiBufLen, kCigiBufCount, kCigiBufLen)
                {
                    ig.SetCigiVersion(4, 0);
                    ig.SetSynchronous(false);
                }

                CigiIGSession ig;
            };

            /// packIgCtrl 用的一次性 CCL 会话（Host 出站：IGCtrl 开头）。
            struct HostPackRuntime
            {
                HostPackRuntime() : host(kCigiBufCount, kCigiBufLen, kCigiBufCount, kCigiBufLen)
                {
                    host.SetCigiVersion(4, 0);
                    host.SetSynchronous(false);
                }

                CigiHostSession host;
            };

            IgPackRuntime& igRuntime()
            {
                static IgPackRuntime rt;
                return rt;
            }

            HostPackRuntime& hostRuntime()
            {
                static HostPackRuntime rt;
                return rt;
            }

            bool packageCurrent(CigiOutgoingMsg& omsg, std::vector<unsigned char>& out)
            {
                Cigi_uint8* buf = nullptr;
                int len = 0;
                if (omsg.PackageMsg(&buf, len) != CIGI_SUCCESS || buf == nullptr || len <= 0)
                {
                    omsg.FreeMsg();
                    return false;
                }
                out.assign(buf, buf + len);
                omsg.FreeMsg();
                return !out.empty();
            }

            std::uint16_t packetIdAt(const unsigned char* data, int n)
            {
                if (data == nullptr || n < 4)
                    return 0xFFFF;
                return static_cast<std::uint16_t>(data[2] | (data[3] << 8));
            }
        } // namespace

        std::uint64_t eyePoseRejectedByRange()
        {
            std::lock_guard lock(gCigiMutex);
            return gEyePoseRejectedByRange;
        }

        void appendEye(CigiOutgoingMsg& omsg, const EyePose* eye)
        {
            if (!eye)
                return;

            CigiEntityPositionCtrlV4 ent{};
            ent.SetEntityID(0);
            ent.SetYaw(static_cast<float>(eye->yawDeg), false);
            ent.SetPitch(static_cast<float>(eye->pitchDeg), false);
            ent.SetRoll(static_cast<float>(eye->rollDeg), false);

            const double lon = normalizeLonDeg(eye->y);
            if (!llaEyeInRange(eye->x, lon, eye->pitchDeg))
            {
                ++gEyePoseRejectedByRange;
                return; // IGCtrl 仍照发（lla设计 §5）
            }
            // 同步层只支持 LLA：Detach + LLA，ParentID 必须为 0（lla设计 §5）。
            ent.SetParentID(0);
            ent.SetAttachState(CigiBaseEntityPositionCtrl::Detach);
            ent.SetLat(eye->x, false);
            ent.SetLon(lon, false);
            ent.SetAlt(eye->z, false);
            omsg << ent;
        }

        bool packSof(std::uint32_t frameCntr, std::vector<unsigned char>& out)
        {
            out.clear();
            std::lock_guard lock(gCigiMutex);
            CigiSOFV4 sof;
            sof.SetFrameCntr(frameCntr);
            CigiOutgoingMsg& omsg = igRuntime().ig.GetOutgoingMsgMgr();
            omsg.BeginMsg();
            omsg << sof;
            return packageCurrent(omsg, out);
        }

        bool packHello(std::uint32_t udpRecvPort, int channelId, std::vector<unsigned char>& out)
        {
            out.clear();
            const std::string body = std::to_string(udpRecvPort) + " " + std::to_string(channelId);
            std::lock_guard lock(gCigiMutex);
            CigiSOFV4 sof;
            sof.SetFrameCntr(0);
            CigiIGMsgV4 igMsg;
            igMsg.SetMsgID(helloMsgId);
            igMsg.SetMsg(body.c_str());
            CigiOutgoingMsg& omsg = igRuntime().ig.GetOutgoingMsgMgr();
            omsg.BeginMsg();
            omsg << sof;
            omsg << igMsg;
            return packageCurrent(omsg, out);
        }

        std::optional<HelloIdentity> parseHello(const unsigned char* data, int n)
        {
            if (!isSofPacket(data, n))
                return std::nullopt;
            const std::uint16_t sofSize =
                static_cast<std::uint16_t>(data[0] | (data[1] << 8));
            if (sofSize < 8 || n < static_cast<int>(sofSize) + 8)
                return std::nullopt;
            const unsigned char* igMsgBytes = data + sofSize;
            const int igMsgLen = n - sofSize;
            if (packetIdAt(igMsgBytes, igMsgLen) != CIGI_IG_MSG_PACKET_ID_V4)
                return std::nullopt;

            CigiIGMsgV4 igMsg;
            if (igMsg.Unpack(const_cast<Cigi_uint8*>(igMsgBytes), false, nullptr) <
                CigiIGMsgV4::PacketHeaderSize)
                return std::nullopt;
            if (igMsg.GetMsgID() != helloMsgId)
                return std::nullopt;

            HelloIdentity identity;
            std::istringstream in(reinterpret_cast<const char*>(igMsg.GetMsg()));
            if (!(in >> identity.udpRecvPort >> identity.channelId))
                return std::nullopt;
            return identity;
        }

        bool packIgCtrl(std::uint32_t frameCntr, std::vector<unsigned char>& out)
        {
            out.clear();
            std::lock_guard lock(gCigiMutex);
            CigiIGCtrlV4 igCtrl;
            igCtrl.SetFrameCntr(frameCntr);
            igCtrl.SetTimeStampValid(false);
            CigiOutgoingMsg& omsg = hostRuntime().host.GetOutgoingMsgMgr();
            omsg.BeginMsg();
            omsg << igCtrl;
            return packageCurrent(omsg, out);
        }

        bool isSofPacket(const unsigned char* data, int n)
        {
            return data != nullptr && n >= 4 && packetIdAt(data, n) == CIGI_SOF_PACKET_ID_V4;
        }

        bool isIgCtrlPacket(const unsigned char* data, int n)
        {
            return data != nullptr && n >= 4 && packetIdAt(data, n) == CIGI_IG_CTRL_PACKET_ID_V4;
        }

        namespace
        {
            // 从 offset 起，累积一条消息（首包 + 后续非 IGCtrl 包）的字节数。
            // 返回完整消息长度；若后续包的 body 未到齐则返回 nullopt（等更多数据，避免截断消息）。
            // 「读不到下一个包头」时乐观认为消息到此结束——命令面一条 flush 即一条消息，
            // 且消息小、本地回环下极少恰好拆在包边界；「后续包 body 不完整」才是必须等待的明确场景。
            std::optional<std::size_t> messageLength(const std::vector<unsigned char>& buf,
                                                     std::size_t offset, std::uint16_t firstSize,
                                                     std::uint16_t startPacketId)
            {
                std::size_t msgLen = firstSize;
                for (;;)
                {
                    if (buf.size() - offset < msgLen + 4)
                        return msgLen; // 读不到下一个包头：乐观认为消息到此结束
                    const std::uint16_t nextId = static_cast<std::uint16_t>(
                        buf[offset + msgLen + 2] | (buf[offset + msgLen + 3] << 8));
                    if (nextId == startPacketId)
                        return msgLen; // 下一个消息起点（IGCtrl 或 SOF）
                    const std::uint16_t nextSize = static_cast<std::uint16_t>(
                        buf[offset + msgLen] | (buf[offset + msgLen + 1] << 8));
                    if (nextSize < 8)
                        return msgLen; // 畸形大小：乐观切出（保留原行为）
                    if (buf.size() - offset < msgLen + nextSize)
                        return std::nullopt; // 后续包 body 不完整：等更多数据
                    msgLen += nextSize;
                }
            }
        } // namespace

        void CigiFrameAssembler::feed(
            const unsigned char* data, int n,
            const std::function<void(const std::vector<unsigned char>&)>& onFrame)
        {
            if (data != nullptr && n > 0)
                _buf.insert(_buf.end(), data, data + n);

            // 消息级分帧：一条消息 = startPacketId 开头 + 后续非起点包。
            // Host→IG 起点 IGCtrl；IG→Host TCP 起点 SOF。必须按完整消息切，而非单包。
            std::size_t offset = 0;
            for (;;)
            {
                if (_buf.size() - offset < 4)
                    break; // 头不够（PacketSize 2B + PacketID 2B），等更多数据
                const std::uint16_t packetSize =
                    static_cast<std::uint16_t>(_buf[offset] | (_buf[offset + 1] << 8)); // 小端
                if (packetSize < 8 || _buf.size() - offset < packetSize)
                    break; // 报文不完整（拆包）

                const auto maybeLen = messageLength(_buf, offset, packetSize, _startPacketId);
                if (!maybeLen)
                    break; // 后续包 body 不完整：等更多数据

                if (onFrame)
                    onFrame(std::vector<unsigned char>(_buf.begin() + offset,
                                                       _buf.begin() + offset + *maybeLen));
                offset += *maybeLen;
            }
            if (offset > 0)
                _buf.erase(_buf.begin(), _buf.begin() + static_cast<std::ptrdiff_t>(offset));
        }
    } // namespace cigi_wire
} // namespace aerovista::sync
