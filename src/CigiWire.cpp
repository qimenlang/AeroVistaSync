#include <aerovista/sync/CigiIncludes.h>

#include <aerovista/sync/CigiWire.h>
#include <aerovista/sync/SyncProtocol.h>

#include "CigiBaseEntityPositionCtrl.h"
#include "CigiBaseEventProcessor.h"
#include "CigiBaseIGCtrl.h"
#include "CigiBaseSOF.h"
#include "CigiEntityPositionCtrlV4.h"
#include "CigiHostSession.h"
#include "CigiIGCtrlV4.h"
#include "CigiIGSession.h"
#include "CigiIncomingMsg.h"
#include "CigiOutgoingMsg.h"
#include "CigiSOFV4.h"

#include <cmath>
#include <cstring>
#include <mutex>

namespace aerovista::sync
{
    namespace cigi_wire
    {
        namespace
        {
        // CCL 非线程安全；Host udpLoop 与引擎线程都会触碰它。
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

            class CaptureIgCtrlProc : public CigiBaseEventProcessor
            {
            public:
                void OnPacketReceived(CigiBasePacket* packet) override
                {
                    auto* ig = dynamic_cast<CigiIGCtrlV4*>(packet);
                    if (!ig)
                        return;
                    got = true;
                    frameCntr = ig->GetFrameCntr();
                    timeStamp = ig->GetTimeStamp();
                    timeStampValid = ig->GetTimeStampValid();
                }

                void reset()
                {
                    got = false;
                    frameCntr = 0;
                    timeStamp = 0;
                    timeStampValid = false;
                }

                bool got = false;
                std::uint32_t frameCntr = 0;
                std::uint32_t timeStamp = 0;
                bool timeStampValid = false;
            };

            class CaptureEntityPosProc : public CigiBaseEventProcessor
            {
            public:
                void OnPacketReceived(CigiBasePacket* packet) override
                {
                    auto* ent = dynamic_cast<CigiEntityPositionCtrlV4*>(packet);
                    if (!ent)
                        return;
                    got = true;
                    eye.entityId = ent->GetEntityID();
                    eye.parentId = ent->GetParentID();
                    eye.yawDeg = ent->GetYaw();
                    eye.pitchDeg = ent->GetPitch();
                    eye.rollDeg = ent->GetRoll();
                    if (ent->GetAttachState() == CigiBaseEntityPositionCtrl::Detach)
                    {
                        eye.frame = EyeFrame::LLA;
                        eye.x = ent->GetLat();
                        eye.y = ent->GetLon();
                        eye.z = ent->GetAlt();
                    }
                    else
                    {
                        eye.frame = EyeFrame::WORLD_LOCAL;
                        eye.x = ent->GetXoff();
                        eye.y = ent->GetYoff();
                        eye.z = ent->GetZoff();
                    }
                }

                void reset()
                {
                    got = false;
                    eye = {};
                }

                bool got = false;
                EyePose eye{};
            };

            class CaptureSofProc : public CigiBaseEventProcessor
            {
            public:
                void OnPacketReceived(CigiBasePacket* packet) override
                {
                    auto* sof = dynamic_cast<CigiSOFV4*>(packet);
                    if (!sof)
                        return;
                    got = true;
                    frameCntr = sof->GetFrameCntr();
                }

                void reset()
                {
                    got = false;
                    frameCntr = 0;
                }

                bool got = false;
                std::uint32_t frameCntr = 0;
            };

        /// 一次性 CCL 初始化。每次调用创建 Cigi*Session 会重建完整的
        /// 出/入包处理器表并主导测试运行时间。
        struct CigiRuntime
            {
                CigiRuntime() :
                    host(kCigiBufCount, kCigiBufLen, kCigiBufCount, kCigiBufLen), ig(kCigiBufCount, kCigiBufLen, kCigiBufCount, kCigiBufLen)
                {
                    host.SetCigiVersion(4, 0);
                    host.SetSynchronous(false);
                    ig.SetCigiVersion(4, 0);
                    ig.SetSynchronous(false);

                    // 处理器比会话存活更久；注册一次（push_back）。
                    ig.GetIncomingMsgMgr().RegisterEventProcessor(CIGI_IG_CTRL_PACKET_ID_V4, &igCtrlProc);
                    ig.GetIncomingMsgMgr().RegisterEventProcessor(CIGI_ENTITY_POSITION_CTRL_PACKET_ID_V4,
                                                                  &entityPosProc);
                    host.GetIncomingMsgMgr().RegisterEventProcessor(CIGI_SOF_PACKET_ID_V4, &sofProc);
                }

                CigiHostSession host;
                CigiIGSession ig;
                CaptureIgCtrlProc igCtrlProc;
                CaptureEntityPosProc entityPosProc;
                CaptureSofProc sofProc;
            };

            CigiRuntime& runtime()
            {
                static CigiRuntime rt;
                return rt;
            }
        } // namespace

        bool isAvsyMagic(const unsigned char* data, int n)
        {
            if (data == nullptr || n < 4)
                return false;
            std::uint32_t magic = 0;
            std::memcpy(&magic, data, sizeof(magic));
            return magic == sync_proto::kMagic;
        }

        std::uint64_t eyePoseRejectedByRange()
        {
            std::lock_guard lock(gCigiMutex);
            return gEyePoseRejectedByRange;
        }

        void appendHostFrame(CigiOutgoingMsg& omsg, std::uint32_t frameCntr, double simTimeMs,
                             const EyePose* eye)
        {
            CigiIGCtrlV4 igCtrl;
            igCtrl.SetFrameCntr(frameCntr);
            igCtrl.SetTimeStamp(simTimeMsToTimeStamp(simTimeMs));
            igCtrl.SetTimeStampValid(true);
            omsg << igCtrl;

            if (!eye)
                return;

            CigiEntityPositionCtrlV4 ent{};
            ent.SetEntityID(0);
            ent.SetYaw(static_cast<float>(eye->yawDeg), false);
            ent.SetPitch(static_cast<float>(eye->pitchDeg), false);
            ent.SetRoll(static_cast<float>(eye->rollDeg), false);

            if (eye->frame == EyeFrame::LLA)
            {
                const double lon = normalizeLonDeg(eye->y);
                if (!llaEyeInRange(eye->x, lon, eye->pitchDeg))
                {
                    ++gEyePoseRejectedByRange;
                    return; // IGCtrl 仍照发（lla设计 §5）
                }
                // 椭球：Detach + LLA，ParentID 必须为 0（lla设计 §5）。
                ent.SetParentID(0);
                ent.SetAttachState(CigiBaseEntityPositionCtrl::Detach);
                ent.SetLat(eye->x, false);
                ent.SetLon(lon, false);
                ent.SetAlt(eye->z, false);
            }
            else
            {
                // 本地世界 XYZ：相对合成父节点做 Attach 偏移（lla设计 §5）。
                ent.SetParentID(1);
                ent.SetAttachState(CigiBaseEntityPositionCtrl::Attach);
                ent.SetXoff(eye->x);
                ent.SetYoff(eye->y);
                ent.SetZoff(eye->z);
            }
            omsg << ent;
        }

        bool packHostFrame(std::uint32_t frameCntr, double simTimeMs, const EyePose* eye,
                           std::vector<unsigned char>& out)
        {
            out.clear();
            std::lock_guard lock(gCigiMutex);
            CigiRuntime& rt = runtime();

            CigiOutgoingMsg& omsg = rt.host.GetOutgoingMsgMgr();
            omsg.BeginMsg();
            appendHostFrame(omsg, frameCntr, simTimeMs, eye);

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

        bool packSof(std::uint32_t frameCntr, std::vector<unsigned char>& out)
        {
            out.clear();
            std::lock_guard lock(gCigiMutex);
            CigiRuntime& rt = runtime();

            CigiSOFV4 sof;
            sof.SetFrameCntr(frameCntr);

            CigiOutgoingMsg& omsg = rt.ig.GetOutgoingMsgMgr();
            omsg.BeginMsg();
            omsg << sof;

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

        bool unpackHostFrame(const unsigned char* data, int n, HostFrame& outFrame)
        {
            outFrame = {};
            if (data == nullptr || n <= 0 || isAvsyMagic(data, n))
                return false;

            std::lock_guard lock(gCigiMutex);
            CigiRuntime& rt = runtime();
            rt.igCtrlProc.reset();
            rt.entityPosProc.reset();

            try
            {
                rt.ig.GetIncomingMsgMgr().ProcessIncomingMsg(const_cast<unsigned char*>(data), n);
            }
            catch (...)
            {
                return false;
            }

            if (!rt.igCtrlProc.got)
                return false;

            outFrame.frameCntr = rt.igCtrlProc.frameCntr;
            outFrame.timeStamp = rt.igCtrlProc.timeStamp;
            outFrame.timeStampValid = rt.igCtrlProc.timeStampValid;
            if (rt.entityPosProc.got)
            {
                EyePose eye = rt.entityPosProc.eye;
                // Detach 带非零 ParentID 对我们的眼点槽非法 —— 丢弃眼点（lla设计 §5）。
                if (eye.frame == EyeFrame::LLA && eye.parentId != 0)
                    ; // leave outFrame.eye empty
                else
                    outFrame.eye = eye;
            }
            return true;
        }

        bool unpackSof(const unsigned char* data, int n, std::uint32_t& frameCntrOut)
        {
            frameCntrOut = 0;
            if (data == nullptr || n <= 0 || isAvsyMagic(data, n))
                return false;

            std::lock_guard lock(gCigiMutex);
            CigiRuntime& rt = runtime();
            rt.sofProc.reset();

            try
            {
                rt.host.GetIncomingMsgMgr().ProcessIncomingMsg(const_cast<unsigned char*>(data), n);
            }
            catch (...)
            {
                return false;
            }

            if (!rt.sofProc.got)
                return false;
            frameCntrOut = rt.sofProc.frameCntr;
            return true;
        }

        namespace
        {
            // 从 offset 起，累积一条消息（首包 + 后续非 IGCtrl 包）的字节数。
            // 返回完整消息长度；若后续包的 body 未到齐则返回 nullopt（等更多数据，避免截断消息）。
            // 「读不到下一个包头」时乐观认为消息到此结束——命令面一条 flush 即一条消息，
            // 且消息小、本地回环下极少恰好拆在包边界；「后续包 body 不完整」才是必须等待的明确场景。
            std::optional<std::size_t> messageLength(const std::vector<unsigned char>& buf,
                                                     std::size_t offset, std::uint16_t firstSize)
            {
                std::size_t msgLen = firstSize;
                for (;;)
                {
                    if (buf.size() - offset < msgLen + 4)
                        return msgLen; // 读不到下一个包头：乐观认为消息到此结束
                    const std::uint16_t nextId = static_cast<std::uint16_t>(
                        buf[offset + msgLen + 2] | (buf[offset + msgLen + 3] << 8));
                    if (nextId == 0x0000)
                        return msgLen; // 下一个 IGCtrl = 新消息开始
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

            // 消息级分帧：一条消息 = IGCtrl(PacketID 0x0000) 开头 + 后续非 IGCtrl 包。
            // CCL ProcessIncomingMsg 要求首包为 IGCtrl，故必须按「完整消息」切，而非「单包」。
            std::size_t offset = 0;
            for (;;)
            {
                if (_buf.size() - offset < 4)
                    break; // 头不够（PacketSize 2B + PacketID 2B），等更多数据
                const std::uint16_t packetSize =
                    static_cast<std::uint16_t>(_buf[offset] | (_buf[offset + 1] << 8)); // 小端
                if (packetSize < 8 || _buf.size() - offset < packetSize)
                    break; // 报文不完整（拆包）

                const auto maybeLen = messageLength(_buf, offset, packetSize);
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
