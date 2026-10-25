#include <aerovista/sync/CigiIncludes.h>

#include <catch2/catch_test_macros.hpp>

#include <aerovista/sync/CigiWire.h>

#include "CigiBaseEntityPositionCtrl.h"
#include "CigiEntityPositionCtrlV4.h"
#include "CigiHostSession.h"
#include "CigiIGCtrlV4.h"
#include "CigiSymbolTextDefV4.h"

#include <cstdint>
#include <memory>
#include <string>
#include <vector>

namespace cigi_wire = aerovista::sync::cigi_wire;

namespace
{
    // 用 CCL HostSession 组装一条 CIGI 消息（自动前置 IGCtrl 帧头），返回线上字节。
    // 供 CigiFrameAssembler 分帧单测使用（验证消息级切流：IGCtrl 开头 + 后续包）。
    std::vector<unsigned char> packPoseMessage(std::uint16_t entityId)
    {
        auto session = std::make_unique<CigiHostSession>(1, 4096, 1, 4096);
        auto& omsg = session->GetOutgoingMsgMgr();
        omsg.BeginMsg();
        CigiIGCtrlV4 igCtrl;
        omsg << igCtrl;
        CigiEntityPositionCtrlV4 place;
        place.SetEntityID(entityId);
        place.SetAttachState(CigiBaseEntityPositionCtrl::Detach);
        place.SetLat(31.23);
        place.SetLon(121.47);
        place.SetAlt(500.0);
        omsg << place;
        Cigi_uint8* buf = nullptr;
        int len = 0;
        if (omsg.PackageMsg(&buf, len) != CIGI_SUCCESS || buf == nullptr)
            return {};
        std::vector<unsigned char> out(buf, buf + len);
        omsg.FreeMsg();
        return out;
    }

    // IGCtrl + EntityPositionCtrlV4 + SymbolTextDefV4（一条消息，两个数据包）。
    std::vector<unsigned char> packPosePlusTextMessage(std::uint16_t entityId, const std::string& text)
    {
        auto session = std::make_unique<CigiHostSession>(1, 4096, 1, 4096);
        auto& omsg = session->GetOutgoingMsgMgr();
        omsg.BeginMsg();
        CigiIGCtrlV4 igCtrl;
        omsg << igCtrl;
        CigiEntityPositionCtrlV4 place;
        place.SetEntityID(entityId);
        place.SetAttachState(CigiBaseEntityPositionCtrl::Detach);
        place.SetLat(31.23);
        place.SetLon(121.47);
        place.SetAlt(500.0);
        CigiSymbolTextDefV4 cmd(text.c_str());
        omsg << place << cmd;
        Cigi_uint8* buf = nullptr;
        int len = 0;
        if (omsg.PackageMsg(&buf, len) != CIGI_SUCCESS || buf == nullptr)
            return {};
        std::vector<unsigned char> out(buf, buf + len);
        omsg.FreeMsg();
        return out;
    }
} // namespace

// =============================================================================
// 1. 线格式契约（单元，直接测 CCL，绿，锚定 §4.1）
// =============================================================================

// =============================================================================
// EntityPositionCtrl 打成固定 48 字节（状态同步设计.md §10，码 CMD-posctrl-48b）。
// 包头的长度和包号按小端读出，与定长常量一致。
// =============================================================================
TEST_CASE("CigiEntityPositionCtrlV4 packs to fixed 48B", "[unit][sync][cmd][wire-contract][CMD-posctrl-48b]")
{
    CigiEntityPositionCtrlV4 place;
    place.SetEntityID(7);
    place.SetAttachState(CigiBaseEntityPositionCtrl::Detach);
    place.SetLat(31.23);
    place.SetLon(121.47);
    place.SetAlt(500.0);

    Cigi_uint8 buf[CIGI_ENTITY_POSITION_CTRL_PACKET_SIZE_V4] = {};
    const int size = place.Pack(&place, buf, nullptr);
    REQUIRE(size == CIGI_ENTITY_POSITION_CTRL_PACKET_SIZE_V4);
    // CCL 在 x86 输出小端（CIGI 4 不强制字节序，接收方检测）；按小端解析头字段。
    const int packetSize = buf[0] | (buf[1] << 8);
    const int packetId = buf[2] | (buf[3] << 8);
    REQUIRE(packetSize == CIGI_ENTITY_POSITION_CTRL_PACKET_SIZE_V4);
    REQUIRE(packetId == CIGI_ENTITY_POSITION_CTRL_PACKET_ID_V4);
}

// =============================================================================
// 变长文本包的长度至少为定长头，并且按 8 字节对齐（状态同步设计.md §10，码 CMD-symbol-text）。
// 包头里的 PacketSize、PacketID 与打包返回值一致。不验文本正文。
// =============================================================================
TEST_CASE("CigiSymbolTextDefV4 packs variable-length Text", "[unit][sync][cmd][wire-contract][CMD-symbol-text]")
{
    CigiSymbolTextDefV4 cmd("place 7 121.47 31.23 500");
    Cigi_uint8 buf[128] = {};
    const int size = cmd.Pack(&cmd, buf, nullptr);
    REQUIRE(size >= CIGI_SYMBOL_TEXT_DEFINITION_PACKET_SIZE_V4);
    REQUIRE(size % 8 == 0);
    const int packetSize = buf[0] | (buf[1] << 8);
    const int packetId = buf[2] | (buf[3] << 8);
    REQUIRE(packetSize == size);
    REQUIRE(packetId == CIGI_SYMBOL_TEXT_DEFINITION_PACKET_ID_V4);
}

// =============================================================================
// 4. TCP 分帧器（CigiFrameAssembler）单元测试——§4.2 消息级切流
// =============================================================================

// =============================================================================
// 一次 feed 一条完整消息，分帧器切出一帧并清空缓冲（状态同步设计.md §10，码 FRM-one-message）。
// 输入由 CCL 打出。分帧器只按包头切字节，不调用 CCL。
// =============================================================================
TEST_CASE("CigiFrameAssembler emits one complete message as one frame",
          "[unit][sync][cmd][framing][FRM-one-message]")
{
    cigi_wire::CigiFrameAssembler assembler;
    const auto msg = packPoseMessage(7);
    REQUIRE_FALSE(msg.empty());

    std::vector<std::vector<unsigned char>> frames;
    assembler.feed(msg.data(), static_cast<int>(msg.size()),
                   [&](const std::vector<unsigned char>& f) { frames.push_back(f); });

    REQUIRE(frames.size() == 1);
    REQUIRE(frames[0] == msg);
    REQUIRE(assembler.bufferEmpty());
}

// =============================================================================
// 一次 feed 里两条消息首尾相接，分帧器按消息切成两帧（状态同步设计.md §10，码 FRM-sticky）。
// 顺序与输入一致，切完缓冲为空。
// =============================================================================
TEST_CASE("CigiFrameAssembler splits sticky messages (two messages in one feed)",
          "[unit][sync][cmd][framing][FRM-sticky]")
{
    cigi_wire::CigiFrameAssembler assembler;
    const auto a = packPoseMessage(7);
    const auto b = packPoseMessage(8);
    REQUIRE_FALSE(a.empty());
    REQUIRE_FALSE(b.empty());

    std::vector<unsigned char> two = a;
    two.insert(two.end(), b.begin(), b.end());

    std::vector<std::vector<unsigned char>> frames;
    assembler.feed(two.data(), static_cast<int>(two.size()),
                   [&](const std::vector<unsigned char>& f) { frames.push_back(f); });

    REQUIRE(frames.size() == 2);
    REQUIRE(frames[0] == a);
    REQUIRE(frames[1] == b);
    REQUIRE(assembler.bufferEmpty());
}

// =============================================================================
// 消息被拆成两次 feed 时，半包先留在缓冲，补齐后才切出一帧（状态同步设计.md §10，码 FRM-split-buffer）。
// =============================================================================
TEST_CASE("CigiFrameAssembler buffers a split message across feeds",
          "[unit][sync][cmd][framing][FRM-split-buffer]")
{
    cigi_wire::CigiFrameAssembler assembler;
    const auto msg = packPoseMessage(7);
    REQUIRE_FALSE(msg.empty());
    const std::size_t half = msg.size() / 2;

    std::vector<std::vector<unsigned char>> frames;
    auto onFrame = [&](const std::vector<unsigned char>& f) { frames.push_back(f); };

    assembler.feed(msg.data(), static_cast<int>(half), onFrame);
    REQUIRE(frames.empty());                // 半包：尚未切出
    REQUIRE_FALSE(assembler.bufferEmpty()); // 残留缓冲

    assembler.feed(msg.data() + half, static_cast<int>(msg.size() - half), onFrame);
    REQUIRE(frames.size() == 1); // 补齐后切出一条完整
    REQUIRE(frames[0] == msg);
    REQUIRE(assembler.bufferEmpty());
}

// =============================================================================
// 一条消息里含 IGCtrl、位姿和文本多个包时，仍切成一帧（状态同步设计.md §10，码 FRM-multi-packet）。
// 不按单个 Packet 切开。
// =============================================================================
TEST_CASE("CigiFrameAssembler keeps a multi-packet message as one frame",
          "[unit][sync][cmd][framing][FRM-multi-packet]")
{
    cigi_wire::CigiFrameAssembler assembler;
    const auto msg = packPosePlusTextMessage(7, "reset");
    REQUIRE_FALSE(msg.empty());

    std::vector<std::vector<unsigned char>> frames;
    assembler.feed(msg.data(), static_cast<int>(msg.size()),
                   [&](const std::vector<unsigned char>& f) { frames.push_back(f); });

    // IGCtrl + EntityPositionCtrlV4 + SymbolTextDefV4 = 一条消息，不按单包切。
    REQUIRE(frames.size() == 1);
    REQUIRE(frames[0] == msg);
    REQUIRE(assembler.bufferEmpty());
}
