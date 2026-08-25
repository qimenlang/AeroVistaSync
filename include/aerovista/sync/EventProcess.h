#pragma once

#include <aerovista/sync/CigiIncludes.h>

#include <atomic>
#include <cstdint>

#include "CigiBaseEventProcessor.h"
#include "CigiEntityPositionCtrlV4.h"
#include "CigiIGCtrlV4.h"

namespace aerovista::sync
{
    /// CIGI 报文处理单元集合（§8.1 processor 归属设计规则）。
    /// 所有 CIGI 报文的 processor 统一定义于此文件；遵循通用模式：
    /// 捕获 → 值拷贝缓存自身 → IgSync/HostSync 开放 getter 供外部拉取。

    /// IGCtrl 帧节拍/时间戳捕获（IG 侧）：缓存 CigiIGCtrlV4 值。
    class IgCtrlCaptureProc : public CigiBaseEventProcessor
    {
    public:
        void OnPacketReceived(CigiBasePacket* packet) override;
        void reset()
        {
            got = false;
            igCtrl = {};
        }
        bool got = false;
        CigiIGCtrlV4 igCtrl{}; ///< CCL 报文值拷贝（§8.1 通用模式）
    };

    /// ownship 眼点捕获（IG 侧）：缓存 CigiEntityPositionCtrlV4 值。
    class EyeCaptureProc : public CigiBaseEventProcessor
    {
    public:
        void OnPacketReceived(CigiBasePacket* packet) override;
        void reset()
        {
            got = false;
            eye = {};
        }
        bool got = false;
        CigiEntityPositionCtrlV4 eye{}; ///< CCL 报文值拷贝（§8.1 通用模式）
    };

    /// SOF 回显计数（Host 侧）：缓存计数。
    class SofCaptureProc : public CigiBaseEventProcessor
    {
    public:
        void OnPacketReceived(CigiBasePacket* packet) override;
        std::atomic<std::uint32_t> count{0};
    };

    /// CollDetVolDefV4 捕获（IG 侧）：缓存值（Host 下发碰撞检测体积定义）。
    /// 已用通用捕获模板 `PacketCaptureProc<CigiCollDetVolDefV4>` 替代（2026-08），
    /// 取走经 `IgSync::takeReceived<CigiCollDetVolDefV4>()`。CollDetVolRespV4 同理（HostSync 侧）。

    /// 通用捕获 processor 的虚基类（§8.1 基础设施通用模式）：供收包端统一遍历拉取。
    /// 实例见 PacketCaptureProc<PacketT>；IgSync/HostSync 经 takeReceived<PacketT>() 按类型取走。
    class CaptureProcBase
    {
    public:
        virtual ~CaptureProcBase() = default;
        /// 是否已捕获到一次报文（供 getter 判断；下一帧 reset 自然清空）。
        virtual bool has() const = 0;
        /// 取走即清：清捕获标志，返回缓存值（业务侧 getter 用）。
        virtual void take() = 0;
        /// 清缓存与标志（帧维护 / 会话复位调用）。
        virtual void reset() = 0;
    };

    /// 通用报文捕获（§8.1）：按 PacketID 注册到收包端 CCL session，OnPacketReceived 值拷贝缓存整包。
    /// 捕获的是 CCL 复用单例，必须立即值拷贝（§8.1）；变长字段（Text/Msg 等）为 std::string/vector，深拷贝安全。
    /// 按发送源（IgSync/HostSync）与链路（UDP 持续 / TCP 一次性）注册到对应 session（cigi梳理.md 链路频率矩阵）。
    template <typename PacketT>
    class PacketCaptureProc final : public CigiBaseEventProcessor, public CaptureProcBase
    {
    public:
        void OnPacketReceived(CigiBasePacket* packet) override
        {
            auto* typed = dynamic_cast<PacketT*>(packet);
            if (!typed)
                return;
            _captured = *typed;
            _has = true;
        }

        bool has() const override { return _has; }
        void take() override { _has = false; }
        void reset() override
        {
            _has = false;
            _captured = PacketT{};
        }

        /// 最近捕获的报文值（has() 为 true 时有效）。
        const PacketT& captured() const { return _captured; }

    private:
        bool _has = false;
        PacketT _captured{};
    };
} // namespace aerovista::sync
