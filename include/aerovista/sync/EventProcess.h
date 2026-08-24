#pragma once

#include <aerovista/sync/CigiIncludes.h>

#include <atomic>
#include <cstdint>

#include "CigiBaseEventProcessor.h"
#include "CigiCollDetSegDefV4.h"
#include "CigiCollDetSegRespV4.h"
#include "CigiCollDetVolDefV4.h"
#include "CigiCollDetVolRespV4.h"
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

    /// CollDetSegDefV4 捕获（IG 侧）：缓存值（Host 下发碰撞检测段定义）。
    class CollDetSegDefProc : public CigiBaseEventProcessor
    {
    public:
        void OnPacketReceived(CigiBasePacket* packet) override;
        void reset()
        {
            got = false;
            segDef = {};
        }
        bool got = false;
        CigiCollDetSegDefV4 segDef{}; ///< CCL 报文值拷贝（§8.1 通用模式）
    };

    /// CollDetSegRespV4 捕获（Host 侧）：缓存值（IG 回碰撞检测段响应）。
    class CollDetSegRespProc : public CigiBaseEventProcessor
    {
    public:
        void OnPacketReceived(CigiBasePacket* packet) override;
        void reset()
        {
            got = false;
            segResp = {};
        }
        bool got = false;
        CigiCollDetSegRespV4 segResp{}; ///< CCL 报文值拷贝（§8.1 通用模式）
    };

    /// CollDetVolDefV4 捕获（IG 侧）：缓存值（Host 下发碰撞检测体积定义）。
    class CollDetVolDefProc : public CigiBaseEventProcessor
    {
    public:
        void OnPacketReceived(CigiBasePacket* packet) override;
        void reset()
        {
            got = false;
            volDef = {};
        }
        bool got = false;
        CigiCollDetVolDefV4 volDef{}; ///< CCL 报文值拷贝（§8.1 通用模式）
    };

    /// CollDetVolRespV4 捕获（Host 侧）：缓存值（IG 回碰撞检测体积响应）。
    class CollDetVolRespProc : public CigiBaseEventProcessor
    {
    public:
        void OnPacketReceived(CigiBasePacket* packet) override;
        void reset()
        {
            got = false;
            volResp = {};
        }
        bool got = false;
        CigiCollDetVolRespV4 volResp{}; ///< CCL 报文值拷贝（§8.1 通用模式）
    };
} // namespace aerovista::sync
