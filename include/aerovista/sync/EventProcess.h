#pragma once

#include <aerovista/sync/CigiIncludes.h>
#include <aerovista/sync/SyncConfig.h>

#include <atomic>
#include <cstdint>
#include <functional>

#include "CigiBaseEventProcessor.h"
#include "CigiIGCtrlV4.h"

namespace aerovista::sync
{
    /// CIGI 报文处理单元集合（§8.1）。所有报文 processor 统一定义于此文件：
    /// 捕获 → 翻译/投递（订阅模式）；见下方各类型与 PacketCaptureProc。

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
        CigiIGCtrlV4 igCtrl{}; ///< CCL 报文值拷贝（§8.1）
    };

    /// SOF 回显计数（Host 侧）。
    class SofCaptureProc : public CigiBaseEventProcessor
    {
    public:
        void OnPacketReceived(CigiBasePacket* packet) override;
        std::atomic<std::uint32_t> count{0};
    };

    /// sink 订阅能力 mixin（2026-08）：类型化投递回调 + setSink。
    /// 纯 mixin，不继承 CigiBaseEventProcessor（后者由 PacketCaptureProc / 定制 processor 自持）。
    /// 投递**非原始报文类型**（如翻译后语义结构）时用 `Sinkable<语义类型>` 混入并自行翻译（见 EyeCaptureProc）。
    template <typename PacketT>
    class Sinkable
    {
    public:
        /// 注入业务投递回调（值持有；空 = 取消订阅）。主线程解包时同步调用，
        /// 只做轻量翻译/入队/置标志（§8.1），重量业务留帧循环消费。
        /// 生命周期：回调体捕获对象须存活至 sync 会话结束（同业务 processor 约定）。
        void setSink(std::function<void(const PacketT&)> sink) { _sink = std::move(sink); }

    protected:
        std::function<void(const PacketT&)> _sink;
    };

    /// 通用报文捕获（§8.1，纯订阅 2026-08）：按 PacketID 注册到收包端 CCL session，
    /// OnPacketReceived = dynamic_cast + 经 sink 同步投递。CCL 复用单例必须立即处理/拷贝（§8.1）。
    /// 注册按发送源（IgSync/HostSync）与链路（UDP 持续 / TCP 一次性）（cigi梳理.md 链路矩阵）。
    /// 定制：需业务翻译/过滤的报文可派生本类并 override OnPacketReceived（范例 EyeCaptureProc）。
    template <typename PacketT>
    class PacketCaptureProc : public CigiBaseEventProcessor, public Sinkable<PacketT>
    {
    public:
        void OnPacketReceived(CigiBasePacket* packet) override
        {
            auto* typed = dynamic_cast<PacketT*>(packet);
            if (!typed)
                return;
            if (this->_sink)
                this->_sink(*typed);
        }
    };

    /// ownship 眼点捕获（IG 侧）：**定制 processor 范例**（2026-08）——投递翻译后语义类型
    /// `HostEyePose`，故直接继承 `CigiBaseEventProcessor` + `Sinkable<HostEyePose>`（不继承
    /// `PacketCaptureProc<CigiEntityPositionCtrlV4>`——后者 sink 是原始 CCL 类型）。
    /// OnPacketReceived：捕获 ownship 眼点 → 翻译 HostEyePose → 基类 `setSink` 投递。
    class EyeCaptureProc : public CigiBaseEventProcessor, public Sinkable<HostEyePose>
    {
    public:
        void OnPacketReceived(CigiBasePacket* packet) override;
    };
} // namespace aerovista::sync
