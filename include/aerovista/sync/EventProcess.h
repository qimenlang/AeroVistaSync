#pragma once

#include <aerovista/sync/CigiIncludes.h>
#include <aerovista/sync/SyncConfig.h>

#include <atomic>
#include <cstdint>
#include <functional>
#include <vector>

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

    /// sink 订阅能力 mixin（2026-08 / 多播 2026-09）：类型化投递回调列表 + addCallback。
    /// 纯 mixin，不继承 CigiBaseEventProcessor（后者由 PacketCaptureProc 自持）。
    /// 与 CCL `EventList`（同 PacketID 多 processor）对齐：同一订阅主题支持多个消费者回调
    /// （多播），`addCallback` 追加；**不提供取消**（订阅在初始化时一次性注册，回调体
    /// 捕获对象须存活至 sync 会话结束——同业务 processor 约定）。投递**非原始报文类型**
    /// （如翻译后语义结构）时用 `Sinkable<语义类型>` 混入并自行翻译。
    template <typename PacketT>
    class Sinkable
    {
    public:
        /// 追加业务投递回调（值持有）。主线程解包时同步调用，
        /// 只做轻量翻译/入队/置标志（§8.1），重量业务留帧循环消费。
        void addCallback(std::function<void(const PacketT&)> callback)
        {
            _sinks.push_back(std::move(callback));
        }

    protected:
        /// 向全部已注册回调投递一份报文（多播，语义对齐 CCL EventList 多 processor）。
        void notify(const PacketT& value)
        {
            for (auto& sink : _sinks)
                sink(value);
        }

        std::vector<std::function<void(const PacketT&)>> _sinks;
    };

    /// 通用报文捕获（§8.1，纯订阅 2026-08）：按 PacketID 注册到收包端 CCL session，
    /// OnPacketReceived = dynamic_cast + 经回调列表同步多播投递。CCL 复用单例必须立即处理/拷贝（§8.1）。
    /// 注册按发送源（IgSync/HostSync）与链路（UDP 持续 / TCP 一次性）（cigi梳理.md 链路矩阵）。
    /// 业务翻译/过滤统一在订阅回调内完成；同一报文类型跨链路多 processor 时各链路的
    /// `PacketCaptureProc<PacketT>` 均向同一回调多播（addCallback 按类型定位，§8.1）。
    template <typename PacketT>
    class PacketCaptureProc : public CigiBaseEventProcessor, public Sinkable<PacketT>
    {
    public:
        void OnPacketReceived(CigiBasePacket* packet) override
        {
            auto* typed = dynamic_cast<PacketT*>(packet);
            if (!typed)
                return;
            this->notify(*typed);
        }
    };
} // namespace aerovista::sync
