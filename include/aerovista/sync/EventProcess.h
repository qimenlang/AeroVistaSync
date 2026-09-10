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
    /// CIGI 报文处理单元集合（状态同步设计初版.md §8.1）。
    /// 所有报文 processor 统一定义于此：捕获后经订阅回调投递；翻译/合成在回调内同步完成。

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
        CigiIGCtrlV4 igCtrl{}; ///< CCL 报文值拷贝（状态同步设计初版.md §8.1）
    };

    /// SOF 回显计数（Host 侧）。
    class SofCaptureProc : public CigiBaseEventProcessor
    {
    public:
        void OnPacketReceived(CigiBasePacket* packet) override;
        std::atomic<std::uint32_t> count{0};
    };

    /// 类型化投递回调列表 + addCallback。纯 mixin，不继承 CigiBaseEventProcessor
    /// （后者由 PacketCaptureProc 自持）。与 CCL `EventList` 对齐：同一主题多回调（多播），
    /// `addCallback` 追加；**不提供取消**（初始化时一次性注册，回调体捕获对象须存活至
    /// sync 会话结束）。现行通用捕获投递 CCL 原类型；若需投递翻译后的语义类型，可对
    /// `Sinkable<语义类型>` 自行 `notify`。
    template <typename PacketT>
    class Sinkable
    {
    public:
        /// 追加业务投递回调（值持有）。主线程解包时同步调用，可在回调内做翻译/合成
        /// （如 Engine `CameraDriver::compose`）。解包栈内耗时会卡住收包；模型加载等
        /// 百 ms 级 IO 仍应另行调度。
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

    /// 通用报文捕获（状态同步设计初版.md §8.1）：按 PacketID 注册到收包端 CCL session，
    /// OnPacketReceived = dynamic_cast + 经回调列表同步多播投递。CCL 复用单例必须立即处理/拷贝。
    /// 注册按发送源（IgSync/HostSync）与链路（UDP 持续 / TCP 一次性）（cigi梳理.md 链路矩阵）。
    /// 业务翻译/过滤/合成统一在订阅回调内完成；同一报文类型跨链路多 processor 时各链路的
    /// `PacketCaptureProc<PacketT>` 均向同一回调多播（addCallback 按类型定位）。
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
