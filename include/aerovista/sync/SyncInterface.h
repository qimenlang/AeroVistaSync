#pragma once

#include <vector>

namespace aerovista::sync
{
    /// `HostSync` / `IgSync` 的中继切齐面（平台同步设计.md §8）。
    /// 纯虚：不持 socket / CCL session / payload 队列（Host UDP 带 fromIp，IG 带 receivedAtUs；
    /// `CigiHostSession` vs `CigiIGSession`）。`initialize` / `outMsg*` / `drainIncoming` /
    /// `addCallback` / `sendSofForIgCtrl` 仍在子类。不改名 `HostSync` / `IgSync`。
    class SyncInterface
    {
    public:
        virtual ~SyncInterface() = default;

        virtual void shutdown() = 0;
        virtual void flushTcp() = 0;
        virtual void flushUdp() = 0;
        /// 已切齐消息原样发送：Host 扇出全体 ready IG；IG 对已连 Host `sendAll`。不加 IGCtrl/SOF。
        virtual void sendTcpMessage(const std::vector<unsigned char>& message) = 0;
        /// 已切齐 UDP 数据报原样 `sendto`：Host 扇出全体 ready IG；IG 发往 `HostTarget::udpPortRecv`。不加帧头。
        virtual void sendUdpMessage(const std::vector<unsigned char>& message) = 0;
        /// 取走已切齐 TCP 消息字节，不解包。与 `drainIncoming` 互斥消费同一 TCP 队列。
        virtual std::vector<std::vector<unsigned char>> takeIncomingTcp() = 0;
        /// 取走 UDP 数据报字节，不解包。与 `drainIncoming` 互斥消费同一 UDP 队列。
        /// Host 侧丢弃 `fromIp`/`fromPort`（无法再做 SOF RTT 配对）。中继不得用 viewhost Host
        /// 的本接口把真实 IG SOF 转给平台。
        virtual std::vector<std::vector<unsigned char>> takeIncomingUdp() = 0;
    };
} // namespace aerovista::sync
