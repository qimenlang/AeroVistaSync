#pragma once

#include <chrono>
#include <cstddef>
#include <cstdint>
#include <deque>
#include <optional>
#include <unordered_map>

namespace aerovista::sync
{
    /// 单 peer 的 IGCtrl↔SOF 应用 RTT 配对器（多通道同步验收测量设计.md §4.5）。
    /// 时钟由调用方注入；Host 接线后 t_recv 为主线程 drain 见到 SOF 的时刻。
    class SofRttTracker
    {
    public:
        static constexpr std::chrono::milliseconds matchTimeout{100};
        static constexpr std::size_t avgWindow = 60;
        static constexpr std::size_t avgMinSamples = 10;

        /// 记下 Host Frame Number 的发出时刻。已在表中的同号不刷新 t_send。
        void onIgCtrlSent(std::uint32_t hostFrameNumber, std::chrono::steady_clock::time_point tSend);
        /// 按回显的 Host Frame Number 配对。先按 tRecv 做超时淘汰，再匹配。
        void onSofReceived(std::uint32_t echoedHostFrameNumber, std::chrono::steady_clock::time_point tRecv);
        /// 把 now - tSend >= matchTimeout 且仍未匹配的帧记为丢失（不进 RTT）。
        void expire(std::chrono::steady_clock::time_point now);

        std::optional<std::chrono::microseconds> lastRtt() const;
        /// 最近 avgWindow 次完成里，匹配样本的算术平均；其中匹配数 < avgMinSamples 时为空。
        std::optional<std::chrono::microseconds> avgRtt() const;
        /// 最近 avgWindow 次完成（匹配或超时）中超时占比；完成数 < avgMinSamples 时为空。不含飞行中。
        std::optional<double> lossRate() const;

        std::size_t matchCount() const;
        std::size_t lossCount() const;
        std::size_t pendingCount() const;

    private:
        void recordMatch(std::chrono::microseconds rtt);
        void recordTimeout();
        void pushCompletion(std::optional<std::chrono::microseconds> sample);

        std::unordered_map<std::uint32_t, std::chrono::steady_clock::time_point> _pending;
        /// 最近完成：有值 = 匹配 RTT，空 = 超时。与 avgRtt / lossRate 同一时间轴。
        std::deque<std::optional<std::chrono::microseconds>> _completions;
        std::optional<std::chrono::microseconds> _lastRtt;
        std::size_t _matchCount = 0;
        std::size_t _lossCount = 0;
    };
} // namespace aerovista::sync
