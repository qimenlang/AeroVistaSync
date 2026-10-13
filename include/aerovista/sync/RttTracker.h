#pragma once

#include <chrono>
#include <cstddef>
#include <cstdint>
#include <deque>
#include <optional>
#include <unordered_map>

namespace aerovista::sync
{
    /// 应用 RTT 丢失时限（多通道同步验收测量设计.md §4.5）。本机样本约 20ms，多等一帧仍记 RTT；满此时限记丢失。
    inline constexpr std::chrono::milliseconds appRttMatchTimeout{50};

    /// 单 peer 的 RTT 配对器。应用 RTT（多通道同步验收测量设计.md §4.5，码 MEAS-rtt）与
    /// 网络 RTT（§4.8，同码 MEAS-rtt）共用同一配对逻辑，只在超时时限上不同：
    /// 应用为 appRttMatchTimeout、网络为 netRttMatchTimeout，由构造参数给定。
    /// 时钟由调用方注入。键是 uint32_t：应用 RTT 为 Host Frame Number，网络 RTT 为探测序号。
    class RttTracker
    {
    public:
        static constexpr std::size_t avgWindow = 60;
        static constexpr std::size_t avgMinSamples = 10;

        explicit RttTracker(std::chrono::milliseconds matchTimeout);

        /// 记下键的发出时刻。已在表中的同号不刷新 t_send。
        void onSent(std::uint32_t key, std::chrono::steady_clock::time_point tSend);
        /// 按回显键配对。先按 tRecv 做超时淘汰，再匹配。
        void onReceived(std::uint32_t key, std::chrono::steady_clock::time_point tRecv);
        /// 把 now - tSend >= matchTimeout 且仍未匹配的键记为丢失（不进 RTT）。
        void expireInFlightAt(std::chrono::steady_clock::time_point now);

        std::optional<std::chrono::microseconds> lastRtt() const;
        /// 最近 avgWindow 次完成里，匹配样本的算术平均；其中匹配数 < avgMinSamples 时为空。
        std::optional<std::chrono::microseconds> avgRtt() const;
        /// 最近 avgWindow 次完成（匹配或超时）中超时占比；完成数 < avgMinSamples 时为空。不含飞行中。
        std::optional<double> lossRate() const;

        std::size_t matchCount() const;
        std::size_t lossCount() const;
        /// 已记下发出时刻、尚未匹配或超时的键数。不进 avgRtt / lossRate。
        std::size_t inFlightCount() const;

    private:
        void recordMatch(std::chrono::microseconds rtt);
        void recordTimeout();
        void pushCompletion(std::optional<std::chrono::microseconds> sample);

        std::chrono::milliseconds _matchTimeout;
        /// 在途：键 → t_send。回显按键取这一时刻；未匹配且未满 matchTimeout 的既不进 RTT，也不算丢失。
        std::unordered_map<std::uint32_t, std::chrono::steady_clock::time_point> _inFlight;
        /// 最近完成：有值 = 匹配 RTT，空 = 超时。与 avgRtt / lossRate 同一时间轴。
        std::deque<std::optional<std::chrono::microseconds>> _completions;
        std::optional<std::chrono::microseconds> _lastRtt;
        std::size_t _matchCount = 0;
        std::size_t _lossCount = 0;
    };
} // namespace aerovista::sync
