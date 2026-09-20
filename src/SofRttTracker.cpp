#include <aerovista/sync/SofRttTracker.h>

namespace aerovista::sync
{
    void SofRttTracker::pushCompletion(std::optional<Duration> sample)
    {
        _completions.push_back(sample);
        if (_completions.size() > avgWindow)
            _completions.pop_front();
    }

    void SofRttTracker::recordMatch(Duration rtt)
    {
        _lastRtt = rtt;
        ++_matchCount;
        pushCompletion(rtt);
    }

    void SofRttTracker::recordTimeout()
    {
        ++_lossCount;
        pushCompletion(std::nullopt);
    }

    void SofRttTracker::expire(TimePoint now)
    {
        for (auto it = _pending.begin(); it != _pending.end();)
        {
            if (now - it->second >= matchTimeout)
            {
                recordTimeout();
                it = _pending.erase(it);
            }
            else
            {
                ++it;
            }
        }
    }

    void SofRttTracker::onIgCtrlSent(std::uint32_t hostFrameNumber, TimePoint tSend)
    {
        expire(tSend);
        if (_pending.find(hostFrameNumber) != _pending.end())
            return;
        _pending.emplace(hostFrameNumber, tSend);
    }

    void SofRttTracker::onSofReceived(std::uint32_t echoedHostFrameNumber, TimePoint tRecv)
    {
        expire(tRecv);
        const auto it = _pending.find(echoedHostFrameNumber);
        if (it == _pending.end())
            return;
        const Duration rtt = std::chrono::duration_cast<Duration>(tRecv - it->second);
        _pending.erase(it);
        recordMatch(rtt);
    }

    std::optional<SofRttTracker::Duration> SofRttTracker::lastRtt() const
    {
        return _lastRtt;
    }

    std::optional<SofRttTracker::Duration> SofRttTracker::avgRtt() const
    {
        std::size_t matches = 0;
        std::int64_t sumUs = 0;
        for (const auto& sample : _completions)
        {
            if (!sample)
                continue;
            ++matches;
            sumUs += sample->count();
        }
        if (matches < avgMinSamples)
            return std::nullopt;
        return Duration{sumUs / static_cast<std::int64_t>(matches)};
    }

    std::optional<double> SofRttTracker::lossRate() const
    {
        if (_completions.size() < avgMinSamples)
            return std::nullopt;

        std::size_t timeouts = 0;
        for (const auto& sample : _completions)
        {
            if (!sample)
                ++timeouts;
        }
        return static_cast<double>(timeouts) / static_cast<double>(_completions.size());
    }

    std::size_t SofRttTracker::matchCount() const
    {
        return _matchCount;
    }

    std::size_t SofRttTracker::lossCount() const
    {
        return _lossCount;
    }

    std::size_t SofRttTracker::pendingCount() const
    {
        return _pending.size();
    }
} // namespace aerovista::sync
