#include <aerovista/sync/SofRttTracker.h>

namespace aerovista::sync
{
    void SofRttTracker::pushCompletion(std::optional<std::chrono::microseconds> sample)
    {
        _completions.push_back(sample);
        if (_completions.size() > avgWindow)
            _completions.pop_front();
    }

    void SofRttTracker::recordMatch(std::chrono::microseconds rtt)
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

    void SofRttTracker::expire(std::chrono::steady_clock::time_point now)
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

    void SofRttTracker::onIgCtrlSent(std::uint32_t hostFrameNumber, std::chrono::steady_clock::time_point tSend)
    {
        expire(tSend);
        if (_pending.find(hostFrameNumber) != _pending.end())
            return;
        _pending.emplace(hostFrameNumber, tSend);
    }

    void SofRttTracker::onSofReceived(std::uint32_t echoedHostFrameNumber, std::chrono::steady_clock::time_point tRecv)
    {
        expire(tRecv);
        const auto it = _pending.find(echoedHostFrameNumber);
        if (it == _pending.end())
            return;
        const auto rtt = std::chrono::duration_cast<std::chrono::microseconds>(tRecv - it->second);
        _pending.erase(it);
        recordMatch(rtt);
    }

    std::optional<std::chrono::microseconds> SofRttTracker::lastRtt() const
    {
        return _lastRtt;
    }

    std::optional<std::chrono::microseconds> SofRttTracker::avgRtt() const
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
        return std::chrono::microseconds{sumUs / static_cast<std::int64_t>(matches)};
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
