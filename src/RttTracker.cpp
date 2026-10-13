#include <aerovista/sync/RttTracker.h>

namespace aerovista::sync
{
    RttTracker::RttTracker(std::chrono::milliseconds matchTimeout) : _matchTimeout(matchTimeout) {}

    void RttTracker::pushCompletion(std::optional<std::chrono::microseconds> sample)
    {
        _completions.push_back(sample);
        if (_completions.size() > avgWindow)
            _completions.pop_front();
    }

    void RttTracker::recordMatch(std::chrono::microseconds rtt)
    {
        _lastRtt = rtt;
        ++_matchCount;
        pushCompletion(rtt);
    }

    void RttTracker::recordTimeout()
    {
        ++_lossCount;
        pushCompletion(std::nullopt);
    }

    void RttTracker::expireInFlightAt(std::chrono::steady_clock::time_point now)
    {
        for (auto it = _inFlight.begin(); it != _inFlight.end();)
        {
            if (now - it->second >= _matchTimeout)
            {
                recordTimeout();
                it = _inFlight.erase(it);
            }
            else
            {
                ++it;
            }
        }
    }

    void RttTracker::onSent(std::uint32_t key, std::chrono::steady_clock::time_point tSend)
    {
        expireInFlightAt(tSend);
        if (_inFlight.find(key) != _inFlight.end())
            return;
        _inFlight.emplace(key, tSend);
    }

    void RttTracker::onReceived(std::uint32_t key, std::chrono::steady_clock::time_point tRecv)
    {
        expireInFlightAt(tRecv);
        const auto it = _inFlight.find(key);
        if (it == _inFlight.end())
            return;
        const auto rtt = std::chrono::duration_cast<std::chrono::microseconds>(tRecv - it->second);
        _inFlight.erase(it);
        recordMatch(rtt);
    }

    std::optional<std::chrono::microseconds> RttTracker::lastRtt() const
    {
        return _lastRtt;
    }

    std::optional<std::chrono::microseconds> RttTracker::avgRtt() const
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

    std::optional<double> RttTracker::lossRate() const
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

    std::size_t RttTracker::matchCount() const
    {
        return _matchCount;
    }

    std::size_t RttTracker::lossCount() const
    {
        return _lossCount;
    }

    std::size_t RttTracker::inFlightCount() const
    {
        return _inFlight.size();
    }
} // namespace aerovista::sync
