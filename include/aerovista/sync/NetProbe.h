#pragma once

#include <chrono>
#include <cstdint>

namespace aerovista::sync
{
    /// 网络探测报线格式（多通道同步验收测量设计.md §4.8，码 MEAS-rtt）。
    /// 8 字节小端：前 4 字节 magic = 'A' 'V' 'R' 'T'（小端拼成 uint32 = 0x54525641），
    /// 后 4 字节为 uint32 探测序号。配对与统计由 RttTracker 承担，时限是 netRttMatchTimeout。
    inline constexpr int netProbeSize = 8;
    /// 网络探测丢失时限（多通道同步验收测量设计.md §4.8）。本机往返约百微秒量级，1ms 以上记丢失。
    inline constexpr std::chrono::milliseconds netRttMatchTimeout{1};

    inline void packNetProbe(std::uint32_t sequence, unsigned char out[netProbeSize])
    {
        // magic = 'AVRT'（小端 0x54525641）。用字符字面量写出，读和写两处都能直接看出 magic 来源。
        out[0] = 'A';
        out[1] = 'V';
        out[2] = 'R';
        out[3] = 'T';
        out[4] = static_cast<unsigned char>(sequence);
        out[5] = static_cast<unsigned char>(sequence >> 8);
        out[6] = static_cast<unsigned char>(sequence >> 16);
        out[7] = static_cast<unsigned char>(sequence >> 24);
    }

    inline bool isNetProbe(const unsigned char* data, int n)
    {
        return data != nullptr && n == netProbeSize && data[0] == 'A' && data[1] == 'V' &&
               data[2] == 'R' && data[3] == 'T';
    }

    inline std::uint32_t netProbeSequence(const unsigned char* data)
    {
        return static_cast<std::uint32_t>(data[4]) | (static_cast<std::uint32_t>(data[5]) << 8) |
               (static_cast<std::uint32_t>(data[6]) << 16) | (static_cast<std::uint32_t>(data[7]) << 24);
    }
} // namespace aerovista::sync
