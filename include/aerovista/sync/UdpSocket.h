#pragma once

#include <string>

#ifdef WIN32
#    ifndef NOMINMAX
#        define NOMINMAX
#    endif
#    include <winsock2.h>
#else
#    include <netinet/in.h>
#    include <sys/socket.h>
#    include <unistd.h>
#endif

namespace aerovista::sync
{
    /// Thin UDP wrapper for the sync plane (replaces the legacy MPV Network.h).
    ///
    /// One unbound send socket (explicit `sendTo` targets) plus one non-blocking
    /// receive socket bound to `rcvPort` on INADDR_ANY. Owns the WSAStartup /
    /// WSACleanup reference count on Windows. IPv4 only, matching the CIGI sync
    /// plane topology (socket总结.md §2).
    class UdpSocket
    {
    public:
        UdpSocket() = default;
        ~UdpSocket();
        UdpSocket(const UdpSocket&) = delete;
        UdpSocket& operator=(const UdpSocket&) = delete;

        /// Create the send + receive sockets and bind the receive socket to
        /// `rcvPort` (non-blocking). `localAddr` / `sndPort` keep the legacy
        /// "default send endpoint" semantics for future default-peer sends.
        /// On failure closes everything and returns false.
        bool initialize(const std::string& localAddr, int sndPort, int rcvPort,
                        std::string* outError = nullptr);
        void close();
        bool valid() const { return _valid; }

        /// Non-blocking receive. >0 bytes received, 0 no data, <0 error.
        int recv(void* buf, int size);

        /// Non-blocking receive reporting the IPv4 source address/port.
        /// Same return contract as `recv`.
        int recvFrom(void* buf, int size, char* fromIp, int fromIpLen, int* fromPort);

        /// Send a datagram to an explicit IPv4 destination. Bytes sent or -1 on error.
        int sendTo(const std::string& ip, int port, const void* buf, int size);

    private:
#ifdef WIN32
        using Handle = SOCKET;
#else
        using Handle = int;
#endif
        static constexpr Handle kInvalid = static_cast<Handle>(-1);

        bool openRecvSocket(int rcvPort, std::string* outError);

        Handle _sendSock = kInvalid;
        Handle _recvSock = kInvalid;
        bool _valid = false;
        std::string _sendAddr;
        int _sendPort = 0;
#ifdef WIN32
        bool _wsaAcquired = false;
#endif
    };
} // namespace aerovista::sync
