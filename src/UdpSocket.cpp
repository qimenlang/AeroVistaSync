#include <aerovista/sync/UdpSocket.h>

#include "SocketCommon.h"

#include <cstdio>
#include <cstring>
#include <string>

namespace aerovista::sync
{
    UdpSocket::~UdpSocket()
    {
        close();
    }

    bool UdpSocket::initialize(int sndPort, int rcvPort, std::string* outError)
    {
        close();
        _sendPort = sndPort;

        socket_common::acquireWsa();
        _wsaAcquired = true;

        _sendSock = socket(AF_INET, SOCK_DGRAM, 0);
        if (_sendSock == kInvalid)
        {
            close();
            return socket_common::setError(outError, "create send socket failed");
        }

        if (!openRecvSocket(rcvPort, outError))
        {
            close();
            return false;
        }

        _valid = true;
        return true;
    }

    void UdpSocket::close()
    {
        if (_sendSock != kInvalid)
        {
#ifdef WIN32
            closesocket(_sendSock);
#else
            close(_sendSock);
#endif
            _sendSock = kInvalid;
        }
        if (_recvSock != kInvalid)
        {
#ifdef WIN32
            closesocket(_recvSock);
#else
            close(_recvSock);
#endif
            _recvSock = kInvalid;
        }
        if (_wsaAcquired)
        {
            socket_common::releaseWsa();
            _wsaAcquired = false;
        }
        _valid = false;
    }

    bool UdpSocket::openRecvSocket(int rcvPort, std::string* outError)
    {
        _recvSock = socket(AF_INET, SOCK_DGRAM, 0);
        if (_recvSock == kInvalid)
            return socket_common::setError(outError, "create receive socket failed");

#ifdef WIN32
        u_long nonBlock = 1;
        ioctlsocket(_recvSock, FIONBIO, &nonBlock);
#else
        fcntl(_recvSock, F_SETFL, O_NONBLOCK);
#endif

        sockaddr_in addr{};
        addr.sin_family = AF_INET;
        addr.sin_addr.s_addr = htonl(INADDR_ANY);
        addr.sin_port = htons(static_cast<unsigned short>(rcvPort));
        if (bind(_recvSock, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) != 0)
            return socket_common::setError(outError, "bind receive socket to port " + std::to_string(rcvPort));

        return true;
    }

    int UdpSocket::recv(void* buf, int size)
    {
        if (!_valid)
            return -1;
        return recvfrom(_recvSock, reinterpret_cast<char*>(buf), size, 0, nullptr, nullptr);
    }

    int UdpSocket::recvFrom(void* buf, int size, char* fromIp, int fromIpLen, int* fromPort)
    {
        if (!_valid)
            return -1;

        sockaddr_in from{};
#ifdef WIN32
        int fromLen = sizeof(from);
#else
        socklen_t fromLen = sizeof(from);
#endif
        const int n = recvfrom(_recvSock, reinterpret_cast<char*>(buf), size, 0,
                               reinterpret_cast<sockaddr*>(&from), &fromLen);
        if (n <= 0)
            return n;

        if (fromPort)
            *fromPort = ntohs(from.sin_port);
        if (fromIp && fromIpLen > 0)
        {
#ifdef WIN32
            strncpy_s(fromIp, static_cast<size_t>(fromIpLen), inet_ntoa(from.sin_addr), _TRUNCATE);
#else
            std::snprintf(fromIp, static_cast<size_t>(fromIpLen), "%s", inet_ntoa(from.sin_addr));
#endif
        }
        return n;
    }

    int UdpSocket::sendTo(const std::string& ip, int port, const void* buf, int size)
    {
        if (!_valid || ip.empty() || !buf || size <= 0)
            return -1;

        sockaddr_in dest{};
        dest.sin_family = AF_INET;
        dest.sin_port = htons(static_cast<unsigned short>(port));
        dest.sin_addr.s_addr = inet_addr(ip.c_str());
        if (dest.sin_addr.s_addr == INADDR_NONE)
            return -1;

        return sendto(_sendSock, reinterpret_cast<const char*>(buf), size, 0,
                      reinterpret_cast<const sockaddr*>(&dest), sizeof(dest));
    }

    int UdpSocket::localPort() const
    {
        if (!_valid)
            return 0;
        sockaddr_in addr{};
#ifdef WIN32
        int len = sizeof(addr);
#else
        socklen_t len = sizeof(addr);
#endif
        if (getsockname(_recvSock, reinterpret_cast<sockaddr*>(&addr), &len) != 0)
            return 0;
        return static_cast<int>(ntohs(addr.sin_port));
    }
} // namespace aerovista::sync
