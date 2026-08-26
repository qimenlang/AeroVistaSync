#include <aerovista/sync/TcpSocket.h>

#include "SocketCommon.h"

namespace aerovista::sync
{
    TcpSocket::~TcpSocket()
    {
        close();
    }

    TcpSocket::TcpSocket(TcpSocket&& other) noexcept
        : _sock(other._sock), _listening(other._listening), _wsaAcquired(other._wsaAcquired.exchange(false))
    {
        other._sock = kInvalid;
        other._listening = false;
    }

    TcpSocket& TcpSocket::operator=(TcpSocket&& other) noexcept
    {
        if (this != &other)
        {
            close();
            _sock = other._sock;
            _listening = other._listening;
            _wsaAcquired = other._wsaAcquired.exchange(false);
            other._sock = kInvalid;
            other._listening = false;
        }
        return *this;
    }

    void TcpSocket::close()
    {
        socket_common::closeHandle(_sock);
        if (_wsaAcquired.exchange(false))
            socket_common::releaseWsa();
        _listening = false;
    }

    bool TcpSocket::listen(int port, std::string* outError)
    {
        close();
        socket_common::acquireWsa();
        _wsaAcquired = true;

        _sock = ::socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
        if (!socket_common::isValid(_sock))
        {
            close();
            return socket_common::setError(outError, "create listen socket failed");
        }

        socket_common::setReuseAddr(_sock);

        sockaddr_in addr{};
        addr.sin_family = AF_INET;
        addr.sin_port = htons(static_cast<unsigned short>(port));
        addr.sin_addr.s_addr = htonl(INADDR_ANY);
        if (::bind(_sock, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) != 0)
        {
            close();
            return socket_common::setError(outError, "bind tcp port " + std::to_string(port));
        }

        if (::listen(_sock, 16) != 0)
        {
            close();
            return socket_common::setError(outError, "listen tcp port " + std::to_string(port));
        }

        socket_common::setNonBlocking(_sock, true);
        _listening = true;
        return true;
    }

    bool TcpSocket::accept(TcpSocket& outClient, std::string* outPeerIp)
    {
        if (!_listening || !socket_common::isValid(_sock))
            return false;

        sockaddr_in clientAddr{};
#ifdef WIN32
        int len = sizeof(clientAddr);
#else
        socklen_t len = sizeof(clientAddr);
#endif
        const Handle client = ::accept(_sock, reinterpret_cast<sockaddr*>(&clientAddr), &len);
        if (!socket_common::isValid(client))
            return false;

        outClient.close();
        socket_common::acquireWsa();
        outClient._wsaAcquired = true;
        outClient._sock = client;
        outClient._listening = false;

        if (outPeerIp)
            *outPeerIp = ::inet_ntoa(clientAddr.sin_addr);
        return true;
    }

    bool TcpSocket::connect(const std::string& ip, int port, int timeoutMs, std::string* outError)
    {
        close();
        socket_common::acquireWsa();
        _wsaAcquired = true;

        _sock = ::socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
        if (!socket_common::isValid(_sock))
        {
            close();
            return socket_common::setError(outError, "create tcp socket failed");
        }

        sockaddr_in addr{};
        addr.sin_family = AF_INET;
        addr.sin_port = htons(static_cast<unsigned short>(port));
        addr.sin_addr.s_addr = ::inet_addr(ip.c_str());
        if (addr.sin_addr.s_addr == INADDR_NONE)
        {
            close();
            return socket_common::setError(outError, "invalid ip " + ip);
        }

        socket_common::setNonBlocking(_sock, true);
        const int cr = ::connect(_sock, reinterpret_cast<sockaddr*>(&addr), sizeof(addr));
        if (cr == 0)
        {
            socket_common::setNonBlocking(_sock, false);
            return true;
        }
        if (!socket_common::lastErrorIsConnectInProgress())
        {
            close();
            return socket_common::setError(outError, "connect failed");
        }

        fd_set wfds;
        FD_ZERO(&wfds);
        FD_SET(_sock, &wfds);
        timeval tv{};
        tv.tv_sec = timeoutMs / 1000;
        tv.tv_usec = (timeoutMs % 1000) * 1000;
#ifdef WIN32
        const int sel = ::select(0, nullptr, &wfds, nullptr, &tv);
#else
        const int sel = ::select(_sock + 1, nullptr, &wfds, nullptr, &tv);
#endif
        if (sel <= 0)
        {
            close();
            return socket_common::setError(outError, "connect timeout");
        }

        int soError = 0;
#ifdef WIN32
        int optLen = sizeof(soError);
        ::getsockopt(_sock, SOL_SOCKET, SO_ERROR, reinterpret_cast<char*>(&soError), &optLen);
#else
        socklen_t optLen = sizeof(soError);
        ::getsockopt(_sock, SOL_SOCKET, SO_ERROR, &soError, &optLen);
#endif
        socket_common::setNonBlocking(_sock, false);
        if (soError != 0)
        {
            close();
            return socket_common::setError(outError, "connect error");
        }
        return true;
    }

    bool TcpSocket::sendAll(const void* data, int len)
    {
        if (!socket_common::isValid(_sock) || !data || len <= 0)
            return false;
        const char* p = static_cast<const char*>(data);
        int sent = 0;
        while (sent < len)
        {
            const int n = ::send(_sock, p + sent, len - sent, 0);
            if (n <= 0)
                return false;
            sent += n;
        }
        return true;
    }

    RecvOutcome TcpSocket::recv(void* buf, int size)
    {
        if (!socket_common::isValid(_sock) || !buf || size <= 0)
            return {RecvKind::IO_ERROR, 0};
        const int n = ::recv(_sock, static_cast<char*>(buf), size, 0);
        if (n > 0)
            return {RecvKind::DATA, n};
        if (n == 0)
            return {RecvKind::PEER_CLOSED, 0};
        return {socket_common::lastErrorIsTimeout() ? RecvKind::TIMEOUT : RecvKind::IO_ERROR, 0};
    }

    void TcpSocket::setRecvTimeout(int timeoutMs)
    {
        return;
        if (socket_common::isValid(_sock))
            socket_common::setRecvTimeout(_sock, timeoutMs);
    }

    bool TcpSocket::recvAll(void* data, int len, int timeoutMs)
    {
        if (!socket_common::isValid(_sock))
            return false;
        socket_common::setRecvTimeout(_sock, timeoutMs);
        char* p = static_cast<char*>(data);
        int got = 0;
        while (got < len)
        {
            const int n = ::recv(_sock, p + got, len - got, 0);
            if (n <= 0)
                return false;
            got += n;
        }
        return true;
    }

    int TcpSocket::localPort() const
    {
        if (!socket_common::isValid(_sock))
            return 0;
        sockaddr_in addr{};
#ifdef WIN32
        int len = sizeof(addr);
#else
        socklen_t len = sizeof(addr);
#endif
        if (::getsockname(_sock, reinterpret_cast<sockaddr*>(&addr), &len) != 0)
            return 0;
        return static_cast<int>(ntohs(addr.sin_port));
    }
} // namespace aerovista::sync
