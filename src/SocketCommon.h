#pragma once

// 传输层内部：Winsock / POSIX socket 平台差异收敛（不对外公开）。
// UdpSocket / TcpSocket 共用，避免公开头泄漏 winsock 细节与重复平台分支。

#include <string>

#ifdef WIN32
#    ifndef NOMINMAX
#        define NOMINMAX
#    endif
#    include <winsock2.h>
#    include <ws2tcpip.h>
#else
#    include <arpa/inet.h>
#    include <cerrno>
#    include <fcntl.h>
#    include <netinet/in.h>
#    include <sys/socket.h>
#    include <unistd.h>
#endif

namespace aerovista::sync
{
    namespace socket_common
    {
#ifdef WIN32
        using Handle = SOCKET;
#else
        using Handle = int;
#endif

        inline Handle invalidHandle()
        {
#ifdef WIN32
            return INVALID_SOCKET;
#else
            return -1;
#endif
        }

        inline bool isValid(Handle s)
        {
#ifdef WIN32
            return s != INVALID_SOCKET;
#else
            return s >= 0;
#endif
        }

        // WSAStartup / WSACleanup 引用计数（进程内全局，UDP/TCP 共用）。
        // 非 Windows 下为空操作，调用方无需 #ifdef。
        void acquireWsa();
        void releaseWsa();

        inline void closeHandle(Handle& s)
        {
            if (!isValid(s))
                return;
#ifdef WIN32
            closesocket(s);
#else
            ::close(s);
#endif
            s = invalidHandle();
        }

        inline void setNonBlocking(Handle s, bool on)
        {
#ifdef WIN32
            u_long mode = on ? 1 : 0;
            ioctlsocket(s, FIONBIO, &mode);
#else
            const int flags = fcntl(s, F_GETFL, 0);
            fcntl(s, F_SETFL, on ? (flags | O_NONBLOCK) : (flags & ~O_NONBLOCK));
#endif
        }

        inline bool setRecvTimeout(Handle s, int timeoutMs)
        {
#ifdef WIN32
            const DWORD tv = static_cast<DWORD>(timeoutMs);
            return setsockopt(s, SOL_SOCKET, SO_RCVTIMEO, reinterpret_cast<const char*>(&tv), sizeof(tv)) == 0;
#else
            timeval tv{};
            tv.tv_sec = timeoutMs / 1000;
            tv.tv_usec = (timeoutMs % 1000) * 1000;
            return setsockopt(s, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv)) == 0;
#endif
        }

        inline void setReuseAddr(Handle s)
        {
            int yes = 1;
#ifdef WIN32
            setsockopt(s, SOL_SOCKET, SO_REUSEADDR, reinterpret_cast<const char*>(&yes), sizeof(yes));
#else
            setsockopt(s, SOL_SOCKET, SO_REUSEADDR, &yes, sizeof(yes));
#endif
        }

        // 最近一次 recv/send 失败是否「非阻塞无数据 / 超时」（可重试，不视为断线）。
        inline bool lastErrorIsTimeout()
        {
#ifdef WIN32
            const int err = WSAGetLastError();
            return err == WSAETIMEDOUT || err == WSAEWOULDBLOCK;
#else
            return errno == EAGAIN || errno == EWOULDBLOCK;
#endif
        }

        // 非阻塞 connect 是否处于「连接进行中」。
        inline bool lastErrorIsConnectInProgress()
        {
#ifdef WIN32
            const int err = WSAGetLastError();
            return err == WSAEWOULDBLOCK || err == WSAEINPROGRESS;
#else
            return errno == EINPROGRESS;
#endif
        }

        inline bool setError(std::string* outError, const std::string& message)
        {
            if (outError)
                *outError = message;
            return false;
        }
    } // namespace socket_common
} // namespace aerovista::sync
