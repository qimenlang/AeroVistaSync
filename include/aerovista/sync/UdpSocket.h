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
    /// 同步平面的 UDP 薄封装。
    ///
    /// 一个未绑定发送 socket（显式 `sendTo` 目标）+ 一个绑定 `rcvPort`（INADDR_ANY）
    /// 的非阻塞接收 socket。Windows 上持有 WSAStartup / WSACleanup 引用计数。
    /// 仅 IPv4，与 CIGI 同步平面拓扑一致（socket总结.md §2）。
    class UdpSocket
    {
    public:
        UdpSocket() = default;
        ~UdpSocket();
        UdpSocket(const UdpSocket&) = delete;
        UdpSocket& operator=(const UdpSocket&) = delete;

        /// 创建发送 + 接收 socket，并把接收 socket 绑定到 `rcvPort`（非阻塞）。
        /// `localAddr` / `sndPort` 保留「默认发送端点」语义，供未来默认对端发送使用。
        /// 失败时关闭全部并返回 false。
        bool initialize(const std::string& localAddr, int sndPort, int rcvPort,
                        std::string* outError = nullptr);
        void close();
        bool valid() const { return _valid; }

        /// 非阻塞接收。>0 收到字节，0 无数据，<0 出错。
        int recv(void* buf, int size);

        /// 非阻塞接收并报告 IPv4 源地址/端口。
        /// 返回约定同 `recv`。
        int recvFrom(void* buf, int size, char* fromIp, int fromIpLen, int* fromPort);

        /// 向显式 IPv4 目标发送数据报。返回发送字节数或 -1。
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
