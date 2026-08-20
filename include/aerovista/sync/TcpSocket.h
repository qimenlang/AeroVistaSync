#pragma once

#include <atomic>
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
    /// recv 的一次结果。TCP 字节流需要区分「数据 / 对端关闭 / 超时 / 错误」，
    /// 与 UdpSocket::recv 的 int 约定不同（UDP 无连接，无 EOF 语义）。
    enum class RecvKind
    {
        DATA,        ///< 读到数据，bytes > 0
        PEER_CLOSED, ///< recv 返回 0：对端关闭写方向（断线）
        TIMEOUT,     ///< 非阻塞无数据 / SO_RCVTIMEO 超时（可继续，非断线）
        IO_ERROR     ///< 其他错误（断线）
    };

    struct RecvOutcome
    {
        RecvKind kind = RecvKind::IO_ERROR;
        int bytes = 0;
    };

    /// 同步平面的 TCP 封装：一个类同时承担「服务端监听」与「已连接收发」两种形态。
    ///
    /// 生命周期：Idle → (listen) Listening → (accept) 产出 Connected；或 Idle → (connect) Connected。
    /// Windows 上 WSAStartup / WSACleanup 引用计数与 UdpSocket 共享（SocketCommon）。
    /// 仅 IPv4。长度分帧（CommandFrameAssembler）与握手协议（WireMsg）不在此层。
    class TcpSocket
    {
    public:
        TcpSocket() = default;
        ~TcpSocket();
        TcpSocket(const TcpSocket&) = delete;
        TcpSocket& operator=(const TcpSocket&) = delete;
        TcpSocket(TcpSocket&& other) noexcept;
        TcpSocket& operator=(TcpSocket&& other) noexcept;

        /// 服务端：socket + SO_REUSEADDR + bind + listen（非阻塞）。失败关闭全部并返回 false。
        bool listen(int port, std::string* outError = nullptr);

        /// 服务端：非阻塞 accept 一个连接，产出新的已连接 TcpSocket。this 保持监听态。
        /// 无挂起连接时返回 false（outClient 不变）。
        bool accept(TcpSocket& outClient, std::string* outPeerIp = nullptr);

        /// 客户端：非阻塞 connect + select 等待 + SO_ERROR 校验。超时/失败返回 false。
        bool connect(const std::string& ip, int port, int timeoutMs, std::string* outError = nullptr);

        /// 已连接 socket：循环发送全部字节。失败返回 false。
        bool sendAll(const void* data, int len);

        /// 已连接 socket：读一次，结果语义见 RecvOutcome。
        RecvOutcome recv(void* buf, int size);

        /// 已连接 socket：设 SO_RCVTIMEO（命令读循环弱心跳用）。
        void setRecvTimeout(int timeoutMs);

        /// 已连接 socket：按超时读满 len 字节。EOF/超时/错误返回 false。
        bool recvAll(void* data, int len, int timeoutMs);

        void close();
        bool valid() const { return _sock != kInvalid; }
        /// 是否监听态（accept 才有效）。
        bool listening() const { return _listening; }
        /// 已绑定 socket 的实际本地端口（listen(0) 时由 OS 分配）。无效/失败返回 0。
        int localPort() const;

    private:
#ifdef WIN32
        using Handle = SOCKET;
#else
        using Handle = int;
#endif
        static constexpr Handle kInvalid = static_cast<Handle>(-1);

        Handle _sock = kInvalid;
        bool _listening = false;
        std::atomic<bool> _wsaAcquired{false};
    };
} // namespace aerovista::sync
