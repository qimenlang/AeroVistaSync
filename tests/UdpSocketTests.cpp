// UdpSocket 单元测试：本机 loopback 验证 socket 基础功能分层。
// 覆盖：sendTo/recv 自环、recvFrom 回填源 IP/端口、非法参数/未初始化返回 -1、
//       端口占用冲突失败、close 幂等。
// 不依赖真实网络（127.0.0.1），也不走 HostSync/IgSync 业务层。

#include <catch2/catch_test_macros.hpp>

#include <aerovista/sync/UdpSocket.h>

#include <cstring>
#include <string>

using aerovista::sync::UdpSocket;

namespace
{
    // 绑定 rcvPort=0（OS 分配）的接收 socket；返回其实际端口。
    bool bindEphemeral(UdpSocket& sock, int& outPort)
    {
        if (!sock.initialize(0))
            return false;
        outPort = sock.localPort();
        return outPort > 0;
    }
} // namespace

// =============================================================================
// 本机回环上 sendTo 的字节能被 recv 原样读回（sync模块化设计.md §7，码 UDP-loopback）。
// 不经过 HostSync。非阻塞 recv 允许少量重试，躲开入队竞态。
// =============================================================================
TEST_CASE("UdpSocket sendTo/recv loopback 自环收发", "[unit][UDP-loopback]")
{
    UdpSocket recvSock;
    int port = 0;
    REQUIRE(bindEphemeral(recvSock, port));

    UdpSocket sendSock;
    REQUIRE(sendSock.initialize(0));

    const char msg[] = "hello-udp";
    REQUIRE(sendSock.sendTo("127.0.0.1", port, msg, sizeof(msg)) == static_cast<int>(sizeof(msg)));

    // loopback sendto 成功后数据已入接收队列；非阻塞 recv 应立即读到，重试兜底极小竞态。
    char buf[64]{};
    int n = -1;
    for (int i = 0; i < 20 && n <= 0; ++i)
        n = recvSock.recv(buf, sizeof(buf));
    REQUIRE(n == static_cast<int>(sizeof(msg)));
    REQUIRE(std::memcmp(buf, msg, sizeof(msg)) == 0);
}

// =============================================================================
// recvFrom 回填发送方 IPv4（sync模块化设计.md §7，码 UDP-recvfrom-peer）。
// 地址钉 127.0.0.1。对端端口由系统分配，只要求大于 0。
// =============================================================================
TEST_CASE("UdpSocket recvFrom 回填源 IP 与端口", "[unit][UDP-recvfrom-peer]")
{
    UdpSocket recvSock;
    int port = 0;
    REQUIRE(bindEphemeral(recvSock, port));

    UdpSocket sendSock;
    REQUIRE(sendSock.initialize(0));

    const char msg[] = "from-test";
    REQUIRE(sendSock.sendTo("127.0.0.1", port, msg, sizeof(msg)) > 0);

    char buf[64]{};
    char fromIp[64]{};
    int fromPort = 0;
    const int n = recvSock.recvFrom(buf, sizeof(buf), fromIp, sizeof(fromIp), &fromPort);
    REQUIRE(n == static_cast<int>(sizeof(msg)));
    REQUIRE(std::string(fromIp) == "127.0.0.1");
    REQUIRE(fromPort > 0);
}

// =============================================================================
// sendTo 遇到空地址、非法地址、空缓冲或长度为 0 时返回 -1（sync模块化设计.md §7，码 UDP-send-bad-arg）。
// 四次都是参数拒绝，不要求错误字符串。
// =============================================================================
TEST_CASE("UdpSocket sendTo 非法参数返回 -1", "[unit][UDP-send-bad-arg]")
{
    UdpSocket sock;
    REQUIRE(sock.initialize(0));

    const char msg[] = "x";
    REQUIRE(sock.sendTo("", 12345, msg, sizeof(msg)) == -1);              // 空 ip
    REQUIRE(sock.sendTo("not_an_ip", 12345, msg, sizeof(msg)) == -1);     // 非法 ip
    REQUIRE(sock.sendTo("127.0.0.1", 12345, nullptr, sizeof(msg)) == -1); // 空 buf
    REQUIRE(sock.sendTo("127.0.0.1", 12345, msg, 0) == -1);               // size<=0
}

// =============================================================================
// 未 initialize 时 recv、recvFrom 和 sendTo 都返回 -1（sync模块化设计.md §7，码 UDP-uninit）。
// 先确认 valid() 为假。
// =============================================================================
TEST_CASE("UdpSocket 未初始化时 recv/sendTo 返回 -1", "[unit][UDP-uninit]")
{
    UdpSocket sock;
    REQUIRE_FALSE(sock.valid());

    char buf[8]{};
    REQUIRE(sock.recv(buf, sizeof(buf)) == -1);

    char fromIp[16]{};
    int fromPort = 0;
    REQUIRE(sock.recvFrom(buf, sizeof(buf), fromIp, sizeof(fromIp), &fromPort) == -1);

    REQUIRE(sock.sendTo("127.0.0.1", 1, buf, sizeof(buf)) == -1);
}

// =============================================================================
// 端口已被占用时 initialize 失败，套接字保持无效（sync模块化设计.md §7，码 UDP-port-busy）。
// 错误文本非空即可，不钉原文。
// =============================================================================
TEST_CASE("UdpSocket initialize 端口占用冲突返回 false", "[unit][UDP-port-busy]")
{
    UdpSocket first;
    int port = 0;
    REQUIRE(bindEphemeral(first, port));

    UdpSocket second;
    std::string error;
    REQUIRE_FALSE(second.initialize(port, &error));
    REQUIRE_FALSE(error.empty());
    REQUIRE_FALSE(second.valid());
}

// =============================================================================
// close 之后再次 close 仍然是无效套接字，且不崩溃（sync模块化设计.md §7，码 UDP-close-idempotent）。
// =============================================================================
TEST_CASE("UdpSocket close 幂等", "[unit][UDP-close-idempotent]")
{
    UdpSocket sock;
    REQUIRE(sock.initialize(0));
    REQUIRE(sock.valid());

    sock.close();
    REQUIRE_FALSE(sock.valid());
    sock.close(); // 幂等，不崩溃
    REQUIRE_FALSE(sock.valid());
}
