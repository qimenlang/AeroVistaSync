#pragma once

#include <string>

namespace aerovista::sync
{
    class HostSync;

    /// Host→IG 报文自检：按链路随机一条 V4（TCP 命令面 / UDP 数据面，viewhost设计.md §4.7）。
    /// IGCtrl 与 ownship 眼点不在此列。返回发出的报文类名。
    std::string sendRandomTcpProbe(HostSync& host);
    std::string sendRandomUdpProbe(HostSync& host);
} // namespace aerovista::sync
