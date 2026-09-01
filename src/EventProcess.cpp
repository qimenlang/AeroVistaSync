#include <aerovista/sync/EventProcess.h>

#include "CigiSOFV4.h"

namespace aerovista::sync
{
    void IgCtrlCaptureProc::OnPacketReceived(CigiBasePacket* packet)
    {
        auto* ig = dynamic_cast<CigiIGCtrlV4*>(packet);
        if (!ig)
            return;
        // CCL 报文对象是复用单例，必须值拷贝缓存（§8.1 通用模式）。
        got = true;
        igCtrl = *ig;
    }

    void SofCaptureProc::OnPacketReceived(CigiBasePacket* packet)
    {
        if (dynamic_cast<CigiSOFV4*>(packet))
            count.fetch_add(1);
    }
} // namespace aerovista::sync
