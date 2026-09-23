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
        notify(igCtrl);
    }

    void SofCaptureProc::OnPacketReceived(CigiBasePacket* packet)
    {
        auto* sof = dynamic_cast<CigiSOFV4*>(packet);
        if (!sof)
            return;
        lastFrameCntr.store(sof->GetFrameCntr());
        count.fetch_add(1);
        notify(*sof);
    }
} // namespace aerovista::sync
