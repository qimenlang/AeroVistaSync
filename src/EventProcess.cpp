#include <aerovista/sync/EventProcess.h>

#include "CigiEntityPositionCtrlV4.h"
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

    void EyeCaptureProc::OnPacketReceived(CigiBasePacket* packet)
    {
        auto* ent = dynamic_cast<CigiEntityPositionCtrlV4*>(packet);
        if (!ent || ent->GetEntityID() != 0)
            return; // 仅捕获 ownship（EntityID==0）眼点；命令实体走业务 processor（§4.1）。
        // 翻译 CCL → HostEyePose（AttachState→frame + 字段提取，§8.1 眼点链路收敛）并经基类订阅投递。
        if (this->_sink)
        {
            HostEyePose pose;
            pose.eulerYprDeg = {ent->GetYaw(), ent->GetPitch(), ent->GetRoll()};
            if (ent->GetAttachState() == CigiBaseEntityPositionCtrl::Detach)
            {
                pose.frame = HostEyeCoordFrame::LLA;
                pose.position = {ent->GetLat(), ent->GetLon(), ent->GetAlt()};
            }
            else
            {
                pose.frame = HostEyeCoordFrame::WORLD_LOCAL;
                pose.position = {ent->GetXoff(), ent->GetYoff(), ent->GetZoff()};
            }
            this->_sink(pose);
        }
    }

    void SofCaptureProc::OnPacketReceived(CigiBasePacket* packet)
    {
        if (dynamic_cast<CigiSOFV4*>(packet))
            count.fetch_add(1);
    }
} // namespace aerovista::sync
