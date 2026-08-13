#pragma once

#include "CigiIncludes.h"

#include "CigiBaseEventProcessor.h"
#include "CigiIGCtrlV4.h"
#include "CigiSOFV4.h"

namespace aerovista::sync
{
    class IGCtrl : public CigiBaseEventProcessor
    {
    public:
        IGCtrl() = default;
        ~IGCtrl() override = default;

        void OnPacketReceived(CigiBasePacket* packet) override;

        void setOrigPacket(CigiIGCtrlV4* packetIn) { _packet = packetIn; }

    protected:
        CigiIGCtrlV4* _packet = nullptr;
    };

    class SofProcessor : public CigiBaseEventProcessor
    {
    public:
        SofProcessor() = default;
        ~SofProcessor() override = default;

        void OnPacketReceived(CigiBasePacket* packet) override;

        void setOrigPacket(CigiSOFV4* packetIn) { _packet = packetIn; }

    protected:
        CigiSOFV4* _packet = nullptr;
    };
} // namespace aerovista::sync
