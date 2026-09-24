#pragma once

#include <afxwin.h>

#include <cstdint>
#include <string>

#include <aerovista/sync/HostDataManager.h>
#include <aerovista/sync/HostSync.h>

#include "resource.h"

class CEntityPropDlg : public CDialog
{
public:
    CEntityPropDlg(aerovista::sync::HostSync& host, aerovista::sync::HostDataManager& data, std::uint16_t entityId,
                   CWnd* pParent);

    enum { IDD = IDD_ENTITY_PROP };

protected:
    BOOL OnInitDialog() override;
    void OnOK() override {}
    void OnCancel() override { EndDialog(IDCANCEL); }

    afx_msg void OnApply();
    afx_msg void OnReset();

    DECLARE_MESSAGE_MAP()

private:
    bool loadRow();
    void captureBaseline();
    bool applyDirtyFields(std::string& error);

    aerovista::sync::HostSync& _host;
    aerovista::sync::HostDataManager& _data;
    std::uint16_t _entityId = 0;

    aerovista::sync::EntityAuthorityState _baselineState = aerovista::sync::EntityAuthorityState::ACTIVE;
    std::uint8_t _baselineAlpha = 255;
    aerovista::sync::EntityAuthorityPose _baselinePose;
};
