#pragma once

#include <afxwin.h>

#include <cstdint>
#include <string>

#include "HostDriver.h"
#include "resource.h"

class CEntityPropDlg : public CDialog
{
public:
    CEntityPropDlg(aerovista::viewhost::HostDriver& driver, std::uint16_t entityId, CWnd* pParent);

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

    aerovista::viewhost::HostDriver& _driver;
    std::uint16_t _entityId = 0;

    aerovista::sync::EntityAuthorityState _baselineState = aerovista::sync::EntityAuthorityState::ACTIVE;
    std::uint8_t _baselineAlpha = 255;
    aerovista::sync::EntityAuthorityPose _baselinePose;
};
