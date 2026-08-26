#pragma once

#include <afxwin.h>

#include <chrono>

#include "HostDriver.h"
#include "ViewHostMath.h"
#include "resource.h"

class CViewHostDlg : public CDialog
{
public:
    explicit CViewHostDlg(CWnd* pParent = nullptr);

    enum { IDD = IDD_VIEWHOST_DIALOG };

protected:
    BOOL OnInitDialog() override;
    void OnOK() override {}
    void OnCancel() override {}
    BOOL PreTranslateMessage(MSG* pMsg) override;

    afx_msg void OnTimer(UINT_PTR nIDEvent);
    afx_msg void OnDestroy();
    afx_msg void OnToggleControl();
    afx_msg void OnPlaceEntity();
    afx_msg void OnExit();

    DECLARE_MESSAGE_MAP()

private:
    bool loadConfig();
    void updateStatusText();

    aerovista::viewhost::HostDriver _driver;
    aerovista::sync::cigi_wire::EyePose _eye;

    bool _controlling = false;
    bool _started = false;
    std::chrono::steady_clock::time_point _startTime{};
    double _lastSimTimeMs = 0.0;
    double _speed = 30.0;    // m/s
    double _turnRate = 60.0; // deg/s

    static constexpr UINT_PTR kTimerId = 1;
};
