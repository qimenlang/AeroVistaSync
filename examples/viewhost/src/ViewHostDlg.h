#pragma once

#include <afxwin.h>

#include <chrono>
#include <string>

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
    afx_msg void OnTestTcp();
    afx_msg void OnTestUdp();
    afx_msg void OnExit();

    DECLARE_MESSAGE_MAP()

private:
    bool loadConfig();
    void updateStatusText();
    /// 订阅 IG→Host TCP 上行报文（16 类响应/通知），收到即记录类名到 _lastRecvName（报文自检，§4.7）。
    void subscribeIgPackets();

    aerovista::viewhost::HostDriver _driver;
    aerovista::sync::cigi_wire::EyePose _eye;

    bool _controlling = false;
    bool _started = false;
    std::chrono::steady_clock::time_point _startTime{};
    double _lastSimTimeMs = 0.0;
    double _speed = 30.0;    // m/s
    double _turnRate = 60.0; // deg/s

    /// 最近收到的 IG→Host 报文类名（F9/F10 上行报文自检，IDC_STATUS_RECV 显示）。
    std::string _lastRecvName;

    static constexpr UINT_PTR kTimerId = 1;
};
