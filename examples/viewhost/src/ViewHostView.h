#pragma once

#include <afxext.h>
#include <afxwin.h>
#include <afxcontrolbars.h>

#include <chrono>
#include <cstdint>
#include <string>

#include <aerovista/sync/HostDriver.h>

#include "HudLabel.h"
#include "ViewHostMath.h"
#include "ViewHostMessages.h"
#include "resource.h"

class CViewHostFrame;

class CViewHostView : public CFormView
{
    DECLARE_DYNCREATE(CViewHostView)

public:
    enum { IDD = IDD_VIEWHOST_DIALOG };

    void refreshEntityTree();
    void setEyeControlling(bool controlling);
    void defocusEntityTree();
    void testTcp();
    void testUdp();
    /// Pane 不在 View 祖先链上；Frame 在 WalkPreTranslateTree 末尾转调，避免 Enter / 树单击丢失。
    BOOL handleHostInput(MSG* pMsg);

protected:
    CViewHostView();

    void DoDataExchange(CDataExchange* dx) override;
    void OnInitialUpdate() override;
    BOOL PreTranslateMessage(MSG* pMsg) override;

    afx_msg void OnTimer(UINT_PTR nIDEvent);
    afx_msg void OnDestroy();
    afx_msg LRESULT OnRefreshEntityTree(WPARAM wparam, LPARAM lparam);
    afx_msg LRESULT OnOpenEntityProperties(WPARAM wparam, LPARAM lparam);

    DECLARE_MESSAGE_MAP()

private:
    bool loadConfig();
    void updateStatusText(bool force = false);
    bool commandEditHasFocus() const;
    void submitCommand();
    /// 订阅 IG→Host TCP 上行报文（16 类响应/通知），收到即记录类名到 _lastRecvName（报文自检，§4.7）。
    void subscribeIgPackets();
    void openEntityProperties(std::uint16_t entityId);
    void closeFrame();
    bool applyEyeControlFromCursor(HWND clickHwnd);
    bool isDialogChrome(HWND clickHwnd) const;
    void createFocusSink();
    CViewHostFrame* hostFrame() const;

    aerovista::sync::HostDriver _driver;
    aerovista::sync::cigi_wire::EyePose _eye;
    CEdit _focusSink;
    CHudLabel _statusReady;
    CHudLabel _eyeLat;

    bool _controlling = false;
    std::chrono::steady_clock::time_point _startTime{};
    std::chrono::steady_clock::time_point _lastStatusHud{};
    int _lastReadyIgShown = -1;
    double _lastSimTimeMs = 0.0;
    double _speed = 30.0;    // m/s
    double _turnRate = 60.0; // deg/s

    /// 最近收到的 IG→Host 报文类名（F9/F10 上行报文自检，IDC_STATUS_TEST 同行显示）。
    std::string _lastRecvName;
    /// 最近一次 testtcp/testudp 的链路 + 类名。
    std::string _lastTestName;

    static constexpr UINT_PTR kTimerId = 1;
    static constexpr UINT kTimerPeriodMs = 16; // ~60 fps，viewhost设计.md §4.3
    static constexpr UINT kStatusHudPeriodMs = 100; // 仪表盘文案 ~10Hz，不跟数据面绑死
};

inline CViewHostView* viewHostView(CWnd* from)
{
    if (from == nullptr)
        return nullptr;

    CWnd* site = from;
    if (CBasePane* pane = DYNAMIC_DOWNCAST(CBasePane, from))
    {
        if (CWnd* dockSite = pane->GetDockSiteFrameWnd())
            site = dockSite;
    }

    CFrameWnd* frame = DYNAMIC_DOWNCAST(CFrameWnd, site);
    if (frame == nullptr)
        frame = site->GetParentFrame();
    if (frame == nullptr)
        return nullptr;

    // 无文档 SDI 不会走 InitialUpdateFrame；未点过仪表盘时 GetActiveView 仍是空。
    if (CViewHostView* view = DYNAMIC_DOWNCAST(CViewHostView, frame->GetActiveView()))
        return view;
    return DYNAMIC_DOWNCAST(CViewHostView, frame->GetDlgItem(AFX_IDW_PANE_FIRST));
}
