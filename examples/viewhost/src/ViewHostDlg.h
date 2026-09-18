#pragma once

#include <afxcmn.h>
#include <afxwin.h>

#include <chrono>
#include <string>

#include "HostDriver.h"
#include "ViewHostMath.h"
#include "resource.h"

// 自定义窗口消息（WM_APP 段），不是 resource.h 控件 ID。
// EntityPropDlg 向父窗口 SendMessage，本对话框 ON_MESSAGE 接收，收发必须同一常量。
inline constexpr UINT wmRefreshEntityTree = WM_APP + 20;
inline constexpr UINT wmOpenEntityProperties = WM_APP + 21;

class CViewHostDlg : public CDialog
{
public:
    explicit CViewHostDlg(CWnd* pParent = nullptr);

    enum { IDD = IDD_VIEWHOST_DIALOG };

    void refreshEntityTree();

protected:
    BOOL OnInitDialog() override;
    void OnOK() override {}
    void OnCancel() override {}
    BOOL PreTranslateMessage(MSG* pMsg) override;

    afx_msg void OnTimer(UINT_PTR nIDEvent);
    afx_msg void OnDestroy();
    afx_msg void OnTestTcp();
    afx_msg void OnTestUdp();
    afx_msg void OnExit();
    afx_msg void OnEntityTreeDblClk(NMHDR* notify, LRESULT* result);
    afx_msg LRESULT OnRefreshEntityTree(WPARAM wparam, LPARAM lparam);
    afx_msg LRESULT OnOpenEntityProperties(WPARAM wparam, LPARAM lparam);

    DECLARE_MESSAGE_MAP()

private:
    bool loadConfig();
    void updateStatusText();
    /// 订阅 IG→Host TCP 上行报文（16 类响应/通知），收到即记录类名到 _lastRecvName（报文自检，§4.7）。
    void subscribeIgPackets();
    void openEntityProperties(std::uint16_t entityId);
    struct EntityTreeHit
    {
        HTREEITEM item = nullptr;
        UINT flags = 0;
        CPoint client{};
    };
    EntityTreeHit hitTestEntityTree();
    bool isEntityLeaf(HTREEITEM item);
    bool isEyePointHit(HTREEITEM item, UINT flags) const;
    bool isEmptyTreeHit(HTREEITEM item, UINT flags) const;
    bool applyEyeControlFromCursor(HWND clickHwnd);
    bool isDialogChrome(HWND clickHwnd) const;
    void createFocusSink();
    void defocusEntityTree();
    void setEyeControlling(bool controlling);
    void updateEyePointLabel();

    aerovista::viewhost::HostDriver _driver;
    aerovista::sync::cigi_wire::EyePose _eye;
    CTreeCtrl _entityTree;
    CEdit _focusSink;
    HTREEITEM _rootItem = nullptr;
    HTREEITEM _eyePointItem = nullptr;
    HTREEITEM _entitiesFolder = nullptr;

    bool _controlling = false;
    std::chrono::steady_clock::time_point _startTime{};
    double _lastSimTimeMs = 0.0;
    double _speed = 30.0;    // m/s
    double _turnRate = 60.0; // deg/s

    /// 最近收到的 IG→Host 报文类名（F9/F10 上行报文自检，IDC_STATUS_RECV 显示）。
    std::string _lastRecvName;

    static constexpr UINT_PTR kTimerId = 1;
    static constexpr UINT kTimerPeriodMs = 16; // ~60 fps，viewhost设计.md §4.3
};
