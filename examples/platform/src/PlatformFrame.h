#pragma once

#include <afxframewndex.h>

#include "CommandPane.h"
#include "IgListPane.h"
#include "PacketProbePane.h"
#include "SceneTreePane.h"

class CPlatformView;

class CPlatformFrame : public CFrameWndEx
{
public:
    /// 固定客户区；无厚边框、无最大化（可最小化）。停靠条占用四周，中间 FormView 只留仪表盘。
    static constexpr DWORD kFrameStyle = WS_OVERLAPPED | WS_CAPTION | WS_SYSMENU | WS_MINIMIZEBOX;

    BOOL PreCreateWindow(CREATESTRUCT& cs) override;
    BOOL PreTranslateMessage(MSG* pMsg) override;
    BOOL LoadFrame(UINT resourceId, DWORD defaultStyle = kFrameStyle | FWS_ADDTOTITLE,
                   CWnd* parentWnd = nullptr, CCreateContext* context = nullptr) override;
    void fitToFormView();
    CPlatformView* hostView();

    CSceneTreePane& sceneTreePane() { return _sceneTreePane; }
    CIgListPane& igListPane() { return _igListPane; }
    CPacketProbePane& packetProbePane() { return _packetProbePane; }
    CCommandPane& commandPane() { return _commandPane; }

protected:
    afx_msg int OnCreate(LPCREATESTRUCT createStruct);
    void ActivateFrame(int nCmdShow = -1) override;

    DECLARE_MESSAGE_MAP()

private:
    bool createPanes();
    bool createDockPane(CDockablePane& pane, UINT id, LPCTSTR caption, DWORD barAlign, DWORD controlBarStyle,
                        CSize size);

    CSceneTreePane _sceneTreePane;
    CPacketProbePane _packetProbePane;
    CIgListPane _igListPane;
    CCommandPane _commandPane;
};
