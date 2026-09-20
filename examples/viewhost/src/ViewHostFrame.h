#pragma once

#include <afxframewndex.h>

class CViewHostFrame : public CFrameWndEx
{
public:
    /// 固定客户区等于 FormView 模板；无厚边框、无最大化（可最小化）。
    static constexpr DWORD kFrameStyle = WS_OVERLAPPED | WS_CAPTION | WS_SYSMENU | WS_MINIMIZEBOX;

    BOOL PreCreateWindow(CREATESTRUCT& cs) override;
    BOOL LoadFrame(UINT resourceId, DWORD defaultStyle = kFrameStyle | FWS_ADDTOTITLE,
                   CWnd* parentWnd = nullptr, CCreateContext* context = nullptr) override;
    void fitToFormView();

protected:
    afx_msg int OnCreate(LPCREATESTRUCT createStruct);
    void ActivateFrame(int nCmdShow = -1) override;

    DECLARE_MESSAGE_MAP()
};
