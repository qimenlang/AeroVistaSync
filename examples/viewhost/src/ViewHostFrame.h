#pragma once

#include <afxframewndex.h>

class CViewHostFrame : public CFrameWndEx
{
public:
    BOOL PreCreateWindow(CREATESTRUCT& cs) override;
    BOOL LoadFrame(UINT resourceId, DWORD defaultStyle = WS_OVERLAPPEDWINDOW | FWS_ADDTOTITLE,
                   CWnd* parentWnd = nullptr, CCreateContext* context = nullptr) override;
    void fitToFormView();

protected:
    afx_msg int OnCreate(LPCREATESTRUCT createStruct);
    void ActivateFrame(int nCmdShow = -1) override;

    DECLARE_MESSAGE_MAP()
};
