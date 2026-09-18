#pragma once

#include <afxframewndex.h>

class CViewHostFrame : public CFrameWndEx
{
public:
    BOOL PreCreateWindow(CREATESTRUCT& cs) override;

protected:
    afx_msg int OnCreate(LPCREATESTRUCT createStruct);

    DECLARE_MESSAGE_MAP()
};
