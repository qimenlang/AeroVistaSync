#include "ViewHostFrame.h"

#include "ViewHostView.h"

#include <afxcontrolbars.h>

BEGIN_MESSAGE_MAP(CViewHostFrame, CFrameWndEx)
    ON_WM_CREATE()
END_MESSAGE_MAP()

BOOL CViewHostFrame::PreCreateWindow(CREATESTRUCT& cs)
{
    if (!CFrameWndEx::PreCreateWindow(cs))
        return FALSE;
    cs.style &= ~FWS_ADDTOTITLE;
    cs.dwExStyle &= ~WS_EX_CLIENTEDGE;
    cs.lpszName = _T("AeroVista viewhost");
    return TRUE;
}

int CViewHostFrame::OnCreate(LPCREATESTRUCT createStruct)
{
    if (CFrameWndEx::OnCreate(createStruct) == -1)
        return -1;

    EnableDocking(CBRS_ALIGN_ANY);

    CCreateContext viewContext{};
    viewContext.m_pNewViewClass = RUNTIME_CLASS(CViewHostView);
    viewContext.m_pCurrentFrame = this;
    if (CreateView(&viewContext, AFX_IDW_PANE_FIRST) == nullptr)
        return -1;

    SetWindowText(_T("AeroVista viewhost"));
    return 0;
}
