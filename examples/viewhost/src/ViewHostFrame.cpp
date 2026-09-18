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
    if (CDockingManager* docking = GetDockingManager())
        docking->DisableRestoreDockState(TRUE);

    CCreateContext viewContext{};
    viewContext.m_pNewViewClass = RUNTIME_CLASS(CViewHostView);
    viewContext.m_pCurrentFrame = this;
    if (CreateView(&viewContext, AFX_IDW_PANE_FIRST) == nullptr)
        return -1;

    SetWindowText(_T("AeroVista viewhost"));
    return 0;
}

BOOL CViewHostFrame::LoadFrame(UINT resourceId, DWORD defaultStyle, CWnd* parentWnd, CCreateContext* context)
{
    if (!CFrameWndEx::LoadFrame(resourceId, defaultStyle, parentWnd, context))
        return FALSE;
    // OnLoadFrame 会按注册表/默认 overlapping 尺寸改窗口，必须在那之后再收。
    fitToFormView();
    return TRUE;
}

void CViewHostFrame::ActivateFrame(int nCmdShow)
{
    CFrameWndEx::ActivateFrame(nCmdShow);
    fitToFormView();
}

void CViewHostFrame::fitToFormView()
{
    auto* view = DYNAMIC_DOWNCAST(CFormView, GetActiveView());
    if (view == nullptr)
        view = DYNAMIC_DOWNCAST(CFormView, GetDlgItem(AFX_IDW_PANE_FIRST));
    if (view == nullptr || view->GetSafeHwnd() == nullptr)
        return;

    const CSize form = view->GetTotalSize();
    if (form.cx <= 0 || form.cy <= 0)
        return;

    RecalcLayout();

    CRect viewRect;
    view->GetWindowRect(&viewRect);
    const int deltaX = form.cx - viewRect.Width();
    const int deltaY = form.cy - viewRect.Height();
    if (deltaX == 0 && deltaY == 0)
        return;

    CRect frameRect;
    GetWindowRect(&frameRect);
    SetWindowPos(nullptr, 0, 0, frameRect.Width() + deltaX, frameRect.Height() + deltaY,
                 SWP_NOMOVE | SWP_NOZORDER);
    RecalcLayout();
}
