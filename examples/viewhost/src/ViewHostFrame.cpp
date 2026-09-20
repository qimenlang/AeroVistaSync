#include "ViewHostFrame.h"

#include "ViewHostView.h"
#include "resource.h"

#include <afxcontrolbars.h>

BEGIN_MESSAGE_MAP(CViewHostFrame, CFrameWndEx)
    ON_WM_CREATE()
END_MESSAGE_MAP()

namespace
{
    constexpr int kTreePaneWidth = 220;
    constexpr int kProbePaneHeight = 72;
    constexpr int kIgListPaneHeight = 300;
    constexpr int kCommandPaneHeight = 88;
    constexpr int kDashboardMinWidth = 480;
    constexpr int kDashboardMinHeight = 80;

    constexpr DWORD kChildBar = WS_CHILD | WS_VISIBLE | WS_CLIPSIBLINGS | WS_CLIPCHILDREN;
    /// 钉在预定边：可拉分隔条改厚度，不能撕成浮动窗、不能关。
    constexpr DWORD kPinnedPane = AFX_CBRS_RESIZE;
}

BOOL CViewHostFrame::PreCreateWindow(CREATESTRUCT& cs)
{
    if (!CFrameWndEx::PreCreateWindow(cs))
        return FALSE;
    cs.style &= ~(WS_THICKFRAME | WS_MAXIMIZEBOX);
    cs.style |= CViewHostFrame::kFrameStyle;
    cs.style &= ~FWS_ADDTOTITLE;
    cs.dwExStyle &= ~WS_EX_CLIENTEDGE;
    cs.lpszName = _T("AeroVista viewhost");
    return TRUE;
}

bool CViewHostFrame::createDockPane(CDockablePane& pane, UINT id, LPCTSTR caption, DWORD barAlign,
                                    DWORD controlBarStyle, CSize size)
{
    const DWORD style = kChildBar | barAlign;
    return pane.Create(caption, this, CRect(0, 0, size.cx, size.cy), TRUE, id, style, AFX_CBRS_REGULAR_TABS,
                       controlBarStyle) != FALSE;
}

bool CViewHostFrame::createPanes()
{
    if (!createDockPane(_sceneTreePane, ID_PANE_SCENE_TREE, _T("场景树"), CBRS_LEFT, kPinnedPane,
                        CSize(kTreePaneWidth, 400)))
        return false;
    if (!createDockPane(_packetProbePane, ID_PANE_PACKET_PROBE, _T("报文自检"), CBRS_TOP, kPinnedPane,
                        CSize(400, kProbePaneHeight)))
        return false;
    // IG 先占满底栏高度；命令行再从底下拆走一截。若反过来 DockToWindow，MFC 会把插入格裁成原格一半，列表被挡。
    if (!createDockPane(_igListPane, ID_PANE_IG_LIST, _T("IG 连接"), CBRS_BOTTOM, kPinnedPane,
                        CSize(400, kIgListPaneHeight + kCommandPaneHeight + 16)))
        return false;
    if (!createDockPane(_commandPane, ID_PANE_COMMAND, _T("命令行"), CBRS_BOTTOM, kPinnedPane,
                        CSize(400, kCommandPaneHeight)))
        return false;

    _sceneTreePane.EnableDocking(CBRS_ALIGN_LEFT);
    _packetProbePane.EnableDocking(CBRS_ALIGN_TOP);
    _commandPane.EnableDocking(CBRS_ALIGN_BOTTOM);
    _igListPane.EnableDocking(CBRS_ALIGN_BOTTOM);
    _sceneTreePane.m_bDisableMove = true;
    _packetProbePane.m_bDisableMove = true;
    _commandPane.m_bDisableMove = true;
    _igListPane.m_bDisableMove = true;
    _sceneTreePane.SetMinSize(CSize(140, 160));
    _packetProbePane.SetMinSize(CSize(220, 64));
    _commandPane.SetMinSize(CSize(120, 72));
    _igListPane.SetMinSize(CSize(200, 260));

    DockPane(&_sceneTreePane);
    DockPane(&_packetProbePane);
    DockPane(&_igListPane);
    if (!_commandPane.DockToWindow(&_igListPane, CBRS_ALIGN_BOTTOM))
        DockPane(&_commandPane);
    return true;
}

int CViewHostFrame::OnCreate(LPCREATESTRUCT createStruct)
{
    if (CFrameWndEx::OnCreate(createStruct) == -1)
        return -1;

    EnableDocking(CBRS_ALIGN_ANY);
    if (CDockingManager* docking = GetDockingManager())
        docking->DisableRestoreDockState(TRUE);

    if (!createPanes())
        return -1;

    CCreateContext viewContext{};
    viewContext.m_pNewViewClass = RUNTIME_CLASS(CViewHostView);
    viewContext.m_pCurrentFrame = this;
    CWnd* createdView = CreateView(&viewContext, AFX_IDW_PANE_FIRST);
    if (createdView == nullptr)
        return -1;
    // 无 CDocument，不会走 InitialUpdateFrame；不主动 SetActiveView 的话 Pane 回调拿不到 View。
    if (CView* view = DYNAMIC_DOWNCAST(CView, createdView))
        SetActiveView(view, FALSE);

    SetWindowText(_T("AeroVista viewhost"));
    return 0;
}

CViewHostView* CViewHostFrame::hostView()
{
    if (CViewHostView* view = DYNAMIC_DOWNCAST(CViewHostView, GetActiveView()))
        return view;
    return DYNAMIC_DOWNCAST(CViewHostView, GetDlgItem(AFX_IDW_PANE_FIRST));
}

BOOL CViewHostFrame::PreTranslateMessage(MSG* pMsg)
{
    // WalkPreTranslateTree 只走「焦点控件 → 祖先 → Frame」，停靠条与 FormView 是兄弟。
    // 不能转调 View::PreTranslateMessage：CFormView 会再调 Frame，形成递归。
    if (CViewHostView* view = hostView())
    {
        if (view->handleHostInput(pMsg))
            return TRUE;
    }
    return CFrameWndEx::PreTranslateMessage(pMsg);
}

BOOL CViewHostFrame::LoadFrame(UINT resourceId, DWORD defaultStyle, CWnd* parentWnd, CCreateContext* context)
{
    if (!CFrameWndEx::LoadFrame(resourceId, defaultStyle, parentWnd, context))
        return FALSE;
    // LoadFrame 必须能 LoadMenu(IDR_MAINFRAME)；空菜单满足加载，再去掉可见菜单栏。
    if (CWnd* menuBar = GetDlgItem(AFX_IDW_MENUBAR))
        menuBar->ShowWindow(SW_HIDE);
    SetMenu(nullptr);
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
    RecalcLayout();

    CFormView* view = hostView();

    CSize form(kDashboardMinWidth, kDashboardMinHeight);
    if (view != nullptr && view->GetSafeHwnd() != nullptr)
    {
        const CSize total = view->GetTotalSize();
        if (total.cx > 0)
            form.cx = total.cx;
        if (total.cy > 0)
            form.cy = total.cy;
    }
    if (form.cx < kDashboardMinWidth)
        form.cx = kDashboardMinWidth;
    if (form.cy < kDashboardMinHeight)
        form.cy = kDashboardMinHeight;

    const int clientW = kTreePaneWidth + form.cx + 24;
    // 底栏 Create 尺寸不含停靠标题/分隔条；多留一截以免 RecalcLayout 把 IG/命令行压到只剩标题。
    const int clientH = kProbePaneHeight + form.cy + kIgListPaneHeight + kCommandPaneHeight + 88;

    CRect windowRect;
    GetWindowRect(&windowRect);
    CRect clientRect;
    GetClientRect(&clientRect);
    SetWindowPos(nullptr, 0, 0, windowRect.Width() - clientRect.Width() + clientW,
                 windowRect.Height() - clientRect.Height() + clientH, SWP_NOMOVE | SWP_NOZORDER);
    RecalcLayout();
}
