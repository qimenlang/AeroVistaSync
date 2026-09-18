#include "ViewHostApp.h"

#include "ViewHostFrame.h"
#include "resource.h"

#include <afxcontrolbars.h>
#include <commctrl.h>

CViewHostApp theApp;

BOOL CViewHostApp::InitInstance()
{
    INITCOMMONCONTROLSEX icc{};
    icc.dwSize = sizeof(icc);
    icc.dwICC = ICC_WIN95_CLASSES | ICC_TREEVIEW_CLASSES;
    InitCommonControlsEx(&icc);

    if (!CWinAppEx::InitInstance())
        return FALSE;

    if (!AfxOleInit())
    {
        AfxMessageBox(_T("OLE 初始化失败"));
        return FALSE;
    }
    AfxEnableControlContainer();
    EnableTaskbarInteraction(FALSE);

    SetRegistryKey(_T("AeroVista"));
    InitContextMenuManager();
    InitKeyboardManager();
    InitTooltipManager();
    CMFCVisualManager::SetDefaultManager(RUNTIME_CLASS(CMFCVisualManagerWindows));

    CViewHostFrame* frame = new CViewHostFrame;
    m_pMainWnd = frame;
    if (!frame->LoadFrame(IDR_MAINFRAME))
    {
        m_pMainWnd = nullptr;
        delete frame;
        AfxMessageBox(_T("创建主窗口失败"));
        return FALSE;
    }

    frame->ShowWindow(SW_SHOW);
    frame->UpdateWindow();
    return TRUE;
}
