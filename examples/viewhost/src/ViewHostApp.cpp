#include "ViewHostApp.h"
#include "ViewHostDlg.h"

#include <commctrl.h>

CViewHostApp theApp;

BOOL CViewHostApp::InitInstance()
{
    CWinApp::InitInstance();

    //注册树控件
    INITCOMMONCONTROLSEX icc{};
    icc.dwSize = sizeof(icc);
    icc.dwICC = ICC_TREEVIEW_CLASSES;
    InitCommonControlsEx(&icc);

    CViewHostDlg dlg;
    m_pMainWnd = &dlg;
    dlg.DoModal();

    return FALSE;
}
