#include "ViewHostApp.h"
#include "ViewHostDlg.h"

CViewHostApp theApp;

BOOL CViewHostApp::InitInstance()
{
    CWinApp::InitInstance();

    CViewHostDlg dlg;
    m_pMainWnd = &dlg;
    dlg.DoModal();

    return FALSE;
}
