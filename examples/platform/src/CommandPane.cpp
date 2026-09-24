#include "CommandPane.h"

#include "resource.h"

BEGIN_MESSAGE_MAP(CCommandPane, CDockablePane)
    ON_WM_CREATE()
    ON_WM_SIZE()
END_MESSAGE_MAP()

int CCommandPane::OnCreate(LPCREATESTRUCT createStruct)
{
    if (CDockablePane::OnCreate(createStruct) == -1)
        return -1;

    constexpr DWORD kEdit = WS_CHILD | WS_VISIBLE | WS_BORDER | WS_TABSTOP | ES_AUTOHSCROLL;
    CRect client;
    GetClientRect(&client);
    if (!_edit.Create(kEdit, client, this, IDC_COMMAND))
        return -1;
    _edit.SetFont(&afxGlobalData.fontRegular);
    adjustLayout();
    return 0;
}

void CCommandPane::OnSize(UINT type, int cx, int cy)
{
    CDockablePane::OnSize(type, cx, cy);
    adjustLayout();
}

void CCommandPane::adjustLayout()
{
    if (_edit.GetSafeHwnd() == nullptr)
        return;

    CRect client;
    GetClientRect(&client);
    const int pad = 4;
    int editW = client.Width() - pad * 2;
    int editH = client.Height() - pad * 2;
    if (editW < 0)
        editW = 0;
    if (editH < 0)
        editH = 0;
    _edit.SetWindowPos(nullptr, pad, pad, editW, editH, SWP_NOZORDER);
}

bool CCommandPane::hasFocus() const
{
    const CWnd* focus = GetFocus();
    return focus != nullptr && (focus == &_edit || _edit.IsChild(focus));
}

CString CCommandPane::trimmedText() const
{
    CString text;
    if (_edit.GetSafeHwnd() != nullptr)
        _edit.GetWindowText(text);
    text.Trim();
    return text;
}

void CCommandPane::clear()
{
    if (_edit.GetSafeHwnd() != nullptr)
        _edit.SetWindowText(_T(""));
}
