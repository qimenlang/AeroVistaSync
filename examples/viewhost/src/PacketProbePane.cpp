#include "PacketProbePane.h"

#include "ViewHostView.h"
#include "resource.h"

BEGIN_MESSAGE_MAP(CPacketProbePane, CDockablePane)
    ON_WM_CREATE()
    ON_WM_SIZE()
    ON_BN_CLICKED(IDC_TEST_TCP, &CPacketProbePane::OnTestTcp)
    ON_BN_CLICKED(IDC_TEST_UDP, &CPacketProbePane::OnTestUdp)
END_MESSAGE_MAP()

int CPacketProbePane::OnCreate(LPCREATESTRUCT createStruct)
{
    if (CDockablePane::OnCreate(createStruct) == -1)
        return -1;

    constexpr DWORD kButton = WS_CHILD | WS_VISIBLE | WS_TABSTOP | BS_PUSHBUTTON;
    constexpr DWORD kLabel = WS_CHILD | WS_VISIBLE | SS_LEFT;
    if (!_testTcp.Create(_T("testtcp"), kButton, CRect(0, 0, 50, 22), this, IDC_TEST_TCP))
        return -1;
    if (!_testUdp.Create(_T("testudp"), kButton, CRect(52, 0, 102, 22), this, IDC_TEST_UDP))
        return -1;
    if (!_status.Create(_T("测试: -    接收: -"), kLabel, CRect(108, 4, 280, 20), this, IDC_STATUS_TEST))
        return -1;

    _testTcp.SetFont(&afxGlobalData.fontRegular);
    _testUdp.SetFont(&afxGlobalData.fontRegular);
    _status.SetFont(&afxGlobalData.fontRegular);
    adjustLayout();
    return 0;
}

void CPacketProbePane::OnSize(UINT type, int cx, int cy)
{
    CDockablePane::OnSize(type, cx, cy);
    adjustLayout();
}

void CPacketProbePane::OnUpdateCmdUI(CFrameWnd* target, int)
{
    // CDockablePane 默认 UpdateDialogControls(..., TRUE)：子按钮在 Frame 上没有
    // ON_UPDATE_COMMAND_UI 就会被 idle 置灰。testtcp/testudp 只在本 Pane 点，不要禁。
    CDockablePane::OnUpdateCmdUI(target, FALSE);
}

void CPacketProbePane::adjustLayout()
{
    if (_testTcp.GetSafeHwnd() == nullptr)
        return;

    CRect client;
    GetClientRect(&client);
    const int pad = 8;
    const int btnW = 50;
    const int btnH = 22;
    const int gap = 6;
    _testTcp.SetWindowPos(nullptr, pad, pad, btnW, btnH, SWP_NOZORDER);
    _testUdp.SetWindowPos(nullptr, pad + btnW + gap, pad, btnW, btnH, SWP_NOZORDER);
    const int statusX = pad + (btnW + gap) * 2;
    int statusWidth = client.Width() - statusX - pad;
    if (statusWidth < 0)
        statusWidth = 0;
    _status.SetWindowPos(nullptr, statusX, pad + 4, statusWidth, 16, SWP_NOZORDER);
}

void CPacketProbePane::setStatus(const CString& text)
{
    if (_status.GetSafeHwnd() != nullptr)
        _status.SetWindowText(text);
}

void CPacketProbePane::OnTestTcp()
{
    if (CViewHostView* view = viewHostView(this))
        view->testTcp();
}

void CPacketProbePane::OnTestUdp()
{
    if (CViewHostView* view = viewHostView(this))
        view->testUdp();
}
