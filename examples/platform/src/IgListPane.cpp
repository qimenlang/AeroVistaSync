#include "IgListPane.h"

#include "resource.h"

#include <optional>
#include <string>

BEGIN_MESSAGE_MAP(CIgListPane, CDockablePane)
    ON_WM_CREATE()
    ON_WM_SIZE()
END_MESSAGE_MAP()

namespace
{
    const TCHAR* igLinkStatus(const aerovista::sync::IgConnection& row)
    {
        if (row.tcpReady && row.udpReady)
            return _T("ready");
        if (row.tcpReady)
            return _T("tcp");
        if (row.udpReady)
            return _T("udp");
        return _T("connecting");
    }

    CString formatDurationMs(const std::optional<std::chrono::microseconds>& duration)
    {
        if (!duration)
            return _T("--");
        CString text;
        text.Format(_T("%.1f ms"), static_cast<double>(duration->count()) / 1000.0);
        return text;
    }

    CString formatLossRate(const std::optional<double>& loss)
    {
        if (!loss)
            return _T("--");
        CString text;
        text.Format(_T("%.1f%%"), *loss * 100.0);
        return text;
    }

    void writeIgListRow(CListCtrl& list, int index, const aerovista::sync::IgConnection& row)
    {
        CString id;
        id.Format(_T("%llu"), static_cast<unsigned long long>(row.id));
        list.SetItemText(index, 0, id);
        list.SetItemText(index, 1, igLinkStatus(row));
        list.SetItemText(index, 2, formatDurationMs(row.avgRtt));
        list.SetItemText(index, 3, formatDurationMs(row.lastRtt));
        list.SetItemText(index, 4, formatLossRate(row.lossRate));
        list.SetItemText(index, 5, formatDurationMs(row.sofAge));
    }
} // namespace

int CIgListPane::OnCreate(LPCREATESTRUCT createStruct)
{
    if (CDockablePane::OnCreate(createStruct) == -1)
        return -1;

    CRect client;
    GetClientRect(&client);
    constexpr DWORD kListStyle =
        WS_CHILD | WS_VISIBLE | WS_BORDER | WS_TABSTOP | LVS_REPORT | LVS_SINGLESEL | LVS_SHOWSELALWAYS | LVS_NOSORTHEADER;
    if (!_list.Create(kListStyle, client, this, IDC_IG_LIST))
        return -1;
    _list.SetExtendedStyle(LVS_EX_FULLROWSELECT | LVS_EX_DOUBLEBUFFER);
    _list.SetFont(&afxGlobalData.fontRegular);
    adjustLayout();
    return 0;
}

void CIgListPane::OnSize(UINT type, int cx, int cy)
{
    CDockablePane::OnSize(type, cx, cy);
    adjustLayout();
}

void CIgListPane::adjustLayout()
{
    if (_list.GetSafeHwnd() == nullptr)
        return;

    CRect client;
    GetClientRect(&client);
    _list.SetWindowPos(nullptr, 0, 0, client.Width(), client.Height(), SWP_NOZORDER);
    setupColumns();
}

void CIgListPane::setupColumns()
{
    CRect client;
    _list.GetClientRect(&client);
    const int width = client.Width();
    if (width <= 0)
        return;

    const int idWidth = width * 10 / 100;
    const int statusWidth = width * 14 / 100;
    const int avgWidth = width * 16 / 100;
    const int lastWidth = width * 16 / 100;
    const int lossWidth = width * 14 / 100;
    const int sofWidth = width - idWidth - statusWidth - avgWidth - lastWidth - lossWidth - 4;
    if (!_columnsReady)
    {
        _list.InsertColumn(0, _T("id"), LVCFMT_LEFT, idWidth);
        _list.InsertColumn(1, _T("状态"), LVCFMT_LEFT, statusWidth);
        _list.InsertColumn(2, _T("平均RTT"), LVCFMT_LEFT, avgWidth);
        _list.InsertColumn(3, _T("最近RTT"), LVCFMT_LEFT, lastWidth);
        _list.InsertColumn(4, _T("丢包率"), LVCFMT_LEFT, lossWidth);
        _list.InsertColumn(5, _T("距上次SOF"), LVCFMT_LEFT, sofWidth);
        _columnsReady = true;
        return;
    }

    _list.SetColumnWidth(0, idWidth);
    _list.SetColumnWidth(1, statusWidth);
    _list.SetColumnWidth(2, avgWidth);
    _list.SetColumnWidth(3, lastWidth);
    _list.SetColumnWidth(4, lossWidth);
    _list.SetColumnWidth(5, sofWidth);
}

void CIgListPane::refresh(const std::vector<aerovista::sync::IgConnection>& rows)
{
    if (_list.GetSafeHwnd() == nullptr)
        return;
    if (rows == _snapshot)
        return;
    _snapshot = rows;

    const int n = static_cast<int>(rows.size());
    if (_list.GetItemCount() != n)
    {
        _list.DeleteAllItems();
        for (int i = 0; i < n; ++i)
            _list.InsertItem(i, _T(""));
    }
    for (int i = 0; i < n; ++i)
        writeIgListRow(_list, i, rows[static_cast<size_t>(i)]);
}
