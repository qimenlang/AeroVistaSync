#include "ViewHostView.h"

#include "EntityPropDlg.h"

#include <aerovista/sync/SyncConfig.h>

#include <atlconv.h>

#include <cstdint>
#include <string>

BEGIN_MESSAGE_MAP(CViewHostView, CFormView)
    ON_WM_TIMER()
    ON_WM_DESTROY()
    ON_BN_CLICKED(IDC_TEST_TCP, &CViewHostView::OnTestTcp)
    ON_BN_CLICKED(IDC_TEST_UDP, &CViewHostView::OnTestUdp)
    ON_NOTIFY(NM_DBLCLK, IDC_ENTITY_TREE, &CViewHostView::OnEntityTreeDblClk)
    ON_MESSAGE(wmRefreshEntityTree, &CViewHostView::OnRefreshEntityTree)
    ON_MESSAGE(wmOpenEntityProperties, &CViewHostView::OnOpenEntityProperties)
END_MESSAGE_MAP()

IMPLEMENT_DYNCREATE(CViewHostView, CFormView)

namespace
{
    bool asyncKeyDown(int virtualKey)
    {
        return (GetAsyncKeyState(virtualKey) & 0x8000) != 0;
    }

    bool isCameraControlKey(WPARAM virtualKey)
    {
        switch (virtualKey)
        {
        case 'W':
        case 'A':
        case 'S':
        case 'D':
        case 'E':
        case 'C':
        case VK_LEFT:
        case VK_RIGHT:
        case VK_UP:
        case VK_DOWN:
            return true;
        default:
            return false;
        }
    }

    constexpr UINT kOnItemHit = TVHT_ONITEM | TVHT_ONITEMINDENT | TVHT_ONITEMRIGHT;
    constexpr UINT kEmptyTreeHit = TVHT_NOWHERE | TVHT_ABOVE | TVHT_BELOW | TVHT_TOLEFT | TVHT_TORIGHT;
    constexpr UINT kFocusSinkId = 4096;

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

    void writeIgListRow(CListCtrl& list, int index, const aerovista::sync::IgConnection& row)
    {
        CString id;
        id.Format(_T("%llu"), static_cast<unsigned long long>(row.id));
        list.SetItemText(index, 0, id);
        list.SetItemText(index, 1, igLinkStatus(row));
    }
} // namespace

CViewHostView::CViewHostView() : CFormView(IDD_VIEWHOST_DIALOG)
{
}

BOOL CViewHostView::PreTranslateMessage(MSG* pMsg)
{
    // 控眼点时吞掉方向键/WASD，避免焦点仍在树上时 TreeView 也响应方向键。
    if (_controlling && pMsg->message == WM_KEYDOWN && isCameraControlKey(pMsg->wParam))
        return TRUE;
    // NM_CLICK 点在树空白处常常不来；在按下时按 HitTest 决定进入/退出。
    if (pMsg->message == WM_LBUTTONDOWN && applyEyeControlFromCursor(pMsg->hwnd))
        return TRUE; // 树/对话框空白、静态文本、分组框：吞掉点击，避免焦点弹回树
    return CFormView::PreTranslateMessage(pMsg);
}

void CViewHostView::OnInitialUpdate()
{
    CFormView::OnInitialUpdate();
    if (CFrameWnd* frame = GetParentFrame())
        frame->RecalcLayout();

    if (!loadConfig())
    {
        AfxMessageBox(_T("加载 viewhost.json 失败，程序退出"));
        closeFrame();
        return;
    }

    // 初始眼点：alt=3m，位于模型群中心南 25m，朝北（yaw=0）水平看模型群。
    // 模型群（viewhost_ig_*.json）：center + 东西南北各 3m（alt=0）。
    _eye.x = 39.908475; // lat：center(39.9087) 南 25m
    _eye.y = 116.397500; // lon：与 center 同经度
    _eye.z = 3.0;        // alt=3m
    _eye.yawDeg = 0.0;   // 朝北
    _eye.pitchDeg = 0.0; // 水平
    _eye.rollDeg = 0.0;

    _startTime = std::chrono::steady_clock::now();
    SetTimer(kTimerId, kTimerPeriodMs, nullptr);

    _entityTree.SubclassDlgItem(IDC_ENTITY_TREE, this);
    setupIgList();
    createFocusSink();
    refreshEntityTree();

    subscribeIgPackets();
    updateStatusText();
}

bool CViewHostView::loadConfig()
{
    aerovista::sync::HostConfig host;
    std::string error;
    if (!aerovista::sync::loadHostConfig("viewhost.json", host, &error))
        return false;
    if (!_driver.initialize(host, &error))
        return false;
    if (!_driver.loadEntityCatalog("entities.json", &error))
        AfxMessageBox(_T("加载 entities.json 失败，实体树为空"));
    return true;
}

void CViewHostView::OnTimer(UINT_PTR nIDEvent)
{
    if (nIDEvent != kTimerId)
    {
        CFormView::OnTimer(nIDEvent);
        return;
    }

    const auto now = std::chrono::steady_clock::now();
    const double simTimeMs = std::chrono::duration<double, std::milli>(now - _startTime).count();
    const double dtSec = (simTimeMs - _lastSimTimeMs) / 1000.0;
    _lastSimTimeMs = simTimeMs;

    // 每帧增量按实际 dt 归一化（viewhost设计.md §4.3）。
    const double moveStep = _speed * dtSec;
    const double turnStepDeg = _turnRate * dtSec;

    if (_controlling)
    {
        double dFwd = 0.0, dRight = 0.0, dUp = 0.0;
        double dyaw = 0.0, dpitch = 0.0;

        if (asyncKeyDown('W'))
            dFwd += moveStep;
        if (asyncKeyDown('S'))
            dFwd -= moveStep;
        if (asyncKeyDown('A'))
            dRight -= moveStep;
        if (asyncKeyDown('D'))
            dRight += moveStep;
        if (asyncKeyDown('E'))
            dUp += moveStep;
        if (asyncKeyDown('C'))
            dUp -= moveStep;

        if (asyncKeyDown(VK_LEFT))
            dyaw += turnStepDeg;
        if (asyncKeyDown(VK_RIGHT))
            dyaw -= turnStepDeg;
        if (asyncKeyDown(VK_UP))
            dpitch += turnStepDeg;
        if (asyncKeyDown(VK_DOWN))
            dpitch -= turnStepDeg;

        aerovista::viewhost::applyManualStep(_eye, dFwd, dRight, dUp, dyaw, dpitch);
    }

    _driver.update(&_eye);
    _driver.pollIncoming(); // Host push 收包：drain 并解包 IG→Host 报文，触发订阅回调（状态同步设计初版.md §8.1）。
    updateStatusText();
}

void CViewHostView::OnDestroy()
{
    KillTimer(kTimerId);
    _driver.shutdown();
    CFormView::OnDestroy();
}

CViewHostView::EntityTreeHit CViewHostView::hitTestEntityTree()
{
    EntityTreeHit hit;
    CPoint screen;
    GetCursorPos(&screen);
    hit.client = screen;
    _entityTree.ScreenToClient(&hit.client);
    hit.item = _entityTree.HitTest(hit.client, &hit.flags);
    return hit;
}

bool CViewHostView::isEntityLeaf(HTREEITEM item)
{
    return item != nullptr && _entitiesFolder != nullptr &&
           _entityTree.GetParentItem(item) == _entitiesFolder;
}

void CViewHostView::updateEyePointLabel()
{
    if (_eyePointItem == nullptr)
        return;
    _entityTree.SetItemText(_eyePointItem, _controlling ? _T("eyePoint [控制中]") : _T("eyePoint"));
}

bool CViewHostView::isEyePointHit(HTREEITEM item, UINT flags) const
{
    return item == _eyePointItem && (flags & kOnItemHit) != 0;
}

bool CViewHostView::isEmptyTreeHit(HTREEITEM item, UINT flags) const
{
    return item == nullptr || (flags & kEmptyTreeHit) != 0;
}

void CViewHostView::createFocusSink()
{
    // 必须 WS_VISIBLE：隐藏窗不能持焦点。放到客户区外，避免看见插入符。
    _focusSink.Create(WS_CHILD | WS_VISIBLE | ES_READONLY, CRect(-4, -4, -2, -2), this, kFocusSinkId);
}

void CViewHostView::defocusEntityTree()
{
    _entityTree.SelectItem(nullptr);
    if (_focusSink.GetSafeHwnd() != nullptr)
        _focusSink.SetFocus();
}

bool CViewHostView::isDialogChrome(HWND clickHwnd) const
{
    if (clickHwnd == GetSafeHwnd())
        return true;

    TCHAR className[32]{};
    if (::GetClassName(clickHwnd, className, 32) == 0)
        return false;
    if (lstrcmpi(className, _T("Static")) == 0)
        return true;
    if (lstrcmpi(className, _T("Button")) != 0)
        return false;
    return (::GetWindowLong(clickHwnd, GWL_STYLE) & BS_TYPEMASK) == BS_GROUPBOX;
}

bool CViewHostView::applyEyeControlFromCursor(HWND clickHwnd)
{
    if (_entityTree.GetSafeHwnd() == nullptr)
        return false;

    const HWND treeHwnd = _entityTree.GetSafeHwnd();
    if (clickHwnd != treeHwnd)
    {
        setEyeControlling(false);
        if (!isDialogChrome(clickHwnd))
        {
            _entityTree.SelectItem(nullptr);
            return false; // 按钮等可持焦控件自己抢走键盘
        }
        defocusEntityTree();
        return true; // 分组框内部是对话框客户区；吞掉点击，避免焦点弹回树
    }

    const EntityTreeHit hit = hitTestEntityTree();
    CRect clientRect;
    _entityTree.GetClientRect(&clientRect);
    if (!clientRect.PtInRect(hit.client))
    {
        setEyeControlling(false);
        return false; // 滚动条等非客户区：交给树，保持焦点
    }

    if (isEyePointHit(hit.item, hit.flags))
    {
        setEyeControlling(true);
        return false;
    }

    setEyeControlling(false);
    if (!isEmptyTreeHit(hit.item, hit.flags))
        return false;

    defocusEntityTree();
    return true;
}

void CViewHostView::setEyeControlling(bool controlling)
{
    _controlling = controlling;
    updateEyePointLabel();
    if (controlling && _eyePointItem != nullptr)
        _entityTree.SelectItem(_eyePointItem);
}

void CViewHostView::refreshEntityTree()
{
    if (_entityTree.GetSafeHwnd() == nullptr)
        return;

    const bool controlling = _controlling;
    _entityTree.DeleteAllItems();
    _rootItem = _entityTree.InsertItem(_T("root"));
    _eyePointItem = _entityTree.InsertItem(_T("eyePoint"), _rootItem);
    _entitiesFolder = _entityTree.InsertItem(_T("entities"), _rootItem);
    _entityTree.SetItemData(_rootItem, 0);
    _entityTree.SetItemData(_eyePointItem, 0);
    _entityTree.SetItemData(_entitiesFolder, 0);
    for (const aerovista::sync::EntityAuthorityRow& row : _driver.entitySnapshot())
    {
        const CString name(CA2T(row.name.c_str(), CP_UTF8));
        const HTREEITEM item = _entityTree.InsertItem(name, _entitiesFolder);
        _entityTree.SetItemData(item, row.entityId);
    }
    _entityTree.Expand(_rootItem, TVE_EXPAND);
    _entityTree.Expand(_entitiesFolder, TVE_EXPAND);
    setEyeControlling(controlling);
}

void CViewHostView::OnEntityTreeDblClk(NMHDR*, LRESULT* result)
{
    *result = 0; // root / entities / eyePoint 仍走默认展开/折叠
    const HTREEITEM item = hitTestEntityTree().item;
    if (!isEntityLeaf(item))
        return;

    *result = TRUE; // 实体叶子：不要再走默认展开
    _entityTree.SelectItem(item);
    // 通知返回后再弹模态框，避免双击的 mouse-up 落到面板按钮上。
    PostMessage(wmOpenEntityProperties, _entityTree.GetItemData(item));
}

void CViewHostView::openEntityProperties(std::uint16_t entityId)
{
    CEntityPropDlg dlg(_driver, entityId, this);
    dlg.DoModal();
    refreshEntityTree();
}

LRESULT CViewHostView::OnOpenEntityProperties(WPARAM wparam, LPARAM)
{
    openEntityProperties(static_cast<std::uint16_t>(wparam));
    return 0;
}

LRESULT CViewHostView::OnRefreshEntityTree(WPARAM, LPARAM)
{
    refreshEntityTree();
    return 0;
}

void CViewHostView::OnTestTcp()
{
    _lastTestName = std::string("TCP ") + _driver.sendRandomTcpPacket();
    updateStatusText();
}

void CViewHostView::OnTestUdp()
{
    _lastTestName = std::string("UDP ") + _driver.sendRandomUdpPacket();
    updateStatusText();
}

void CViewHostView::closeFrame()
{
    if (CFrameWnd* frame = GetParentFrame())
        frame->PostMessage(WM_CLOSE);
}

void CViewHostView::setupIgList()
{
    _igList.SubclassDlgItem(IDC_IG_LIST, this);
    _igList.SetExtendedStyle(LVS_EX_FULLROWSELECT | LVS_EX_DOUBLEBUFFER);
    CRect client;
    _igList.GetClientRect(&client);
    const int idWidth = client.Width() / 3;
    _igList.InsertColumn(0, _T("id"), LVCFMT_LEFT, idWidth);
    _igList.InsertColumn(1, _T("状态"), LVCFMT_LEFT, client.Width() - idWidth - 4);
}

void CViewHostView::refreshIgList()
{
    const auto rows = _driver.igSnapshot();
    if (rows == _igSnapshot)
        return;
    _igSnapshot = rows;

    const int n = static_cast<int>(rows.size());
    if (_igList.GetItemCount() != n)
    {
        _igList.DeleteAllItems();
        for (int i = 0; i < n; ++i)
            _igList.InsertItem(i, _T(""));
    }
    for (int i = 0; i < n; ++i)
        writeIgListRow(_igList, i, rows[static_cast<size_t>(i)]);
}

void CViewHostView::updateStatusText()
{
    auto setText = [this](int id, const CString& text)
    {
        if (CWnd* wnd = GetDlgItem(id))
            wnd->SetWindowText(text);
    };

    CString conn;
    conn.Format(_T("Ready IG: %d    IGCtrl 发送: %u    SOF 接收: %u"), _driver.readyIgCount(),
                _driver.igCtrlSentCount(), _driver.sofReceivedCount());
    setText(IDC_STATUS_READY, conn);

    CString eye;
    eye.Format(_T("lat: %.6f    lon: %.6f    alt: %.1f    yaw: %.2f    pitch: %.2f    roll: %.2f"),
               _eye.x, _eye.y, _eye.z, _eye.yawDeg, _eye.pitchDeg, _eye.rollDeg);
    setText(IDC_EYE_LAT, eye);

    CString probe;
    probe.Format(_T("测试: %hs    接收: %hs"), _lastTestName.empty() ? "-" : _lastTestName.c_str(),
                 _lastRecvName.empty() ? "-" : _lastRecvName.c_str());
    setText(IDC_STATUS_TEST, probe);

    refreshIgList();
}

void CViewHostView::subscribeIgPackets()
{
    // IG→Host TCP 上行报文自检（§4.7）：F9 随机发送，Host 侧 subscribe 收到即刷新「接收」。
    // 与 engine PacketProbeHandler 的 kTcpProbes 16 类一一对应（HostSync registerTcpProcessors 已注册）。
    _driver.addCallback<CigiIGMsgV4>([this](const CigiIGMsgV4&) { _lastRecvName = "CigiIGMsgV4"; });
    _driver.addCallback<CigiEventNotificationV4>([this](const CigiEventNotificationV4&) { _lastRecvName = "CigiEventNotificationV4"; });
    _driver.addCallback<CigiAnimationStopV4>([this](const CigiAnimationStopV4&) { _lastRecvName = "CigiAnimationStopV4"; });
    _driver.addCallback<CigiHatHotRespV4>([this](const CigiHatHotRespV4&) { _lastRecvName = "CigiHatHotRespV4"; });
    _driver.addCallback<CigiHatHotXRespV4>([this](const CigiHatHotXRespV4&) { _lastRecvName = "CigiHatHotXRespV4"; });
    _driver.addCallback<CigiLosRespV4>([this](const CigiLosRespV4&) { _lastRecvName = "CigiLosRespV4"; });
    _driver.addCallback<CigiLosXRespV4>([this](const CigiLosXRespV4&) { _lastRecvName = "CigiLosXRespV4"; });
    _driver.addCallback<CigiSensorRespV4>([this](const CigiSensorRespV4&) { _lastRecvName = "CigiSensorRespV4"; });
    _driver.addCallback<CigiSensorXRespV4>([this](const CigiSensorXRespV4&) { _lastRecvName = "CigiSensorXRespV4"; });
    _driver.addCallback<CigiPositionRespV4>([this](const CigiPositionRespV4&) { _lastRecvName = "CigiPositionRespV4"; });
    _driver.addCallback<CigiWeatherCondRespV4>([this](const CigiWeatherCondRespV4&) { _lastRecvName = "CigiWeatherCondRespV4"; });
    _driver.addCallback<CigiAerosolRespV4>([this](const CigiAerosolRespV4&) { _lastRecvName = "CigiAerosolRespV4"; });
    _driver.addCallback<CigiMaritimeSurfaceRespV4>([this](const CigiMaritimeSurfaceRespV4&) { _lastRecvName = "CigiMaritimeSurfaceRespV4"; });
    _driver.addCallback<CigiTerrestrialSurfaceRespV4>([this](const CigiTerrestrialSurfaceRespV4&) { _lastRecvName = "CigiTerrestrialSurfaceRespV4"; });
    _driver.addCallback<CigiCollDetSegRespV4>([this](const CigiCollDetSegRespV4&) { _lastRecvName = "CigiCollDetSegRespV4"; });
    _driver.addCallback<CigiCollDetVolRespV4>([this](const CigiCollDetVolRespV4&) { _lastRecvName = "CigiCollDetVolRespV4"; });
}
