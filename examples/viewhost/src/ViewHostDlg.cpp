#include "ViewHostDlg.h"

#include "EntityPropDlg.h"

#include <aerovista/sync/SyncConfig.h>

#include <atlconv.h>

#include <cstdint>
#include <string>

BEGIN_MESSAGE_MAP(CViewHostDlg, CDialog)
    ON_WM_TIMER()
    ON_WM_DESTROY()
    ON_BN_CLICKED(IDC_TOGGLE_CONTROL, &CViewHostDlg::OnToggleControl)
    ON_BN_CLICKED(IDC_TEST_TCP, &CViewHostDlg::OnTestTcp)
    ON_BN_CLICKED(IDC_TEST_UDP, &CViewHostDlg::OnTestUdp)
    ON_BN_CLICKED(IDC_EXIT, &CViewHostDlg::OnExit)
    ON_NOTIFY(NM_DBLCLK, IDC_ENTITY_TREE, &CViewHostDlg::OnEntityTreeDblClk)
    ON_MESSAGE(WM_APP + 20, &CViewHostDlg::OnRefreshEntityTree)
    ON_MESSAGE(WM_APP + 21, &CViewHostDlg::OnOpenEntityProperties)
END_MESSAGE_MAP()

CViewHostDlg::CViewHostDlg(CWnd* pParent) : CDialog(IDD_VIEWHOST_DIALOG, pParent)
{
}

BOOL CViewHostDlg::PreTranslateMessage(MSG* pMsg)
{
    // 空格键切换「开始控制 / 停止控制」。
    // 不用 WM_KEYDOWN 消息映射是因为焦点在子控件上时按键不路由到对话框；
    // PreTranslateMessage 在派发前拦截，对话框内全局生效（viewhost设计.md §4.5 三个坑之一）。
    if (pMsg->message == WM_KEYDOWN && pMsg->wParam == VK_SPACE)
    {
        OnToggleControl();
        return TRUE; // 吞掉该消息，避免空格还被其它机制消费
    }
    return CDialog::PreTranslateMessage(pMsg);
}

BOOL CViewHostDlg::OnInitDialog()
{
    CDialog::OnInitDialog();

    if (!loadConfig())
    {
        AfxMessageBox(_T("加载 viewhost.json 失败，程序退出"));
        EndDialog(IDCANCEL);
        return TRUE;
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
    _started = true;
    SetTimer(kTimerId, 16, nullptr);

    _entityTree.SubclassDlgItem(IDC_ENTITY_TREE, this);
    refreshEntityTree();

    subscribeIgPackets();
    updateStatusText();
    return TRUE;
}

bool CViewHostDlg::loadConfig()
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

void CViewHostDlg::OnTimer(UINT_PTR nIDEvent)
{
    if (nIDEvent != kTimerId)
    {
        CDialog::OnTimer(nIDEvent);
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

        if (GetAsyncKeyState('W') & 0x8000)
            dFwd += moveStep;
        if (GetAsyncKeyState('S') & 0x8000)
            dFwd -= moveStep;
        if (GetAsyncKeyState('A') & 0x8000)
            dRight -= moveStep;
        if (GetAsyncKeyState('D') & 0x8000)
            dRight += moveStep;
        if (GetAsyncKeyState('E') & 0x8000)
            dUp += moveStep;
        if (GetAsyncKeyState('C') & 0x8000)
            dUp -= moveStep;

        if (GetAsyncKeyState(VK_LEFT) & 0x8000)
            dyaw += turnStepDeg;
        if (GetAsyncKeyState(VK_RIGHT) & 0x8000)
            dyaw -= turnStepDeg;
        if (GetAsyncKeyState(VK_UP) & 0x8000)
            dpitch += turnStepDeg;
        if (GetAsyncKeyState(VK_DOWN) & 0x8000)
            dpitch -= turnStepDeg;

        aerovista::viewhost::applyManualStep(_eye, dFwd, dRight, dUp, dyaw, dpitch);
    }

    _driver.update(&_eye);
    _driver.pollIncoming(); // Host push 收包：drain 并解包 IG→Host 报文，触发订阅回调（§8.1）。
    updateStatusText();
}

void CViewHostDlg::OnDestroy()
{
    KillTimer(kTimerId);
    _driver.shutdown();
    CDialog::OnDestroy();
}

void CViewHostDlg::OnToggleControl()
{
    _controlling = !_controlling;
    updateStatusText();
}

void CViewHostDlg::refreshEntityTree()
{
    if (_entityTree.GetSafeHwnd() == nullptr)
        return;

    _entityTree.DeleteAllItems();
    const HTREEITEM root = _entityTree.InsertItem(_T("entities"));
    _entityTree.SetItemData(root, 0);
    for (const aerovista::sync::EntityAuthorityRow& row : _driver.entitySnapshot())
    {
        const CString name(CA2T(row.name.c_str(), CP_UTF8));
        const HTREEITEM item = _entityTree.InsertItem(name, root);
        _entityTree.SetItemData(item, row.entityId);
    }
    _entityTree.Expand(root, TVE_EXPAND);
}

void CViewHostDlg::OnEntityTreeDblClk(NMHDR*, LRESULT* result)
{
    *result = 0; // 根节点仍走默认展开/折叠
    CPoint screen;
    GetCursorPos(&screen);
    CPoint client = screen;
    _entityTree.ScreenToClient(&client);
    UINT flags = 0;
    const HTREEITEM item = _entityTree.HitTest(client, &flags);
    if (item == nullptr || item == _entityTree.GetRootItem())
        return;

    *result = TRUE; // 叶子：不要再走默认展开
    _entityTree.SelectItem(item);
    // 通知返回后再弹模态框，避免双击的 mouse-up 落到面板按钮上。
    PostMessage(WM_APP + 21, _entityTree.GetItemData(item));
}

void CViewHostDlg::openEntityProperties(std::uint16_t entityId)
{
    CEntityPropDlg dlg(_driver, entityId, this);
    dlg.DoModal();
    refreshEntityTree();
}

LRESULT CViewHostDlg::OnOpenEntityProperties(WPARAM wparam, LPARAM)
{
    openEntityProperties(static_cast<std::uint16_t>(wparam));
    return 0;
}

LRESULT CViewHostDlg::OnRefreshEntityTree(WPARAM, LPARAM)
{
    refreshEntityTree();
    return 0;
}

void CViewHostDlg::OnTestTcp()
{
    const std::string name = _driver.sendRandomTcpPacket();
    CString status;
    status.Format(_T("最近测试: TCP %hs"), name.c_str());
    SetDlgItemText(IDC_STATUS_TEST, status);
}

void CViewHostDlg::OnTestUdp()
{
    const std::string name = _driver.sendRandomUdpPacket();
    CString status;
    status.Format(_T("最近测试: UDP %hs"), name.c_str());
    SetDlgItemText(IDC_STATUS_TEST, status);
}

void CViewHostDlg::OnExit()
{
    EndDialog(IDCANCEL);
}

void CViewHostDlg::updateStatusText()
{
    auto setText = [this](int id, const CString& text)
    {
        if (CWnd* wnd = GetDlgItem(id))
            wnd->SetWindowText(text);
    };

    CString ready;
    ready.Format(_T("Ready IG: %d"), _driver.readyIgCount());
    setText(IDC_STATUS_READY, ready);

    CString ctrl;
    ctrl.Format(_T("IGCtrl 发送: %u"), _driver.igCtrlSentCount());
    setText(IDC_STATUS_IGCTRL, ctrl);

    CString sof;
    sof.Format(_T("SOF 接收: %u"), _driver.sofReceivedCount());
    setText(IDC_STATUS_SOF, sof);

    CString lat, lon, alt, ypr;
    lat.Format(_T("lat: %.6f"), _eye.x);
    lon.Format(_T("lon: %.6f"), _eye.y);
    alt.Format(_T("alt: %.1f"), _eye.z);
    ypr.Format(_T("yaw: %.2f  pitch: %.2f  roll: %.2f"), _eye.yawDeg, _eye.pitchDeg, _eye.rollDeg);
    setText(IDC_EYE_LAT, lat);
    setText(IDC_EYE_LON, lon);
    setText(IDC_EYE_ALT, alt);
    setText(IDC_EYE_YPR, ypr);

    if (!_lastRecvName.empty())
    {
        CString recv;
        recv.Format(_T("最近接收: %hs"), _lastRecvName.c_str());
        setText(IDC_STATUS_RECV, recv);
    }

    setText(IDC_TOGGLE_CONTROL, _controlling ? _T("停止控制") : _T("开始控制"));
}

void CViewHostDlg::subscribeIgPackets()
{
    // IG→Host TCP 上行报文自检（§4.7）：F9 随机发送，Host 侧 subscribe 收到即刷新「最近接收」。
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
