#include "PlatformView.h"

#include "EntityPropDlg.h"
#include "PlatformFrame.h"

#include <aerovista/sync/CigiWire.h>
#include <aerovista/sync/SyncConfig.h>

#include "CigiEntityPositionCtrlV4.h"
#include "CigiSymbolTextDefV4.h"

#include <atlconv.h>

#include <chrono>
#include <cstdint>
#include <string>

BEGIN_MESSAGE_MAP(CPlatformView, CFormView)
    ON_WM_TIMER()
    ON_WM_DESTROY()
    ON_MESSAGE(wmRefreshEntityTree, &CPlatformView::OnRefreshEntityTree)
    ON_MESSAGE(wmOpenEntityProperties, &CPlatformView::OnOpenEntityProperties)
END_MESSAGE_MAP()

IMPLEMENT_DYNCREATE(CPlatformView, CFormView)

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

    constexpr UINT kFocusSinkId = 4096;

    bool isReturnKey(const MSG* pMsg)
    {
        return (pMsg->message == WM_KEYDOWN || pMsg->message == WM_CHAR) && pMsg->wParam == VK_RETURN;
    }

    std::string cstringToUtf8(const CString& text)
    {
        return std::string(CT2A(text, CP_UTF8));
    }
} // namespace

CPlatformView::CPlatformView() : CFormView(IDD_PLATFORM_DIALOG)
{
}

void CPlatformView::DoDataExchange(CDataExchange* dx)
{
    CFormView::DoDataExchange(dx);
    DDX_Control(dx, IDC_STATUS_READY, _statusReady);
    DDX_Control(dx, IDC_EYE_LAT, _eyeLat);
}

CPlatformFrame* CPlatformView::hostFrame() const
{
    return DYNAMIC_DOWNCAST(CPlatformFrame, GetParentFrame());
}

BOOL CPlatformView::handleHostInput(MSG* pMsg)
{
    if (isReturnKey(pMsg) && commandEditHasFocus())
    {
        submitCommand();
        return TRUE;
    }
    // 控眼点时吞掉方向键/WASD，避免焦点仍在树上时 TreeView 也响应方向键。
    if (_controlling && !commandEditHasFocus() && pMsg->message == WM_KEYDOWN &&
        isCameraControlKey(pMsg->wParam))
        return TRUE;
    // NM_CLICK 点在树空白处常常不来；在按下时按 HitTest 决定进入/退出。
    if (pMsg->message == WM_LBUTTONDOWN && applyEyeControlFromCursor(pMsg->hwnd))
        return TRUE;
    return FALSE;
}

BOOL CPlatformView::PreTranslateMessage(MSG* pMsg)
{
    if (handleHostInput(pMsg))
        return TRUE;
    return CFormView::PreTranslateMessage(pMsg);
}

void CPlatformView::OnInitialUpdate()
{
    CFormView::OnInitialUpdate();
    ModifyStyle(0, WS_CLIPCHILDREN);
    if (CFrameWnd* frame = GetParentFrame())
        frame->RecalcLayout();

    if (!loadConfig())
    {
        AfxMessageBox(_T("加载 platform.json 失败，程序退出"));
        closeFrame();
        return;
    }

    // 初始眼点：alt=3m，位于模型群中心南 25m，朝北（yaw=0）水平看模型群。
    // 模型群（engine viewhost_ig_*.json）：center + 东西南北各 3m（alt=0）。
    _eye.x = 39.908475; // lat：center(39.9087) 南 25m
    _eye.y = 116.397500; // lon：与 center 同经度
    _eye.z = 3.0;        // alt=3m
    _eye.yawDeg = 0.0;   // 朝北
    _eye.pitchDeg = 0.0; // 水平
    _eye.rollDeg = 0.0;

    _startTime = std::chrono::steady_clock::now();
    SetTimer(kTimerId, kTimerPeriodMs, nullptr);

    createFocusSink();
    refreshEntityTree();

    subscribeIgPackets();
    updateStatusText();
}

bool CPlatformView::loadConfig()
{
    aerovista::sync::HostConfig host;
    std::string error;
    if (!aerovista::sync::loadHostConfig("platform.json", host, &error))
        return false;
    if (!_host.initialize(host))
        return false;
    _host.run();
    if (!_data.loadEntityCatalog("entities.json", &error))
        AfxMessageBox(_T("加载 entities.json 失败，实体树为空"));
    return true;
}

void CPlatformView::OnTimer(UINT_PTR nIDEvent)
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

    if (_controlling && !commandEditHasFocus())
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

        aerovista::platform::applyManualStep(_eye, dFwd, dRight, dUp, dyaw, dpitch);
    }

    auto& omsg = _host.outMsgWithIgCtrlUdp();
    aerovista::sync::cigi_wire::appendEye(omsg, &_eye);
    _host.flushUdp();
    _host.drainIncoming();
    updateStatusText();
}

void CPlatformView::OnDestroy()
{
    KillTimer(kTimerId);
    _host.shutdown();
    CFormView::OnDestroy();
}

void CPlatformView::createFocusSink()
{
    // 必须 WS_VISIBLE：隐藏窗不能持焦点。放到客户区外，避免看见插入符。
    _focusSink.Create(WS_CHILD | WS_VISIBLE | ES_READONLY, CRect(-4, -4, -2, -2), this, kFocusSinkId);
}

void CPlatformView::defocusEntityTree()
{
    if (CPlatformFrame* frame = hostFrame())
        frame->sceneTreePane().clearSelection();
    if (_focusSink.GetSafeHwnd() != nullptr)
        _focusSink.SetFocus();
}

bool CPlatformView::isDialogChrome(HWND clickHwnd) const
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

bool CPlatformView::applyEyeControlFromCursor(HWND clickHwnd)
{
    CPlatformFrame* frame = hostFrame();
    if (frame != nullptr)
    {
        CSceneTreePane& treePane = frame->sceneTreePane();
        // 点在树上时进入/退出已写好；false 只表示「不吞点击」，不是「没处理」。
        if (treePane.isTreeHwnd(clickHwnd))
            return treePane.handleEyeControlClick(clickHwnd, *this);
    }

    setEyeControlling(false);
    if (!isDialogChrome(clickHwnd))
        return false;
    defocusEntityTree();
    return true;
}

void CPlatformView::setEyeControlling(bool controlling)
{
    _controlling = controlling;
    if (CPlatformFrame* frame = hostFrame())
        frame->sceneTreePane().applyEyeControlling(controlling);
}

void CPlatformView::refreshEntityTree()
{
    if (CPlatformFrame* frame = hostFrame())
        frame->sceneTreePane().rebuild(_data.entitySnapshot(), _controlling);
}

void CPlatformView::openEntityProperties(std::uint16_t entityId)
{
    CEntityPropDlg dlg(_host, _data, entityId, this);
    dlg.DoModal();
    refreshEntityTree();
}

LRESULT CPlatformView::OnOpenEntityProperties(WPARAM wparam, LPARAM)
{
    openEntityProperties(static_cast<std::uint16_t>(wparam));
    return 0;
}

LRESULT CPlatformView::OnRefreshEntityTree(WPARAM, LPARAM)
{
    refreshEntityTree();
    return 0;
}

void CPlatformView::testTcp()
{
    CigiSymbolTextDefV4 packet("testtcp");
    _host.outMsgWithIgCtrlTcp() << packet;
    _host.flushTcp();
    _lastTestName = "TCP CigiSymbolTextDefV4";
    updateStatusText(true);
}

void CPlatformView::testUdp()
{
    CigiEntityPositionCtrlV4 packet;
    packet.SetEntityID(7);
    _host.outMsgWithIgCtrlUdp() << packet;
    _host.flushUdp();
    _lastTestName = "UDP CigiEntityPositionCtrlV4";
    updateStatusText(true);
}

void CPlatformView::closeFrame()
{
    if (CFrameWnd* frame = GetParentFrame())
        frame->PostMessage(WM_CLOSE);
}

bool CPlatformView::commandEditHasFocus() const
{
    CPlatformFrame* frame = hostFrame();
    return frame != nullptr && frame->commandPane().hasFocus();
}

void CPlatformView::submitCommand()
{
    CPlatformFrame* frame = hostFrame();
    if (frame == nullptr)
        return;

    const CString text = frame->commandPane().trimmedText();
    if (text.IsEmpty())
        return;

    const std::string utf8 = cstringToUtf8(text);
    if (utf8.empty())
        return;

    CigiSymbolTextDefV4 packet(utf8.c_str());
    _host.outMsgWithIgCtrlTcp() << packet;
    _host.flushTcp();

    frame->commandPane().clear();
    _lastTestName = "TCP CigiSymbolTextDefV4";
    updateStatusText(true);
}

void CPlatformView::updateStatusText(bool force)
{
    const int ready = _host.readyIgCount();
    const auto now = std::chrono::steady_clock::now();
    // 数据面仍 60fps；仪表盘文案 ~10Hz，由 CHudLabel 离屏绘制，避免 CStatic 擦背景闪烁。
    if (!force && ready == _lastReadyIgShown &&
        now - _lastStatusHud < std::chrono::milliseconds(kStatusHudPeriodMs))
        return;

    _lastStatusHud = now;
    _lastReadyIgShown = ready;

    CString conn;
    conn.Format(_T("Ready IG: %d    IGCtrl 发送: %u    SOF 接收: %u"), ready, _host.igCtrlSentCount(),
                _host.sofReceivedCount());
    _statusReady.setText(conn);

    CString eye;
    eye.Format(_T("lat: %.6f    lon: %.6f    alt: %.1f    yaw: %.2f    pitch: %.2f    roll: %.2f"),
               _eye.x, _eye.y, _eye.z, _eye.yawDeg, _eye.pitchDeg, _eye.rollDeg);
    _eyeLat.setText(eye);

    CPlatformFrame* frame = hostFrame();
    if (frame == nullptr)
        return;

    CString probe;
    probe.Format(_T("测试: %hs    接收: %hs"), _lastTestName.empty() ? "-" : _lastTestName.c_str(),
                 _lastRecvName.empty() ? "-" : _lastRecvName.c_str());
    frame->packetProbePane().setStatus(probe);
    frame->igListPane().refresh(_host.igSnapshot());
}

void CPlatformView::subscribeIgPackets()
{
    // IG→Host TCP 上行报文自检：F9 随机发送，Host 侧 subscribe 收到即刷新「接收」。
    // 与 engine PacketProbeHandler 的 kTcpProbes 16 类一一对应（HostSync registerTcpProcessors 已注册）。
    _host.addCallback<CigiIGMsgV4>([this](const CigiIGMsgV4&) { _lastRecvName = "CigiIGMsgV4"; });
    _host.addCallback<CigiEventNotificationV4>([this](const CigiEventNotificationV4&) { _lastRecvName = "CigiEventNotificationV4"; });
    _host.addCallback<CigiAnimationStopV4>([this](const CigiAnimationStopV4&) { _lastRecvName = "CigiAnimationStopV4"; });
    _host.addCallback<CigiHatHotRespV4>([this](const CigiHatHotRespV4&) { _lastRecvName = "CigiHatHotRespV4"; });
    _host.addCallback<CigiHatHotXRespV4>([this](const CigiHatHotXRespV4&) { _lastRecvName = "CigiHatHotXRespV4"; });
    _host.addCallback<CigiLosRespV4>([this](const CigiLosRespV4&) { _lastRecvName = "CigiLosRespV4"; });
    _host.addCallback<CigiLosXRespV4>([this](const CigiLosXRespV4&) { _lastRecvName = "CigiLosXRespV4"; });
    _host.addCallback<CigiSensorRespV4>([this](const CigiSensorRespV4&) { _lastRecvName = "CigiSensorRespV4"; });
    _host.addCallback<CigiSensorXRespV4>([this](const CigiSensorXRespV4&) { _lastRecvName = "CigiSensorXRespV4"; });
    _host.addCallback<CigiPositionRespV4>([this](const CigiPositionRespV4&) { _lastRecvName = "CigiPositionRespV4"; });
    _host.addCallback<CigiWeatherCondRespV4>([this](const CigiWeatherCondRespV4&) { _lastRecvName = "CigiWeatherCondRespV4"; });
    _host.addCallback<CigiAerosolRespV4>([this](const CigiAerosolRespV4&) { _lastRecvName = "CigiAerosolRespV4"; });
    _host.addCallback<CigiMaritimeSurfaceRespV4>([this](const CigiMaritimeSurfaceRespV4&) { _lastRecvName = "CigiMaritimeSurfaceRespV4"; });
    _host.addCallback<CigiTerrestrialSurfaceRespV4>([this](const CigiTerrestrialSurfaceRespV4&) { _lastRecvName = "CigiTerrestrialSurfaceRespV4"; });
    _host.addCallback<CigiCollDetSegRespV4>([this](const CigiCollDetSegRespV4&) { _lastRecvName = "CigiCollDetSegRespV4"; });
    _host.addCallback<CigiCollDetVolRespV4>([this](const CigiCollDetVolRespV4&) { _lastRecvName = "CigiCollDetVolRespV4"; });
}
