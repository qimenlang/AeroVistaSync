#include "ViewHostDlg.h"

#include <aerovista/sync/SyncConfig.h>

#include <atlconv.h>

#include <sstream>
#include <string>

BEGIN_MESSAGE_MAP(CViewHostDlg, CDialog)
    ON_WM_TIMER()
    ON_WM_DESTROY()
    ON_BN_CLICKED(IDC_TOGGLE_CONTROL, &CViewHostDlg::OnToggleControl)
    ON_BN_CLICKED(IDC_ENTITY_PLACE, &CViewHostDlg::OnPlaceEntity)
    ON_BN_CLICKED(IDC_EXIT, &CViewHostDlg::OnExit)
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
    _eye.frame = aerovista::sync::cigi_wire::EyeFrame::LLA;
    _eye.x = 39.908475; // lat：center(39.9087) 南 25m
    _eye.y = 116.397500; // lon：与 center 同经度
    _eye.z = 3.0;        // alt=3m
    _eye.yawDeg = 0.0;   // 朝北
    _eye.pitchDeg = 0.0; // 水平
    _eye.rollDeg = 0.0;

    _startTime = std::chrono::steady_clock::now();
    _started = true;
    SetTimer(kTimerId, 16, nullptr);

    // 实体摆放默认值（模型群中心，viewhost_ig_*.json 实体所在区域）。
    SetDlgItemText(IDC_ENTITY_ID, _T("7"));
    SetDlgItemText(IDC_ENTITY_LAT, _T("39.908700"));
    SetDlgItemText(IDC_ENTITY_LON, _T("116.397500"));
    SetDlgItemText(IDC_ENTITY_ALT, _T("0.0"));
    SetDlgItemText(IDC_ENTITY_YPR, _T("0.0 0.0 0.0"));

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

void CViewHostDlg::OnPlaceEntity()
{
    CString idText, latText, lonText, altText, yprText;
    GetDlgItemText(IDC_ENTITY_ID, idText);
    GetDlgItemText(IDC_ENTITY_LAT, latText);
    GetDlgItemText(IDC_ENTITY_LON, lonText);
    GetDlgItemText(IDC_ENTITY_ALT, altText);
    GetDlgItemText(IDC_ENTITY_YPR, yprText);

    // yaw pitch roll 空格分隔解析；缺省为 0。
    double yaw = 0.0, pitch = 0.0, roll = 0.0;
    const std::string yprA = CT2A(yprText.GetString()).m_psz;
    std::istringstream ypr(yprA);
    ypr >> yaw;
    ypr >> pitch;
    ypr >> roll;

    const int entityId = _ttoi(idText);
    const double lat = _ttof(latText);
    const double lon = _ttof(lonText);
    const double alt = _ttof(altText);
    if (entityId <= 0)
    {
        SetDlgItemText(IDC_STATUS_PLACE, _T("最近摆放: Entity ID 无效"));
        return;
    }

    // 命令面（TCP）一次性摆放：绝对 LLA（Detach）。
    _driver.sendEntityPose(static_cast<std::uint16_t>(entityId), lat, lon, alt, yaw, pitch, roll);

    CString status;
    status.Format(_T("最近摆放: id=%d lat=%.6f lon=%.6f alt=%.1f ypr=%.2f/%.2f/%.2f"), entityId, lat, lon, alt, yaw,
                  pitch, roll);
    SetDlgItemText(IDC_STATUS_PLACE, status);
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

    setText(IDC_TOGGLE_CONTROL, _controlling ? _T("停止控制") : _T("开始控制"));
}
