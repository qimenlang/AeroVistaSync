#include "EntityPropDlg.h"

#include <atlconv.h>

#include <optional>
#include <string>

BEGIN_MESSAGE_MAP(CEntityPropDlg, CDialog)
    ON_BN_CLICKED(IDC_PROP_APPLY, &CEntityPropDlg::OnApply)
    ON_BN_CLICKED(IDC_PROP_RESET, &CEntityPropDlg::OnReset)
END_MESSAGE_MAP()

namespace
{
    CString utf8ToCString(const std::string& text)
    {
        return CString(CA2T(text.c_str(), CP_UTF8));
    }

    double dlgItemDouble(CWnd& dlg, int id)
    {
        CString text;
        dlg.GetDlgItemText(id, text);
        return _ttof(text);
    }

    bool nearlyEqual(double a, double b)
    {
        const double d = a - b;
        return d < 1e-9 && d > -1e-9;
    }
} // namespace

CEntityPropDlg::CEntityPropDlg(aerovista::viewhost::HostDriver& driver, std::uint16_t entityId, CWnd* pParent)
    : CDialog(IDD_ENTITY_PROP, pParent), _driver(driver), _entityId(entityId)
{
}

BOOL CEntityPropDlg::OnInitDialog()
{
    CDialog::OnInitDialog();
    if (!loadRow())
    {
        AfxMessageBox(_T("未找到该实体"));
        EndDialog(IDCANCEL);
        return TRUE;
    }
    return TRUE;
}

bool CEntityPropDlg::loadRow()
{
    const std::optional<aerovista::sync::EntityAuthorityRow> row = _driver.entityRow(_entityId);
    if (!row)
        return false;

    CString idText;
    idText.Format(_T("%u"), static_cast<unsigned>(row->entityId));
    SetDlgItemText(IDC_PROP_ID, idText);
    SetDlgItemText(IDC_PROP_NAME, utf8ToCString(row->name));

    auto* stateCombo = static_cast<CComboBox*>(GetDlgItem(IDC_PROP_STATE));
    if (stateCombo && stateCombo->GetCount() == 0)
    {
        stateCombo->AddString(_T("Standby"));
        stateCombo->AddString(_T("Active"));
    }
    if (stateCombo)
        stateCombo->SetCurSel(row->entityState == aerovista::sync::EntityAuthorityState::ACTIVE ? 1 : 0);

    SetDlgItemInt(IDC_PROP_ALPHA, row->alpha, FALSE);

    CString lat, lon, alt, yaw, pitch, roll;
    lat.Format(_T("%.6f"), row->pose.lat);
    lon.Format(_T("%.6f"), row->pose.lon);
    alt.Format(_T("%.3f"), row->pose.alt);
    yaw.Format(_T("%.3f"), row->pose.yawDeg);
    pitch.Format(_T("%.3f"), row->pose.pitchDeg);
    roll.Format(_T("%.3f"), row->pose.rollDeg);
    SetDlgItemText(IDC_PROP_LAT, lat);
    SetDlgItemText(IDC_PROP_LON, lon);
    SetDlgItemText(IDC_PROP_ALT, alt);
    SetDlgItemText(IDC_PROP_YAW, yaw);
    SetDlgItemText(IDC_PROP_PITCH, pitch);
    SetDlgItemText(IDC_PROP_ROLL, roll);

    captureBaseline();
    return true;
}

void CEntityPropDlg::captureBaseline()
{
    const auto row = _driver.entityRow(_entityId);
    if (!row)
        return;
    _baselineState = row->entityState;
    _baselineAlpha = row->alpha;
    _baselinePose = row->pose;
}

bool CEntityPropDlg::applyDirtyFields(std::string& error)
{
    auto* stateCombo = static_cast<CComboBox*>(GetDlgItem(IDC_PROP_STATE));
    const int sel = stateCombo ? stateCombo->GetCurSel() : 0;
    const auto state = (sel == 1) ? aerovista::sync::EntityAuthorityState::ACTIVE
                                  : aerovista::sync::EntityAuthorityState::STANDBY;
    BOOL alphaOk = FALSE;
    const UINT alphaVal = GetDlgItemInt(IDC_PROP_ALPHA, &alphaOk, FALSE);
    if (!alphaOk || alphaVal > 255)
    {
        error = "alpha must be 0..255";
        return false;
    }
    const auto alpha = static_cast<std::uint8_t>(alphaVal);

    const double lat = dlgItemDouble(*this, IDC_PROP_LAT);
    const double lon = dlgItemDouble(*this, IDC_PROP_LON);
    const double alt = dlgItemDouble(*this, IDC_PROP_ALT);
    const double yaw = dlgItemDouble(*this, IDC_PROP_YAW);
    const double pitch = dlgItemDouble(*this, IDC_PROP_PITCH);
    const double roll = dlgItemDouble(*this, IDC_PROP_ROLL);

    const bool ctrlDirty = state != _baselineState || alpha != _baselineAlpha;
    const bool poseDirty = !nearlyEqual(lat, _baselinePose.lat) || !nearlyEqual(lon, _baselinePose.lon) ||
                           !nearlyEqual(alt, _baselinePose.alt) || !nearlyEqual(yaw, _baselinePose.yawDeg) ||
                           !nearlyEqual(pitch, _baselinePose.pitchDeg) || !nearlyEqual(roll, _baselinePose.rollDeg);

    if (ctrlDirty && !_driver.setEntityCtrl(_entityId, state, alpha, &error))
        return false;

    if (poseDirty)
    {
        aerovista::sync::EntityAuthorityPose pose;
        pose.lat = lat;
        pose.lon = lon;
        pose.alt = alt;
        pose.yawDeg = yaw;
        pose.pitchDeg = pitch;
        pose.rollDeg = roll;
        if (!_driver.setEntityPose(_entityId, pose, &error))
            return false;
    }

    aerovista::viewhost::HostDriver::EntitySend send;
    send.entityCtrl = ctrlDirty;
    send.entityPosition = poseDirty;
    if (!ctrlDirty && !poseDirty)
        return true;
    if (!_driver.sendEntity(_entityId, send, &error))
        return false;

    captureBaseline();
    return true;
}

void CEntityPropDlg::OnApply()
{
    std::string error;
    if (!applyDirtyFields(error))
    {
        CString msg;
        msg.Format(_T("Apply 失败: %hs"), error.c_str());
        AfxMessageBox(msg);
        return;
    }
    if (CWnd* parent = GetParent())
        parent->SendMessage(WM_APP + 20);
}

void CEntityPropDlg::OnReset()
{
    if (!loadRow())
        AfxMessageBox(_T("未找到该实体"));
}
