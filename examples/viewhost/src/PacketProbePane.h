#pragma once

#include <afxcontrolbars.h>
#include <afxwin.h>

class CPacketProbePane : public CDockablePane
{
public:
    void setStatus(const CString& text);

protected:
    afx_msg int OnCreate(LPCREATESTRUCT createStruct);
    afx_msg void OnSize(UINT type, int cx, int cy);
    afx_msg void OnTestTcp();
    afx_msg void OnTestUdp();
    void OnUpdateCmdUI(CFrameWnd* target, int disableIfNoHandler) override;

    DECLARE_MESSAGE_MAP()

private:
    void adjustLayout();

    CButton _testTcp;
    CButton _testUdp;
    CStatic _status;
};
