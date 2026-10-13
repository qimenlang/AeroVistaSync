#pragma once

#include <afxcmn.h>
#include <afxcontrolbars.h>

#include <aerovista/sync/HostSync.h>

#include <vector>

class CIgListPane : public CDockablePane
{
public:
    void refresh(const std::vector<aerovista::sync::IgConnection>& rows);

protected:
    afx_msg int OnCreate(LPCREATESTRUCT createStruct);
    afx_msg void OnSize(UINT type, int cx, int cy);

    DECLARE_MESSAGE_MAP()

private:
    void adjustLayout();
    void setupColumns();

    CListCtrl _list;
    std::vector<aerovista::sync::IgConnection> _snapshot;
    bool _columnsReady = false;
};
