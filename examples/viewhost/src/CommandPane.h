#pragma once

#include <afxcontrolbars.h>
#include <afxwin.h>

class CCommandPane : public CDockablePane
{
public:
    bool hasFocus() const;
    CString trimmedText() const;
    void clear();

protected:
    afx_msg int OnCreate(LPCREATESTRUCT createStruct);
    afx_msg void OnSize(UINT type, int cx, int cy);

    DECLARE_MESSAGE_MAP()

private:
    void adjustLayout();

    CEdit _edit;
};
