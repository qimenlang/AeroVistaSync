#pragma once

#include <afxwin.h>

/// 仪表盘用静态文本：不走 CStatic SetWindowText 的擦背景，离屏画完再上屏。
class CHudLabel : public CStatic
{
public:
    void setText(const CString& text);

protected:
    void PreSubclassWindow() override;
    afx_msg BOOL OnEraseBkgnd(CDC* dc);
    afx_msg void OnPaint();

    DECLARE_MESSAGE_MAP()

private:
    CString _text;
};
