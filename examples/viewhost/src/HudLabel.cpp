#include "HudLabel.h"

#include <afxcontrolbars.h>
#include <uxtheme.h>

#pragma comment(lib, "uxtheme.lib")

BEGIN_MESSAGE_MAP(CHudLabel, CStatic)
    ON_WM_ERASEBKGND()
    ON_WM_PAINT()
END_MESSAGE_MAP()

void CHudLabel::PreSubclassWindow()
{
    CStatic::PreSubclassWindow();
    GetWindowText(_text);
}

void CHudLabel::setText(const CString& text)
{
    if (text == _text)
        return;
    _text = text;
    if (GetSafeHwnd() != nullptr)
        Invalidate(FALSE);
}

BOOL CHudLabel::OnEraseBkgnd(CDC*)
{
    return TRUE;
}

void CHudLabel::OnPaint()
{
    CPaintDC paintDC(this);
    CMemDC memDC(paintDC, this);
    CDC& dc = memDC.GetDC();

    CRect rc;
    GetClientRect(&rc);
    if (DrawThemeParentBackground(GetSafeHwnd(), dc.GetSafeHdc(), &rc) != S_OK)
        dc.FillSolidRect(rc, ::GetSysColor(COLOR_BTNFACE));

    dc.SetBkMode(TRANSPARENT);
    dc.SetTextColor(::GetSysColor(COLOR_BTNTEXT));
    CFont* font = GetFont();
    CFont* oldFont = font != nullptr ? dc.SelectObject(font) : nullptr;
    dc.DrawText(_text, rc, DT_LEFT | DT_VCENTER | DT_SINGLELINE | DT_NOPREFIX | DT_END_ELLIPSIS);
    if (oldFont != nullptr)
        dc.SelectObject(oldFont);
}
