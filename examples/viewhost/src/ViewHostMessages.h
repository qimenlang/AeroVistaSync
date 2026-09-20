#pragma once

#include <afxwin.h>

// 自定义窗口消息（WM_APP 段），不是 resource.h 控件 ID。
// EntityPropDlg 向父窗口 SendMessage，本视图 ON_MESSAGE 接收，收发必须同一常量。
inline constexpr UINT wmRefreshEntityTree = WM_APP + 20;
inline constexpr UINT wmOpenEntityProperties = WM_APP + 21;
