#include "SceneTreePane.h"

#include "ViewHostMessages.h"
#include "ViewHostView.h"
#include "resource.h"

#include <atlconv.h>

BEGIN_MESSAGE_MAP(CSceneTreePane, CDockablePane)
    ON_WM_CREATE()
    ON_WM_SIZE()
    ON_NOTIFY(NM_DBLCLK, IDC_ENTITY_TREE, &CSceneTreePane::OnTreeDblClk)
END_MESSAGE_MAP()

namespace
{
    constexpr UINT kOnItemHit = TVHT_ONITEM | TVHT_ONITEMINDENT | TVHT_ONITEMRIGHT;
    constexpr UINT kEmptyTreeHit = TVHT_NOWHERE | TVHT_ABOVE | TVHT_BELOW | TVHT_TOLEFT | TVHT_TORIGHT;
    constexpr DWORD kTreeStyle = WS_CHILD | WS_VISIBLE | WS_BORDER | WS_TABSTOP | TVS_HASBUTTONS | TVS_HASLINES |
                                 TVS_LINESATROOT | TVS_SHOWSELALWAYS;
}

int CSceneTreePane::OnCreate(LPCREATESTRUCT createStruct)
{
    if (CDockablePane::OnCreate(createStruct) == -1)
        return -1;

    CRect client;
    GetClientRect(&client);
    if (!_tree.Create(kTreeStyle, client, this, IDC_ENTITY_TREE))
        return -1;
    _tree.SetFont(&afxGlobalData.fontRegular);
    adjustLayout();
    return 0;
}

void CSceneTreePane::OnSize(UINT type, int cx, int cy)
{
    CDockablePane::OnSize(type, cx, cy);
    adjustLayout();
}

void CSceneTreePane::adjustLayout()
{
    if (_tree.GetSafeHwnd() == nullptr)
        return;

    CRect client;
    GetClientRect(&client);
    _tree.SetWindowPos(nullptr, 0, 0, client.Width(), client.Height(), SWP_NOZORDER);
}

void CSceneTreePane::rebuild(const std::vector<aerovista::sync::EntityAuthorityRow>& rows, bool controlling)
{
    if (_tree.GetSafeHwnd() == nullptr)
        return;

    _tree.DeleteAllItems();
    _rootItem = _tree.InsertItem(_T("root"));
    _eyePointItem = _tree.InsertItem(_T("eyePoint"), _rootItem);
    _entitiesFolder = _tree.InsertItem(_T("entities"), _rootItem);
    _tree.SetItemData(_rootItem, 0);
    _tree.SetItemData(_eyePointItem, 0);
    _tree.SetItemData(_entitiesFolder, 0);
    for (const aerovista::sync::EntityAuthorityRow& row : rows)
    {
        const CString name(CA2T(row.name.c_str(), CP_UTF8));
        const HTREEITEM item = _tree.InsertItem(name, _entitiesFolder);
        _tree.SetItemData(item, row.entityId);
    }
    _tree.Expand(_rootItem, TVE_EXPAND);
    _tree.Expand(_entitiesFolder, TVE_EXPAND);
    applyEyeControlling(controlling);
}

void CSceneTreePane::applyEyeControlling(bool controlling)
{
    updateEyePointLabel(controlling);
    if (controlling && _eyePointItem != nullptr)
        _tree.SelectItem(_eyePointItem);
}

void CSceneTreePane::clearSelection()
{
    if (_tree.GetSafeHwnd() != nullptr)
        _tree.SelectItem(nullptr);
}

bool CSceneTreePane::isTreeHwnd(HWND hwnd) const
{
    return hwnd != nullptr && hwnd == _tree.GetSafeHwnd();
}

void CSceneTreePane::updateEyePointLabel(bool controlling)
{
    if (_eyePointItem == nullptr)
        return;
    _tree.SetItemText(_eyePointItem, controlling ? _T("eyePoint [控制中]") : _T("eyePoint"));
}

CSceneTreePane::EntityTreeHit CSceneTreePane::hitTestEntityTree()
{
    EntityTreeHit hit;
    CPoint screen;
    GetCursorPos(&screen);
    hit.client = screen;
    _tree.ScreenToClient(&hit.client);
    hit.item = _tree.HitTest(hit.client, &hit.flags);
    return hit;
}

bool CSceneTreePane::isEntityLeaf(HTREEITEM item) const
{
    return item != nullptr && _entitiesFolder != nullptr && _tree.GetParentItem(item) == _entitiesFolder;
}

bool CSceneTreePane::isEyePointHit(HTREEITEM item, UINT flags) const
{
    return item == _eyePointItem && (flags & kOnItemHit) != 0;
}

bool CSceneTreePane::isEmptyTreeHit(HTREEITEM item, UINT flags) const
{
    return item == nullptr || (flags & kEmptyTreeHit) != 0;
}

bool CSceneTreePane::handleEyeControlClick(HWND clickHwnd, CViewHostView& view)
{
    if (_tree.GetSafeHwnd() == nullptr)
        return false;

    const HWND treeHwnd = _tree.GetSafeHwnd();
    if (clickHwnd != treeHwnd)
        return false;

    const EntityTreeHit hit = hitTestEntityTree();
    CRect clientRect;
    _tree.GetClientRect(&clientRect);
    if (!clientRect.PtInRect(hit.client))
    {
        view.setEyeControlling(false);
        return false; // 滚动条等非客户区：交给树，保持焦点
    }

    if (isEyePointHit(hit.item, hit.flags))
    {
        view.setEyeControlling(true);
        return false; // 不吞：交给树选中并持焦，调用方不得再清 _controlling
    }

    view.setEyeControlling(false);
    if (!isEmptyTreeHit(hit.item, hit.flags))
        return false;

    view.defocusEntityTree();
    return true;
}

void CSceneTreePane::OnTreeDblClk(NMHDR*, LRESULT* result)
{
    *result = 0; // root / entities / eyePoint 仍走默认展开/折叠
    const HTREEITEM item = hitTestEntityTree().item;
    if (!isEntityLeaf(item))
        return;

    *result = TRUE; // 实体叶子：不要再走默认展开
    _tree.SelectItem(item);
    if (CViewHostView* view = viewHostView(this))
        view->PostMessage(wmOpenEntityProperties, _tree.GetItemData(item));
}
