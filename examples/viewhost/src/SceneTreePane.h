#pragma once

#include <afxcmn.h>
#include <afxcontrolbars.h>

#include <aerovista/sync/HostDataManager.h>

#include <cstdint>
#include <vector>

class CViewHostView;

class CSceneTreePane : public CDockablePane
{
public:
    void rebuild(const std::vector<aerovista::sync::EntityAuthorityRow>& rows, bool controlling);
    void applyEyeControlling(bool controlling);
    void clearSelection();
    bool isTreeHwnd(HWND hwnd) const;
    /// 点在树上：处理眼点进入/退出。返回 true 表示已吞掉点击。
    bool handleEyeControlClick(HWND clickHwnd, CViewHostView& view);

protected:
    afx_msg int OnCreate(LPCREATESTRUCT createStruct);
    afx_msg void OnSize(UINT type, int cx, int cy);
    afx_msg void OnTreeDblClk(NMHDR* notify, LRESULT* result);

    DECLARE_MESSAGE_MAP()

private:
    struct EntityTreeHit
    {
        HTREEITEM item = nullptr;
        UINT flags = 0;
        CPoint client{};
    };

    void adjustLayout();
    void updateEyePointLabel(bool controlling);
    EntityTreeHit hitTestEntityTree();
    bool isEntityLeaf(HTREEITEM item) const;
    bool isEyePointHit(HTREEITEM item, UINT flags) const;
    bool isEmptyTreeHit(HTREEITEM item, UINT flags) const;

    CTreeCtrl _tree;
    HTREEITEM _rootItem = nullptr;
    HTREEITEM _eyePointItem = nullptr;
    HTREEITEM _entitiesFolder = nullptr;
};
