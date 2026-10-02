#pragma once

#include <functional>
#include <string>
#include <vector>

#include <wx/dataview.h>

#include "slic3r/GUI/ObjectDataViewModel.hpp"   // ItemType, ObjectDataViewModel
#include "slic3r/Utils/UndoRedo.hpp"        // UndoRedo::Snapshot, UndoRedo::SnapshotType

namespace Slic3r {
class ModelObject;
class ModelConfig;
} // namespace Slic3r

namespace Slic3r::GUI {

// The GUI and undo hooks the filament column change needs. resolve_layer_config may be empty;
// itLayer items then count as "no config".
struct ObjectListFilamentChangeHooks
{
    std::function<void(const std::string& snapshot_name)> take_snapshot;
    std::function<void()> notify_scene_update;
    std::function<ModelConfig*(const wxDataViewItem&)> resolve_layer_config;
};

// The read-only part of the object-list data-view model the filament column change needs.
// ObjectDataViewModel cannot be constructed without a live GUI_App, so tests supply a fake.
struct ObjectListItemView
{
    virtual ~ObjectListItemView() = default;

    virtual ItemType       GetItemType(const wxDataViewItem& item) const = 0;
    virtual int            GetIdByItem(const wxDataViewItem& item) const = 0;
    virtual wxDataViewItem GetObject(const wxDataViewItem& item) const = 0;
    virtual int            GetVolumeIdByItem(const wxDataViewItem& item) const = 0;
    virtual int            get_real_volume_index_in_3d(int ui_object_value, int ui_volume_value) const = 0;
    // The number the filament column currently *displays* for this row (node text via atoi),
    // which is not necessarily what the row's ModelConfig holds; see apply_filament_column_change.
    virtual int            GetExtruderNumber(const wxDataViewItem& item) const = 0;
};

// Production adapter: forwards all six queries straight to the live ObjectDataViewModel.
class ObjectDataViewModelItemView final : public ObjectListItemView
{
public:
    explicit ObjectDataViewModelItemView(ObjectDataViewModel& model) : m_model(model) {}

    ItemType       GetItemType(const wxDataViewItem& item) const override { return m_model.GetItemType(item); }
    int            GetIdByItem(const wxDataViewItem& item) const override { return m_model.GetIdByItem(item); }
    wxDataViewItem GetObject(const wxDataViewItem& item) const override { return m_model.GetObject(item); }
    int            GetVolumeIdByItem(const wxDataViewItem& item) const override { return m_model.GetVolumeIdByItem(item); }
    // ObjectDataViewModel::get_real_volume_index_in_3d is non-const (it indexes a std::map with
    // operator[]); the adapter holds a reference, so calling it from a const method is fine.
    int            get_real_volume_index_in_3d(int ui_object_value, int ui_volume_value) const override
        { return m_model.get_real_volume_index_in_3d(ui_object_value, ui_volume_value); }
    int            GetExtruderNumber(const wxDataViewItem& item) const override { return m_model.GetExtruderNumber(item); }

private:
    ObjectDataViewModel& m_model;
};

// The body of ObjectList::update_filament_in_config. Returns true if a mutation (and snapshot)
// occurred, false on an early no-op return.
bool apply_filament_column_change(std::vector<ModelObject*>& objects,
                                  const ObjectListItemView& objects_model,
                                  const wxDataViewItem& item,
                                  const ObjectListFilamentChangeHooks& hooks);

// The same "a re-render is not a user action" rule as apply_filament_column_change, applied to
// the Undo / Redo stack. take_snapshot() releases every snapshot above the active one, yet a
// snapshot whose name ends in '!' counts as no project change, so it could silently destroy the
// state the user was about to Redo (e.g. "select partplate!" after an Undo).
//
// Returns true when taking `snapshot_name` / `snapshot_type` at `active_snapshot_time` would throw
// away a redo branch while recording no project change of its own, i.e. when the caller must skip
// it. `snapshots` is ordered by timestamp. Genuine user actions always return false.
bool snapshot_would_discard_redo_branch(const std::vector<UndoRedo::Snapshot>& snapshots,
                                        size_t                                active_snapshot_time,
                                        const std::string&                    snapshot_name,
                                        UndoRedo::SnapshotType                snapshot_type);

} // namespace Slic3r::GUI
