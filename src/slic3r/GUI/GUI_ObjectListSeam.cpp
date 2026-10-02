#include "GUI_ObjectListSeam.hpp"

#include "slic3r/GUI/ObjectDataViewModel.hpp"
#include "slic3r/GUI/I18N.hpp"
#include "libslic3r/Model.hpp"

namespace Slic3r::GUI {

namespace {

// The number the filament column would render for a row backed by `cfg`, mirroring
// ObjectDataViewModel's own rules:
//   * an explicit "extruder" key is rendered as-is;
//   * an OBJECT row with no key is rendered "1", not "0"
//     (AddObject: `config.has("extruder") ? config.extruder() : 1`, then wxString::Format("%d"));
//   * a volume row with no key inherits the OBJECT row's text
//     (AddVolumeChild: `extruder_str = root->m_extruder` when the passed extruder is 0).
// Anything else (layer rows) has no such rule, so 0, the "unset" sentinel, is returned and the
// explicit-key comparison in apply_filament_column_change remains the only guard for it.
int displayed_extruder_for_config(const ModelConfig&          cfg,
                                  const ItemType             item_type,
                                  const ObjectListItemView&  objects_model,
                                  const wxDataViewItem&      item)
{
    if (cfg.has("extruder"))
        return cfg.extruder();
    if (item_type & itObject)
        return 1;
    if (item_type & itVolume)
        return objects_model.GetExtruderNumber(objects_model.GetObject(item));
    return 0;
}

} // namespace

bool apply_filament_column_change(std::vector<ModelObject*>& objects,
                                  const ObjectListItemView& objects_model,
                                  const wxDataViewItem& item,
                                  const ObjectListFilamentChangeHooks& hooks)
{
    ModelConfig* config = nullptr;
    const ItemType item_type = objects_model.GetItemType(item);
    if (item_type & itObject) {
        const int obj_idx = objects_model.GetIdByItem(item);
        config = &objects[obj_idx]->config;
    }
    else {
        const int obj_idx = objects_model.GetIdByItem(objects_model.GetObject(item));
        if (item_type & itVolume) {
            const int ui_volume_idx = objects_model.GetVolumeIdByItem(item);
            if (obj_idx < 0 || ui_volume_idx < 0)
                return false;
            int volume_in3d_idx = objects_model.get_real_volume_index_in_3d(obj_idx, ui_volume_idx);
            config              = &objects[obj_idx]->volumes[volume_in3d_idx]->config;
        }
        else if (item_type & itLayer)
            config = hooks.resolve_layer_config ? hooks.resolve_layer_config(item) : nullptr;
    }

    if (!config)
        return false;

    const int extruder = objects_model.GetExtruderNumber(item);
    if (config->extruder() == extruder)
        return false;

    // The row's text is not evidence of a change: a row with no "extruder" key still displays a
    // number (1 for an object, the object's text for an inheriting part). Comparing it against the
    // stored 0 made every re-render look like an edit, which wrote an explicit key and, for an
    // object row, erased every volume's key. Compare against what the row would display instead.
    if (displayed_extruder_for_config(*config, item_type, objects_model, item) == extruder)
        return false;

    if (hooks.take_snapshot)
        hooks.take_snapshot(_u8L("Change Filament"));

    config->set_key_value("extruder", new ConfigOptionInt(extruder));

    // BBS
    if (item_type & itObject) {
        const int obj_idx = objects_model.GetIdByItem(item);
        for (ModelVolume* mv : objects[obj_idx]->volumes) {
            if (mv->config.has("extruder"))
                mv->config.erase("extruder");
        }
    }

    // update scene
    if (hooks.notify_scene_update)
        hooks.notify_scene_update();

    return true;
}

bool snapshot_would_discard_redo_branch(const std::vector<UndoRedo::Snapshot>& snapshots,
                                        size_t                                active_snapshot_time,
                                        const std::string&                    snapshot_name,
                                        UndoRedo::SnapshotType                snapshot_type)
{
    // A snapshot that records a project change may truncate: abandoning the redo branch is what
    // editing after an Undo means. This is the same test Plater::priv::take_snapshot and
    // snapshot_modifies_project(const Snapshot&) apply.
    const bool records_a_project_change = snapshot_modifies_project(snapshot_type) &&
                                          (snapshot_name.empty() || snapshot_name.back() != '!');
    if (records_a_project_change)
        return false;

    // Otherwise skip it only when a project-modifying snapshot above the active time is at risk.
    // At the top of the stack this scan is empty and the snapshot is recorded as before.
    for (const UndoRedo::Snapshot& snapshot : snapshots)
        if (snapshot.timestamp > active_snapshot_time && snapshot_modifies_project(snapshot))
            return true;
    return false;
}

} // namespace Slic3r::GUI
