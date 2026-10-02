#pragma once

#include "FeatureSplitEditorSupport.hpp"

#include <wx/defs.h>
#include <wx/stattext.h>
#include <wx/string.h>

#include <algorithm>
#include <array>
#include <cstddef>
#include <functional>
#include <optional>
#include <string>
#include <utility>
#include <vector>

namespace Slic3r::GUI {

struct FeatureSplitNativeSeed {
    std::vector<std::string> labels;
    FeatureSplitSelection selection;
};

class FeatureSplitNativeLifecycle {
public:
    bool may_access_state(bool controls_alive) const { return !m_shutting_down && controls_alive; }
    void begin_shutdown() { m_shutting_down = true; }
private:
    bool m_shutting_down {false};
};

inline FeatureSplitNativeSeed build_feature_split_native_seed(
    const std::vector<std::string>& preset_labels,
    std::optional<std::pair<int, int>> effective_pair)
{
    FeatureSplitNativeSeed result;
    result.labels.reserve(preset_labels.size());
    for (size_t index = 0; index < preset_labels.size(); ++index)
        result.labels.push_back(feature_split_filament_label(index + 1, preset_labels[index]));
    if (!effective_pair || effective_pair->first <= 0 || effective_pair->second <= 0 ||
        effective_pair->first == effective_pair->second ||
        size_t(effective_pair->first) > preset_labels.size() ||
        size_t(effective_pair->second) > preset_labels.size())
        return result;
    result.selection = {effective_pair->first - 1, effective_pair->second - 1};
    return result;
}

inline int feature_split_status_width_dip() { return 240; }
inline long feature_split_status_window_style() { return wxST_ELLIPSIZE_END; }

// Width a Mixed-Nozzle cadence combo needs so its closed label and its drop list (which follows
// the control's width) stay legible. `measure` is injected so the rule is testable without a wxApp.
inline int mixed_nozzle_cadence_combo_width(const std::vector<wxString>& labels, int min_width,
                                            int padding,
                                            const std::function<int(const wxString&)>& measure)
{
    int width = min_width;
    for (const wxString& label : labels)
        width = std::max(width, measure(label) + padding);
    return width;
}

struct FeatureSplitNativeLiveApply {
    FeatureSplitEditorState state;
    std::optional<FeatureSplitApplyRequest> request;
};

struct FeatureSplitPlateSettingsVisibility {
    bool feature_rows {false};
    bool body_editor {false};
    // The whole Mixed-Nozzle block in Plate Settings: the "Mixed-Nozzle Slicing" label, its status
    // and its "Set up..." button. Independent of mode, unlike the two editor flags above.
    bool mixed_block {false};
    bool operator==(const FeatureSplitPlateSettingsVisibility& rhs) const
    { return feature_rows == rhs.feature_rows && body_editor == rhs.body_editor &&
             mixed_block == rhs.mixed_block; }
};

inline FeatureSplitNativeLiveApply build_feature_split_native_live_apply(
    const FeatureSplitStableTarget& target,
    const FeatureSplitSelection& selection,
    int current_plate_index,
    size_t indexed_plate_id,
    MixedNozzleSlicingMode effective_mode,
    size_t configured_nozzle_count,
    size_t logical_filament_count,
    const PrintConfig& resolver_config)
{
    FeatureSplitNativeLiveApply result;
    if (!feature_split_target_valid(target, current_plate_index, indexed_plate_id)) {
        result.state.diagnostic = FeatureSplitEditorDiagnostic::StaleTarget;
        return result;
    }
    const auto pair = feature_split_pair_from_selections(selection, logical_filament_count);
    result.state = build_feature_split_editor_state(effective_mode, configured_nozzle_count, pair,
                                                    logical_filament_count, resolver_config);
    if (!result.state.can_apply)
        return result;
    result.request = FeatureSplitApplyRequest{true, target, pair->first, pair->second, logical_filament_count};
    return result;
}

inline FeatureSplitPlateSettingsVisibility feature_split_plate_settings_visibility(
    MixedNozzleSlicingMode mode, std::size_t configured_nozzle_count)
{
    // The same topology test as mixed_nozzle_configured_topology_supported(), spelled out so this
    // header does not depend on the setup controller. A toolchanger runs the modes on two tools.
    if (configured_nozzle_count < 2)
        return {false, false, false};
    return {mode == MixedNozzleSlicingMode::FeatureSplit, mode == MixedNozzleSlicingMode::BodySplit, true};
}

// Like the sidebar, Plate Settings keeps the inline Feature editor behind "Edit here" in its More
// menu. The rows are still filled on every refresh, so OK validates and applies the same selection
// whether or not they were opened.
inline bool feature_split_plate_settings_edit_here_offered(const FeatureSplitPlateSettingsVisibility& visibility)
{
    return visibility.feature_rows;
}

inline bool feature_split_plate_settings_rows_shown(const FeatureSplitPlateSettingsVisibility& visibility,
                                                    bool edit_here)
{
    return visibility.feature_rows && edit_here;
}

class FeatureSplitApplyAccounting {
public:
    bool needs_ordinary_dirty_bookkeeping(bool ordinary_changed) const
    {
        return ordinary_changed && !m_editor_dirty_done;
    }
    void feature_dirty_bookkeeping_done() { m_editor_dirty_done = true; }
    void body_dirty_bookkeeping_done() { m_editor_dirty_done = true; }
private:
    bool m_editor_dirty_done {false};
};

inline bool plate_settings_spiral_changed(bool current_value, bool current_has_local_config,
                                          bool selected_value, bool selected_as_global)
{
    return selected_as_global ? current_has_local_config
                              : !current_has_local_config || current_value != selected_value;
}

} // namespace Slic3r::GUI
