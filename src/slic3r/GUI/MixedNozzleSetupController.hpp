#ifndef slic3r_GUI_MixedNozzleSetupController_hpp_
#define slic3r_GUI_MixedNozzleSetupController_hpp_

#include <algorithm>
#include <functional>
#include <cmath>
#include <cstddef>
#include <optional>
#include <string>

#include <utility>
#include <vector>

#include "libslic3r/MixedNozzleBinding.hpp"
#include "slic3r/GUI/PartPlate.hpp"
#include "slic3r/GUI/MixedNozzleTowerPolicy.hpp"

namespace Slic3r::GUI {

struct MixedNozzleModeSource {
    bool inherits_project {true};
    MixedNozzleSlicingMode effective {MixedNozzleSlicingMode::Off};

    bool operator==(const MixedNozzleModeSource &rhs) const
    {
        return inherits_project == rhs.inherits_project && effective == rhs.effective;
    }
    bool operator!=(const MixedNozzleModeSource &rhs) const { return !(*this == rhs); }
};

inline MixedNozzleSlicingMode mixed_nozzle_project_default(const DynamicPrintConfig &project)
{
    if (const auto *option = project.option<ConfigOptionEnum<MixedNozzleSlicingMode>>("mixed_nozzle_slicing_mode"))
        return option->value;
    return MixedNozzleSlicingMode::Off;
}

inline MixedNozzleModeSource effective_mixed_nozzle_mode(const DynamicPrintConfig &project, const PartPlate &plate)
{
    bool inherited = true;
    const MixedNozzleSlicingMode effective = plate.get_real_mixed_nozzle_slicing_mode(project, &inherited);
    return {inherited, effective};
}

enum class MixedNozzleSetupCommit {
    Cancel,
    SaveProjectDefault,
    ApplyProjectDefaultToAll,
    ApplyCurrentPlate,
    ApplyPlateOverrideToAll
};

struct MixedNozzleSetupRequest {
    MixedNozzleSetupCommit commit {MixedNozzleSetupCommit::Cancel};
    bool use_project_default {false};
    MixedNozzleSlicingMode mode {MixedNozzleSlicingMode::Off};
    std::optional<std::vector<double>> nozzle_diameters;
    std::vector<double> min_layer_heights;
    std::vector<double> max_layer_heights;
    // Keeping it on the request pins the automatic policy to this transaction.
    std::optional<MixedNozzleTowerPolicyRequest> tower_policy;
    // Resolved by detached staging before the snapshot. Empty keeps the mode-only behaviour; a
    // value is preflighted by the bundle adapter.
    std::optional<std::string> resolved_process_name;
    std::optional<std::string> resolved_printer_name;
    bool transactional_state_changed {false};
    bool resolve_default_process {false};
    // Resolve the GUI proposal from the staged final presets and effective plate state.
    bool resolve_tower_usage {false};
};

inline bool mixed_nozzle_project_nozzle_pair_valid(const std::vector<double> &diameters)
{
    return diameters.size() == 2 && std::all_of(diameters.begin(), diameters.end(),
        [](double value) { return std::isfinite(value) && value > 0.; });
}

// Writes the requested nozzle pair with both layer height envelopes. Without a pair, writes nothing.
inline void apply_requested_nozzle_pair(DynamicPrintConfig &config, const MixedNozzleSetupRequest &request)
{
    if (!request.nozzle_diameters)
        return;
    config.set_key_value("nozzle_diameter", new ConfigOptionFloats(*request.nozzle_diameters));
    config.set_key_value("min_layer_height", new ConfigOptionFloats(request.min_layer_heights));
    config.set_key_value("max_layer_height", new ConfigOptionFloats(request.max_layer_heights));
}

// The plate commit the filament map dialog performs: Manual mode plus the map. The wizard stages
// the same two values onto a copied plate config, because its Apply has one publication boundary
// and PartPlate::set_filament_map_mode() reads the live GUI app. A mode change clears the map, so
// the map is written straight after, as the plate setters do.
inline void mixed_nozzle_write_manual_filament_map(DynamicPrintConfig &plate_config,
                                                   const std::vector<int> &filament_map)
{
    if (filament_map.empty())
        return;
    plate_config.set_key_value("filament_map_mode", new ConfigOptionEnum<FilamentMapMode>(fmmManual));
    plate_config.set_key_value("filament_map", new ConfigOptionInts(filament_map));
}

// Keep both installed nozzles without opening setup: the project pair and both envelopes go through
// the ordinary setup commit with the mode left as it is.
inline MixedNozzleSetupRequest mixed_nozzle_keep_pair_request(
    const DynamicPrintConfig &project, const std::vector<double> &nozzle_diameters,
    const std::vector<double> &min_layer_heights, const std::vector<double> &max_layer_heights)
{
    MixedNozzleSetupRequest request;
    request.commit = MixedNozzleSetupCommit::SaveProjectDefault;
    request.use_project_default = false;
    // Keeping the pair is not a mode change.
    request.mode = mixed_nozzle_project_default(project);
    request.nozzle_diameters = nozzle_diameters;
    request.min_layer_heights = min_layer_heights;
    request.max_layer_heights = max_layer_heights;
    return request;
}

// Turning Mixed-Nozzle Slicing off with nothing else changed. The wizard, the Printer row and the
// sidebar all commit it through this one request.
inline MixedNozzleSetupRequest mixed_nozzle_off_request(bool project_default)
{
    MixedNozzleSetupRequest request;
    request.commit = project_default ? MixedNozzleSetupCommit::SaveProjectDefault
                                     : MixedNozzleSetupCommit::ApplyCurrentPlate;
    request.use_project_default = false;
    request.mode = MixedNozzleSlicingMode::Off;
    // Re-resolve which plates still use the tower, so staging releases the plates this turns off
    // and, once none is left, restores the tower values Automatic changed and the user left alone.
    request.resolve_tower_usage = true;
    return request;
}

struct MixedNozzleSetupHooks {
    std::function<void()> take_snapshot;
    std::function<void(PartPlate &)> invalidate;
    std::function<void()> notify_batch_changed;
    // Called after the snapshot and before project and plate writes, to publish preflighted preset
    // values. Empty keeps the mode-only behaviour.
    std::function<bool()> publish_staged_state;
    std::function<void(const std::string &, const DynamicPrintConfig &,
                       const std::string &, const DynamicPrintConfig &,
                       const std::string &, const DynamicPrintConfig &,
                       const std::string &, const DynamicPrintConfig &)> record_setup_transition;

};

// Whether the request changes the project mode or a plate's own mode.
inline bool plate_mode_would_change(const DynamicPrintConfig &project, const std::vector<PartPlate *> &plates,
                                    const MixedNozzleSetupRequest &request, const PartPlate *current_plate)
{
    const auto plate_changes = [&request](const PartPlate *plate) {
        if (plate == nullptr)
            return false;
        const bool has = plate->config()->has("mixed_nozzle_slicing_mode");
        return request.use_project_default ? has : (!has || plate->get_mixed_nozzle_slicing_mode() != request.mode);
    };
    switch (request.commit) {
    case MixedNozzleSetupCommit::SaveProjectDefault:
        return mixed_nozzle_project_default(project) != request.mode;
    case MixedNozzleSetupCommit::ApplyProjectDefaultToAll:
        return mixed_nozzle_project_default(project) != request.mode ||
               std::any_of(plates.begin(), plates.end(), [](const PartPlate *plate) {
                   return plate != nullptr && plate->config()->has("mixed_nozzle_slicing_mode");
               });
    case MixedNozzleSetupCommit::ApplyCurrentPlate: return plate_changes(current_plate);
    case MixedNozzleSetupCommit::ApplyPlateOverrideToAll: return std::any_of(plates.begin(), plates.end(), plate_changes);
    default: return false;
    }
}

template <class OnConfigChange>
inline void notify_mixed_nozzle_commit(OnConfigChange &&on_config_change)
{
    std::forward<OnConfigChange>(on_config_change)();
}

inline bool commit_mixed_nozzle_setup(DynamicPrintConfig &project,
                                      const std::vector<PartPlate *> &plates,
                                      const MixedNozzleSetupRequest &request,
                                      const MixedNozzleSetupHooks &hooks,
                                      PartPlate *current_plate = nullptr)
{
    if (request.commit == MixedNozzleSetupCommit::Cancel ||
        (request.commit == MixedNozzleSetupCommit::ApplyCurrentPlate &&
         (current_plate == nullptr || std::find(plates.begin(), plates.end(), current_plate) == plates.end())))
        return false;

    const bool project_scope = request.commit == MixedNozzleSetupCommit::SaveProjectDefault ||
                               request.commit == MixedNozzleSetupCommit::ApplyProjectDefaultToAll;
    if (request.nozzle_diameters &&
        (!project_scope || !mixed_nozzle_project_nozzle_pair_valid(*request.nozzle_diameters)))
        return false;
    if (request.nozzle_diameters) {
        const auto valid_limits = [](const std::vector<double> &limits) {
            return limits.size() == 2 && std::all_of(limits.begin(), limits.end(),
                [](double value) { return std::isfinite(value) && value >= 0.; });
        };
        if (!valid_limits(request.min_layer_heights) || !valid_limits(request.max_layer_heights))
            return false;
    }

    std::vector<MixedNozzleModeSource> before;
    before.reserve(plates.size());
    for (const PartPlate *plate : plates)
        before.emplace_back(effective_mixed_nozzle_mode(project, *plate));

    const auto *old_nozzles = project.option<ConfigOptionFloats>("nozzle_diameter");
    const auto *old_minimums = project.option<ConfigOptionFloats>("min_layer_height");
    const auto *old_maximums = project.option<ConfigOptionFloats>("max_layer_height");
    const bool nozzle_changes = request.nozzle_diameters &&
                                (old_nozzles == nullptr || old_nozzles->values != *request.nozzle_diameters ||
                                 old_minimums == nullptr || old_minimums->values != request.min_layer_heights ||
                                 old_maximums == nullptr || old_maximums->values != request.max_layer_heights);
    bool changes = nozzle_changes;
    if (request.tower_policy || request.resolved_process_name || request.resolved_printer_name ||
        request.transactional_state_changed)
        changes = true;
    if (request.tower_policy) {
        const auto policy = evaluate_tower_policy(*request.tower_policy);
        if (policy.blocking_choice || (!policy.admission_diagnostics.empty() && request.tower_policy->stale))
            return false;
    }
    changes = changes || plate_mode_would_change(project, plates, request, current_plate);

    if (!changes)
        return false;
    if (hooks.take_snapshot)
        hooks.take_snapshot();
    if (hooks.publish_staged_state && !hooks.publish_staged_state())
        return false;

    // Diameter and both envelopes are written together, in one Undo transaction.
    apply_requested_nozzle_pair(project, request);

    if (request.commit == MixedNozzleSetupCommit::SaveProjectDefault ||
        request.commit == MixedNozzleSetupCommit::ApplyProjectDefaultToAll) {
        project.set_key_value("mixed_nozzle_slicing_mode",
            new ConfigOptionEnum<MixedNozzleSlicingMode>(request.mode));
    }
    if (request.commit == MixedNozzleSetupCommit::ApplyProjectDefaultToAll) {
        for (PartPlate *plate : plates)
            plate->clear_mixed_nozzle_slicing_mode();
    } else if (request.commit == MixedNozzleSetupCommit::ApplyCurrentPlate && current_plate != nullptr) {
        if (request.use_project_default)
            current_plate->clear_mixed_nozzle_slicing_mode();
        else
            current_plate->set_mixed_nozzle_slicing_mode(request.mode);
    } else if (request.commit == MixedNozzleSetupCommit::ApplyPlateOverrideToAll) {
        for (PartPlate *plate : plates) {
            if (request.use_project_default)
                plate->clear_mixed_nozzle_slicing_mode();
            else
                plate->set_mixed_nozzle_slicing_mode(request.mode);
        }
    }

    std::vector<MixedNozzleModeSource> after;
    after.reserve(plates.size());
    for (PartPlate *plate : plates)
        after.emplace_back(effective_mixed_nozzle_mode(project, *plate));

    for (size_t i = 0; i < plates.size(); ++i) {
        const bool feature_departure = before[i].effective == MixedNozzleSlicingMode::FeatureSplit &&
                                       after[i].effective != MixedNozzleSlicingMode::FeatureSplit;
        if (feature_departure)
            plates[i]->clear_feature_split_filaments();
        if ((nozzle_changes || request.transactional_state_changed || before[i] != after[i]) && hooks.invalidate)
            hooks.invalidate(*plates[i]);
    }
    if (hooks.notify_batch_changed)
        hooks.notify_batch_changed();
    return true;
}

// Whether the material rebind is applied along with an assignment change.
enum class MixedNozzleRebindAction { Cancel, Apply };

} // namespace Slic3r::GUI

#endif
