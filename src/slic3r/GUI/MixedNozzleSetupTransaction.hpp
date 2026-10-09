#pragma once

#include <cstdint>
#include <limits>
#include <string>
#include <utility>
#include <vector>

#include "libslic3r/PresetBundle.hpp"
#include <boost/log/trivial.hpp>
#include "slic3r/GUI/MixedNozzleSetupController.hpp"
#include "slic3r/GUI/MixedNozzleNativeEntry.hpp"
#include "libslic3r/Slicing.hpp"

namespace Slic3r::GUI {

struct MixedNozzleSelectedPresetState {
    std::string printer_name;
    std::string process_name;
    DynamicPrintConfig printer_config;
    DynamicPrintConfig process_config;
};

struct MixedNozzleSetupSignature {
    std::string printer_name;
    std::string process_name;
    std::vector<std::uint64_t> plate_ids;
    DynamicPrintConfig project_config;
    DynamicPrintConfig printer_config;
    DynamicPrintConfig process_config;
    std::vector<DynamicPrintConfig> plate_configs;
    std::uint64_t preset_generation {0};
};

struct MixedNozzleSetupPlan {
    MixedNozzleSetupSignature before;
    MixedNozzleSelectedPresetState after;
    DynamicPrintConfig named_printer_baseline;
    DynamicPrintConfig named_process_baseline;
    DynamicPrintConfig proposed_project_config;
    MixedNozzleTowerPolicyLedger resulting_ledger;
    std::vector<std::uint64_t> affected_plate_ids;
    std::string diagnostic;
};

inline MixedNozzleSetupSignature mixed_nozzle_setup_signature(
    const PresetBundle &bundle, const std::vector<PartPlate *> &plates)
{
    MixedNozzleSetupSignature signature;
    signature.printer_name = bundle.printers.get_edited_preset().name;
    signature.process_name = bundle.prints.get_edited_preset().name;
    signature.project_config = bundle.project_config;
    signature.printer_config = bundle.printers.get_edited_preset().config;
    signature.process_config = bundle.prints.get_edited_preset().config;
    signature.preset_generation = bundle.mixed_nozzle_preset_generation;
    signature.plate_ids.reserve(plates.size());
    for (const PartPlate *plate : plates)
        if (plate != nullptr)
            signature.plate_ids.push_back(plate->id().id);
    signature.plate_configs.reserve(plates.size());
    for (const PartPlate *plate : plates)
        if (plate != nullptr)
            signature.plate_configs.push_back(*plate->config());
    return signature;
}


inline MixedNozzleSlicingMode proposed_mixed_nozzle_plate_mode(
    const DynamicPrintConfig& project, const PartPlate& plate,
    const MixedNozzleSetupRequest& request, const PartPlate* current_plate)
{
    const bool project_scope = request.commit == MixedNozzleSetupCommit::SaveProjectDefault ||
                               request.commit == MixedNozzleSetupCommit::ApplyProjectDefaultToAll;
    const auto project_mode = project_scope ? request.mode : mixed_nozzle_project_default(project);
    if (request.commit == MixedNozzleSetupCommit::ApplyProjectDefaultToAll)
        return project_mode;
    if (request.commit == MixedNozzleSetupCommit::ApplyPlateOverrideToAll ||
        (request.commit == MixedNozzleSetupCommit::ApplyCurrentPlate && &plate == current_plate))
        return request.use_project_default ? project_mode : request.mode;
    return plate.config()->has("mixed_nozzle_slicing_mode") ? plate.get_mixed_nozzle_slicing_mode() : project_mode;
}

// Uses the CLI-capable assignment collector so staging reads the supplied final config, never the
// live process. Plate maps override the project map as in slicing.
inline MixedNozzleTowerPolicyRequest resolve_mixed_nozzle_setup_tower_usage(
    const PresetBundle& staged, const std::vector<PartPlate*>& plates,
    const MixedNozzleSetupRequest& request, const PartPlate* current_plate)
{
    auto policy = request.tower_policy.value_or(MixedNozzleTowerPolicyRequest{});
    policy.mode = MixedNozzleSlicingMode::Off;
    policy.resolved_physical_tools.clear();
    policy.consuming_plate_ids.clear();
    policy.mapping_unresolved = false;
    std::optional<DynamicPrintConfig> full;
    for (PartPlate* plate : plates) {
        if (!plate) continue;
        const auto mode = proposed_mixed_nozzle_plate_mode(staged.project_config, *plate, request, current_plate);
        if (mode == MixedNozzleSlicingMode::Off) continue;
        policy.mode = mode;
        if (plate->empty() || staged.filament_presets.empty()) {
            policy.mapping_unresolved = true;
            continue;
        }
        // An empty plate is pending; full_config assumes initialized filament slots, which may not
        // exist.
        if (!full)
            full = staged.full_config();
        DynamicPrintConfig effective = *full;
        effective.apply(*plate->config());
        effective.set_key_value("mixed_nozzle_slicing_mode", new ConfigOptionEnum<MixedNozzleSlicingMode>(mode));
        const auto maps = plate->get_real_filament_maps(*full);
        effective.set_key_value("filament_map", new ConfigOptionInts(maps));
        auto logical = plate->get_extruders_under_cli(true, effective);
        const auto pair = plate->get_effective_feature_split_filaments(effective, maps.size());
        if (mode == MixedNozzleSlicingMode::FeatureSplit && !pair)
            policy.mapping_unresolved = true;
        logical = mixed_nozzle_prime_tower_filaments(std::move(logical), mode, pair, maps);
        PrintConfig resolver;
        resolver.apply(effective, true);
        std::set<int> physical_tools;
        for (const int filament : logical) {
            if (filament <= 0) { policy.mapping_unresolved = true; continue; }
            const auto resolved = resolve_mixed_nozzle_tool(resolver, size_t(filament - 1),
                                                             MixedNozzleResolveScope::PhysicalToolOnly);
            if (!resolved) { policy.mapping_unresolved = true; continue; }
            physical_tools.insert(int(resolved.tool->physical_extruder));
        }
        if (logical.empty()) policy.mapping_unresolved = true;
        // One-tool plates never imply a two-tool job. Only a consuming plate contributes tools and
        // a plate slot to the shared ledger.
        if (physical_tools.size() >= 2) {
            policy.resolved_physical_tools.insert(policy.resolved_physical_tools.end(), physical_tools.begin(), physical_tools.end());
            policy.consuming_plate_ids.push_back(std::uint64_t(plate->get_index()));
        }
    }
    return policy;
}

inline MixedNozzleSetupPlan stage_mixed_nozzle_setup(
    const PresetBundle &live_bundle, const std::vector<PartPlate *> &plates,
    const MixedNozzleSetupRequest &request, PartPlate* current_plate = nullptr)
{
    MixedNozzleSetupPlan plan;
    plan.before = mixed_nozzle_setup_signature(live_bundle, plates);
    plan.proposed_project_config = live_bundle.project_config;
    apply_requested_nozzle_pair(plan.proposed_project_config, request);
    for (const PartPlate *plate : plates)
        if (plate != nullptr)
            plan.affected_plate_ids.push_back(plate->id().id);

    PresetBundle staged(live_bundle);
    if (request.resolved_printer_name) {
        // select_preset_by_name() falls back to the first visible preset for a hidden one, and a
        // variant the user never installed is hidden. Publication shows it.
        if (Preset *printer = staged.printers.find_preset(*request.resolved_printer_name))
            printer->is_visible = true;
        if (!staged.printers.select_preset_by_name(*request.resolved_printer_name, true) ||
            staged.printers.get_selected_preset_name() != *request.resolved_printer_name) {
            plan.diagnostic = "Mixed-Nozzle setup printer selection is unavailable.";
            return plan;
        }
    }
    apply_requested_nozzle_pair(staged.project_config, request);
    staged.update_compatible(PresetSelectCompatibleType::Never);
    if (request.resolve_default_process) {
        PrintConfig physical;
        physical.apply(staged.full_config(), true);
        const double minimum_base_height = request.mode == MixedNozzleSlicingMode::BodySplit
            ? (physical.nozzle_diameter.values.size() == 2
                ? std::max(resolved_min_layer_height(physical, 0), resolved_min_layer_height(physical, 1))
                : std::numeric_limits<double>::infinity()) : 0.;
        std::vector<MixedNozzleProcessRow> rows;
        for (const Preset &preset : staged.prints.get_presets()) {
            const auto *height = preset.config.option<ConfigOptionFloat>("mixed_nozzle_coarse_layer_height");
            const auto *fine = preset.config.option<ConfigOptionFloat>("layer_height");
            const auto *priority = preset.config.option<ConfigOptionInt>("mixed_nozzle_default_priority");
            if (preset.is_system && preset.is_compatible && height && fine)
                rows.push_back({preset.name, height->value, fine->value, priority ? priority->value : 0});
        }
        const std::string selected = mixed_nozzle_default_process_row(std::move(rows), minimum_base_height);
        if (selected.empty()) {
            plan.diagnostic = "Mixed-Nozzle setup has no compatible process preset for the staged printer.";
            return plan;
        }
        staged.prints.select_preset_by_name(selected, true);
    }
    const Preset *requested_process = request.resolved_process_name
        ? staged.prints.find_preset(*request.resolved_process_name) : nullptr;
    if (request.resolved_process_name &&
        (requested_process == nullptr || !requested_process->is_compatible)) {
        BOOST_LOG_TRIVIAL(warning) << "stage_mixed_nozzle_setup: process " << *request.resolved_process_name
                                   << (requested_process == nullptr ? " is not installed" : " does not fit the staged printer")
                                   << "; project min_layer_height "
                                   << (staged.project_config.has("min_layer_height") ? staged.project_config.opt_serialize("min_layer_height") : std::string("none"))
                                   << ", max_layer_height "
                                   << (staged.project_config.has("max_layer_height") ? staged.project_config.opt_serialize("max_layer_height") : std::string("none"));
        plan.diagnostic = "Mixed-Nozzle setup process selection is unavailable.";
        return plan;
    }
    if (request.resolved_process_name)
        staged.prints.select_preset_by_name(*request.resolved_process_name, true);

    plan.after.printer_name = staged.printers.get_edited_preset().name;
    plan.after.process_name = staged.prints.get_edited_preset().name;
    plan.after.printer_config = staged.printers.get_edited_preset().config;
    plan.after.process_config = staged.prints.get_edited_preset().config;
    if (const auto* preset = live_bundle.printers.find_preset(plan.after.printer_name))
        plan.named_printer_baseline = preset->config;
    if (const auto* preset = live_bundle.prints.find_preset(plan.after.process_name))
        plan.named_process_baseline = preset->config;

    std::optional<MixedNozzleTowerPolicyLedger> existing_ledger;
    bool has_serialized_ledger = false;
    if (const auto *serialized = live_bundle.project_config.option<ConfigOptionString>(MIXED_NOZZLE_TOWER_POLICY_OPTION)) {
        if (!serialized->value.empty()) {
            has_serialized_ledger = true;
            existing_ledger = MixedNozzleTowerPolicyLedger::parse(serialized->value);
        }
    }

    const auto resolved_policy = request.resolve_tower_usage
        ? std::optional<MixedNozzleTowerPolicyRequest>(resolve_mixed_nozzle_setup_tower_usage(staged, plates, request, current_plate))
        : request.tower_policy;
    if (resolved_policy) {
        const auto& policy = *resolved_policy;
        const MixedNozzleTowerPolicyEvaluation evaluation = evaluate_tower_policy(policy);
        if (evaluation.blocking_choice || policy.stale) {
            plan.diagnostic = evaluation.admission_diagnostics.empty()
                ? "Mixed-Nozzle setup policy is not admissible."
                : evaluation.admission_diagnostics.front();
            return plan;
        }
        if (evaluation.manage_values && has_serialized_ledger && !existing_ledger) {
            plan.diagnostic = "Mixed-Nozzle tower ownership metadata is unknown or corrupt; existing values are preserved.";
            return plan;
        }
        if (existing_ledger && evaluation.tower_required &&
            existing_ledger->process_scope != plan.after.process_name) {
            // A new process may be selected. The previous edge is already captured for Undo, and
            // its ownership cannot carry into the new scope.
            existing_ledger.reset();
            plan.proposed_project_config.erase(MIXED_NOZZLE_TOWER_POLICY_OPTION);
        }
        if (evaluation.manage_values && evaluation.tower_required) {
            const auto dirty_keys = [](const auto &collection) {
                const auto dirty = collection.current_dirty_options();
                return std::set<std::string>(dirty.begin(), dirty.end());
            };
            MixedNozzleTowerLedgerInputs inputs;
            inputs.process = &plan.after.process_config;
            inputs.printer = &plan.after.printer_config;
            inputs.process_scope = plan.after.process_name;
            inputs.printer_scope = plan.after.printer_name;
            inputs.explicit_process_keys = dirty_keys(staged.prints);
            inputs.explicit_printer_keys = dirty_keys(staged.printers);
            inputs.existing = existing_ledger ? &*existing_ledger : nullptr;
            inputs.provenance = policy.provenance;
            plan.resulting_ledger = mixed_nozzle_tower_build_ledger(inputs);
            mixed_nozzle_apply_tower_values(plan.after.process_config, &plan.after.printer_config,
                                            plan.resulting_ledger);
            plan.resulting_ledger.consuming_plate_ids = policy.consuming_plate_ids;
            plan.proposed_project_config.set_key_value(MIXED_NOZZLE_TOWER_POLICY_OPTION,
                new ConfigOptionString(plan.resulting_ledger.serialize()));
        } else if (existing_ledger && !policy.mapping_unresolved && evaluation.tower_required &&
                   existing_ledger->process_scope == plan.after.process_name) {
            auto remaining = *existing_ledger;
            remaining.consuming_plate_ids = policy.consuming_plate_ids;
            plan.proposed_project_config.set_key_value(MIXED_NOZZLE_TOWER_POLICY_OPTION,
                new ConfigOptionString(remaining.serialize()));
        } else if (existing_ledger && !policy.mapping_unresolved && !evaluation.tower_required &&
                   existing_ledger->process_scope == plan.after.process_name &&
                   policy.consuming_plate_ids.empty()) {
            auto remaining = *existing_ledger;
            DynamicPrintConfig *printer_target =
                existing_ledger->printer_scope == plan.after.printer_name ? &plan.after.printer_config : nullptr;
            for (const auto &key : mixed_nozzle_restore_tower_values(plan.after.process_config,
                                                                      printer_target, *existing_ledger))
                remaining.manual_keys.insert(key);
            remaining.managed_keys.clear();
            remaining.before_values.clear();
            remaining.applied_values.clear();
            remaining.consuming_plate_ids.clear();
            if (remaining.manual_keys.empty()) plan.proposed_project_config.erase(MIXED_NOZZLE_TOWER_POLICY_OPTION);
            else plan.proposed_project_config.set_key_value(MIXED_NOZZLE_TOWER_POLICY_OPTION,
                new ConfigOptionString(remaining.serialize()));
        }
    }
    if (request.resolved_process_name || request.resolve_default_process)
        plan.proposed_project_config.erase("mixed_nozzle_coarse_layer_height");
    plan.diagnostic.clear();
    return plan;
}

inline bool publish_mixed_nozzle_setup_presets(
    PresetBundle &bundle, const MixedNozzleSetupPlan &plan)
{
    if (!plan.diagnostic.empty())
        return false;
    if (bundle.printers.find_preset(plan.after.printer_name) == nullptr ||
        bundle.prints.find_preset(plan.after.process_name) == nullptr)
        return false;
    // Setup may move to a printer variant that was not installed; show it, as stock sync does.
    if (Preset *printer = bundle.printers.find_preset(plan.after.printer_name))
        printer->is_visible = true;
    bundle.printers.select_preset_by_name(plan.after.printer_name, true);
    bundle.prints.select_preset_by_name(plan.after.process_name, true);
    bundle.printers.get_edited_preset().config = plan.after.printer_config;
    bundle.prints.get_edited_preset().config = plan.after.process_config;
    bundle.project_config = plan.proposed_project_config;
    return true;
}

inline bool mixed_nozzle_setup_plan_matches_live(
    const PresetBundle &bundle, const std::vector<PartPlate *> &plates,
    const MixedNozzleSetupPlan &plan)
{
    const auto* target_printer = bundle.printers.find_preset(plan.after.printer_name);
    const auto* target_process = bundle.prints.find_preset(plan.after.process_name);
    if (!target_printer || !target_process || target_printer->config != plan.named_printer_baseline ||
        target_process->config != plan.named_process_baseline) return false;
    const MixedNozzleSetupSignature current = mixed_nozzle_setup_signature(bundle, plates);
    return current.printer_name == plan.before.printer_name &&
           current.process_name == plan.before.process_name &&
           current.project_config == plan.before.project_config &&
           current.printer_config == plan.before.printer_config &&
           current.process_config == plan.before.process_config &&
           current.plate_ids == plan.before.plate_ids &&
           current.plate_configs == plan.before.plate_configs &&
           current.preset_generation == plan.before.preset_generation;
}

inline bool mixed_nozzle_setup_plan_changes_state(const MixedNozzleSetupPlan &plan,
                                                  const MixedNozzleSetupRequest &request,
                                                  const std::vector<PartPlate *> &plates,
                                                  PartPlate *current_plate)
{
    return plate_mode_would_change(plan.before.project_config, plates, request, current_plate) ||
           plan.before.printer_name != plan.after.printer_name ||
           plan.before.process_name != plan.after.process_name ||
           !(plan.before.printer_config == plan.after.printer_config) ||
           !(plan.before.process_config == plan.after.process_config) ||
           !(plan.before.project_config == plan.proposed_project_config);
}

inline bool commit_mixed_nozzle_setup_transaction(
    PresetBundle &bundle, const std::vector<PartPlate *> &plates,
    const MixedNozzleSetupPlan &plan, const MixedNozzleSetupRequest &request,
    const MixedNozzleSetupHooks &hooks, PartPlate *current_plate = nullptr)
{
    if (!plan.diagnostic.empty() || !mixed_nozzle_setup_plan_matches_live(bundle, plates, plan) ||
        !mixed_nozzle_setup_plan_changes_state(plan, request, plates, current_plate))
        return false;
    MixedNozzleSetupRequest commit_request = request;
    commit_request.resolved_process_name.reset();
    commit_request.resolved_printer_name.reset();
    commit_request.tower_policy.reset();
    commit_request.resolve_default_process = false;
    commit_request.transactional_state_changed = true;
    MixedNozzleSetupHooks staged_hooks = hooks;
    staged_hooks.publish_staged_state = [&bundle, plan] {
        return publish_mixed_nozzle_setup_presets(bundle, plan);
    };
    const bool committed = commit_mixed_nozzle_setup(bundle.project_config, plates, commit_request,
                                                     staged_hooks, current_plate);
    if (committed && hooks.record_setup_transition)
        hooks.record_setup_transition(plan.before.printer_name, plan.before.printer_config,
                                      plan.before.process_name, plan.before.process_config,
                                      plan.after.printer_name, plan.after.printer_config,
                                      plan.after.process_name, plan.after.process_config);
    return committed;
}

inline bool commit_mixed_nozzle_setup_transaction(
    PresetBundle &bundle, const std::vector<PartPlate *> &plates,
    const MixedNozzleSetupRequest &request, const MixedNozzleSetupHooks &hooks,
    PartPlate *current_plate = nullptr)
{
    return commit_mixed_nozzle_setup_transaction(bundle, plates,
        stage_mixed_nozzle_setup(bundle, plates, request, current_plate), request, hooks, current_plate);
}

} // namespace Slic3r::GUI
