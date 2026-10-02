#include "FeatureSplitEditorSupport.hpp"

#include "BodySplitEditorModel.hpp"
#include "libslic3r/Slicing.hpp"
#include "libslic3r/libslic3r.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdio>
#include <iomanip>
#include <sstream>

namespace Slic3r::GUI {
namespace {

constexpr std::array<const char*, 5> fine_role_keys = {
    "outer_wall_filament_id", "inner_wall_filament_id", "internal_solid_filament_id",
    "top_surface_filament_id", "bottom_surface_filament_id"};
constexpr const char* coarse_role_key = "sparse_infill_filament_id";
// Process-owned (s_Preset_print_options), never a project or plate key.
constexpr const char* coarse_cadence_key = "mixed_nozzle_coarse_layer_height";

// Plain millimetre text for a limit sentence, without trailing zeros: "0.42 mm", not
// "0.420000 mm".
std::string limit_height_text(double value)
{
    char buffer[64];
    std::snprintf(buffer, sizeof(buffer), "%.4g", value);
    return std::string(buffer);
}

std::string resolver_status(const MixedNozzleDiagnostic& diagnostic)
{
    return std::string("[") + diagnostic.stable_code() + "] " + diagnostic.message();
}

std::string ready_status(const FeatureSplitResolvedRole& fine, const FeatureSplitResolvedRole& coarse)
{
    std::ostringstream out;
    // The nozzle by side and size, never a tool code.
    out << std::fixed << std::setprecision(2)
        << "Fine on the " << (fine.physical_tool == 0 ? "left " : "right ") << fine.nozzle_diameter
        << " mm nozzle, coarse on the " << (coarse.physical_tool == 0 ? "left " : "right ") << coarse.nozzle_diameter
        << " mm nozzle.";
    return out.str();
}

bool exact_local_pair(PartPlate& plate, int fine, int coarse)
{
    const DynamicPrintConfig* config = plate.config();
    for (const char* key : fine_role_keys) {
        const auto* option = config->option<ConfigOptionInt>(key);
        if (option == nullptr || option->value != fine)
            return false;
    }
    const auto* option = config->option<ConfigOptionInt>(coarse_role_key);
    return option != nullptr && option->value == coarse;
}

} // namespace

bool feature_split_editor_visible(MixedNozzleSlicingMode effective_mode)
{
    return effective_mode == MixedNozzleSlicingMode::FeatureSplit;
}

bool feature_split_target_valid(const FeatureSplitStableTarget& target,
                                int current_plate_index,
                                size_t indexed_plate_id)
{
    return target.plate_index >= 0 && target.plate_index == current_plate_index &&
           target.plate_id != 0 && target.plate_id == indexed_plate_id;
}

std::string feature_split_filament_label(size_t one_based_filament, const std::string& preset_label)
{
    return std::to_string(one_based_filament) + ": " + preset_label;
}

std::optional<std::pair<int, int>> feature_split_pair_from_selections(
    const FeatureSplitSelection& selections, size_t logical_filament_count)
{
    if (selections.fine_selection < 0 || selections.coarse_selection < 0 || logical_filament_count == 0 ||
        size_t(selections.fine_selection) >= logical_filament_count ||
        size_t(selections.coarse_selection) >= logical_filament_count ||
        selections.fine_selection == selections.coarse_selection)
        return std::nullopt;
    return std::pair{selections.fine_selection + 1, selections.coarse_selection + 1};
}

std::optional<FeatureSplitApplyRequest> build_feature_split_apply_request(
    const FeatureSplitStableTarget& target,
    const FeatureSplitSelection& selections,
    size_t logical_filament_count)
{
    const auto pair = feature_split_pair_from_selections(selections, logical_filament_count);
    if (!pair)
        return std::nullopt;
    return FeatureSplitApplyRequest{true, target, pair->first, pair->second, logical_filament_count};
}

FeatureSplitBoundedText bounded_feature_split_text(const std::string& full, size_t max_chars)
{
    std::string one_line = full;
    std::replace(one_line.begin(), one_line.end(), '\n', ' ');
    std::replace(one_line.begin(), one_line.end(), '\r', ' ');
    FeatureSplitBoundedText result{one_line, full};
    if (one_line.size() <= max_chars)
        return result;
    if (max_chars <= 3) {
        result.display.assign(max_chars, '.');
        return result;
    }
    result.display = one_line.substr(0, max_chars - 3) + "...";
    return result;
}

FeatureSplitEditorState build_feature_split_editor_state(
    MixedNozzleSlicingMode effective_mode,
    size_t configured_nozzle_count,
    std::optional<std::pair<int, int>> effective_pair,
    size_t logical_filament_count,
    const PrintConfig& resolver_config)
{
    FeatureSplitEditorState result;
    result.visible = feature_split_editor_visible(effective_mode);
    result.effective_pair = effective_pair;
    if (!result.visible) {
        result.diagnostic = FeatureSplitEditorDiagnostic::NotFeatureMode;
        return result;
    }
    if (!mixed_nozzle_configured_topology_supported(configured_nozzle_count)) {
        result.diagnostic = FeatureSplitEditorDiagnostic::UnsupportedTopology;
        result.full_status = MIXED_NOZZLE_UNSUPPORTED_TOPOLOGY;
        return result;
    }
    if (!effective_pair) {
        result.diagnostic = FeatureSplitEditorDiagnostic::IncompleteRoute;
        result.full_status = "[MNS-FEATURE-A01] Feature Split needs both a fine and a coarse material. Open Set up / Change... and choose them.";
        return result;
    }

    const int fine_filament = effective_pair->first;
    const int coarse_filament = effective_pair->second;
    if (fine_filament <= 0 || coarse_filament <= 0 ||
        size_t(fine_filament) > logical_filament_count || size_t(coarse_filament) > logical_filament_count) {
        result.diagnostic = FeatureSplitEditorDiagnostic::LogicalFilamentInvalid;
        result.full_status = "[MNS-FEATURE-A02] A Feature Split material is not one of this project's filaments.";
        return result;
    }
    if (fine_filament == coarse_filament) {
        result.diagnostic = FeatureSplitEditorDiagnostic::DistinctLogicalFilamentsRequired;
        result.full_status = "[MNS-FEATURE-A03] The fine and coarse materials must be different filaments.";
        return result;
    }

    const auto fine_resolution = resolve_mixed_nozzle_tool(
        resolver_config, size_t(fine_filament - 1), MixedNozzleResolveScope::CompleteNozzleMetadata);
    if (!fine_resolution.tool) {
        result.diagnostic = FeatureSplitEditorDiagnostic::IncompleteRoute;
        result.full_status = fine_resolution.diagnostic ? resolver_status(*fine_resolution.diagnostic) :
            "[MNS-FEATURE-A04] The fine material is not assigned to a nozzle.";
        return result;
    }
    const auto coarse_resolution = resolve_mixed_nozzle_tool(
        resolver_config, size_t(coarse_filament - 1), MixedNozzleResolveScope::CompleteNozzleMetadata);
    if (!coarse_resolution.tool) {
        result.diagnostic = FeatureSplitEditorDiagnostic::IncompleteRoute;
        result.full_status = coarse_resolution.diagnostic ? resolver_status(*coarse_resolution.diagnostic) :
            "[MNS-FEATURE-A05] The coarse material is not assigned to a nozzle.";
        return result;
    }

    result.fine = FeatureSplitResolvedRole{fine_filament, fine_resolution.tool->physical_extruder,
                                           fine_resolution.tool->nozzle_diameter};
    result.coarse = FeatureSplitResolvedRole{coarse_filament, coarse_resolution.tool->physical_extruder,
                                             coarse_resolution.tool->nozzle_diameter};
    if (result.fine->physical_tool == result.coarse->physical_tool) {
        result.diagnostic = FeatureSplitEditorDiagnostic::DistinctPhysicalToolsRequired;
        result.full_status = "[MNS-FEATURE-A06] The fine and coarse materials must print on different nozzles.";
        return result;
    }

    result.can_apply = true;
    result.diagnostic = FeatureSplitEditorDiagnostic::None;
    result.full_status = ready_status(*result.fine, *result.coarse);
    return result;
}

MixedNozzleCoarseEnvelope mixed_nozzle_coarse_envelope(const PrintConfig& resolver_config)
{
    MixedNozzleCoarseEnvelope envelope;
    size_t coarse = 0;
    if (resolver_config.nozzle_diameter.size() > 2) {
        // Two of N: the coarse toolhead is the first widest nozzle.
        const auto tools = mixed_nozzle_default_tool_pair(resolver_config.nozzle_diameter.values);
        if (!tools)
            return envelope;
        coarse = tools->second;
    } else {
        if (resolver_config.nozzle_diameter.size() != 2)
            return envelope;
        const double first = resolver_config.nozzle_diameter.get_at(0);
        const double second = resolver_config.nozzle_diameter.get_at(1);
        if (first == second)
            return envelope;
        coarse = first < second ? 1 : 0;
    }
    const double diameter = resolver_config.nozzle_diameter.get_at(coarse);
    const double minimum = resolved_min_layer_height(resolver_config, coarse);
    const double maximum = std::min(resolved_max_layer_height(resolver_config, coarse), diameter);
    if (!std::isfinite(minimum) || !std::isfinite(maximum) || minimum <= 0. || maximum < minimum)
        return envelope;
    envelope.resolved = true;
    envelope.minimum = minimum;
    envelope.maximum = maximum;
    envelope.nozzle_diameter = diameter;
    return envelope;
}

std::vector<double> mixed_nozzle_admissible_coarse_heights(double fine_height,
                                                           const PrintConfig& resolver_config)
{
    std::vector<double> heights;
    if (!std::isfinite(fine_height) || fine_height <= 0.)
        return heights;
    const MixedNozzleCoarseEnvelope envelope = mixed_nozzle_coarse_envelope(resolver_config);
    if (!envelope.resolved)
        return heights;
    const int last_ratio = int(std::floor((envelope.maximum + EPSILON) / fine_height));
    for (int ratio = 2; ratio <= last_ratio; ++ratio) {
        const double height = fine_height * double(ratio);
        if (height + EPSILON >= envelope.minimum)
            heights.push_back(height);
    }
    return heights;
}

std::string mixed_nozzle_height_limit_sentence(double height, double nozzle_diameter,
                                               double minimum, double maximum)
{
    if (!std::isfinite(height) || height <= 0.)
        return "Enter a layer height in millimeters.";
    if (height > maximum + EPSILON)
        return limit_height_text(height) + " mm is above the " + limit_height_text(nozzle_diameter) +
               " mm nozzle's " + limit_height_text(maximum) + " mm limit.";
    if (height + EPSILON < minimum)
        return limit_height_text(height) + " mm is below the " + limit_height_text(nozzle_diameter) +
               " mm nozzle's " + limit_height_text(minimum) + " mm minimum.";
    return {};
}

std::string mixed_nozzle_coarse_height_rejection(double fine_height, double coarse_height,
                                                 const PrintConfig& resolver_config)
{
    if (!std::isfinite(fine_height) || fine_height <= 0.)
        return "Choose a fine layer height first.";
    const MixedNozzleCoarseEnvelope envelope = mixed_nozzle_coarse_envelope(resolver_config);
    if (!envelope.resolved)
        return "This printer has no second nozzle with layer height limits of its own, "
               "so there is no coarse cadence to set.";
    const std::string outside = mixed_nozzle_height_limit_sentence(
        coarse_height, envelope.nozzle_diameter, envelope.minimum, envelope.maximum);
    if (!outside.empty())
        return outside;
    const std::vector<double> heights = mixed_nozzle_admissible_coarse_heights(
        fine_height, resolver_config);
    for (double height : heights)
        if (is_approx(height, coarse_height))
            return {};
    // Inside the nozzle's range but not a whole number of fine layers, so the coarse nozzle
    // would have to land between two fine planes.
    return limit_height_text(coarse_height) + " mm is not a whole number of " +
           limit_height_text(fine_height) + " mm layers.";
}

std::vector<double> feature_split_coarse_cadence_choices(double base_layer_height,
                                                          const PrintConfig& resolver_config)
{
    // The same list the wizard's cadence page and Apply check read. The Coarse Core never runs at
    // ratio one, and the list is independent of Body Split's shared-base minimum.
    return mixed_nozzle_admissible_coarse_heights(base_layer_height, resolver_config);
}

PrintConfig feature_split_resolver_config_from_full(const DynamicPrintConfig& full_config)
{
    // A strict apply throws: PrintConfig has no slot for the print-object and print-region keys a
    // full config carries. Compose through the same tolerant helper Body Split uses.
    return mixed_nozzle_resolver_config_from_full(full_config);
}

std::optional<double> feature_split_effective_coarse_layer_height(
    const DynamicPrintConfig& project, const DynamicPrintConfig& process)
{
    // Mirrors construct_full_config()'s layering exactly: print preset first, project on top.
    if (const auto* override_height = project.option<ConfigOptionFloat>(coarse_cadence_key))
        return override_height->value;
    if (const auto* process_height = process.option<ConfigOptionFloat>(coarse_cadence_key))
        return process_height->value;
    return std::nullopt;
}

bool adopt_project_feature_split_cadence_override(DynamicPrintConfig& project,
                                                  DynamicPrintConfig& process)
{
    const auto* override_height = project.option<ConfigOptionFloat>(coarse_cadence_key);
    if (override_height == nullptr)
        return false;
    // Copy before erasing: the option is owned by `project`.
    const double value = override_height->value;
    const auto* process_height = process.option<ConfigOptionFloat>(coarse_cadence_key);
    if (process_height == nullptr || !is_approx(process_height->value, value))
        process.set_key_value(coarse_cadence_key, new ConfigOptionFloat(value));
    project.erase(coarse_cadence_key);
    return true;
}

FeatureSplitApplyResult commit_feature_split_editor_request(
    DynamicPrintConfig& project,
    DynamicPrintConfig& process,
    PartPlate& plate,
    MixedNozzleSlicingMode effective_mode,
    int current_plate_index,
    size_t indexed_plate_id,
    size_t configured_nozzle_count,
    const PrintConfig& resolver_config,
    const std::vector<double>& coarse_cadence_choices,
    const FeatureSplitApplyRequest& request,
    const FeatureSplitMutationHooks& hooks)
{
    FeatureSplitApplyResult result;
    if (!request.apply)
        return result;
    if (!feature_split_target_valid(request.target, current_plate_index, indexed_plate_id)) {
        result.diagnostic = FeatureSplitEditorDiagnostic::StaleTarget;
        return result;
    }

    const auto state = build_feature_split_editor_state(
        effective_mode, configured_nozzle_count,
        std::pair{request.fine_filament, request.coarse_filament},
        request.logical_filament_count, resolver_config);
    result.diagnostic = state.diagnostic;
    if (!state.can_apply)
        return result;

    // Fail closed like build_feature_split_editor_state(): a requested cadence outside the
    // qualified list is rejected before any snapshot or mutation.
    if (request.coarse_layer_height) {
        const bool qualified = std::any_of(coarse_cadence_choices.begin(), coarse_cadence_choices.end(),
            [&](double choice) { return is_approx(choice, *request.coarse_layer_height); });
        if (!qualified) {
            result.diagnostic = FeatureSplitEditorDiagnostic::CadenceNotQualified;
            return result;
        }
    }

    result.applied = true;
    const bool filament_unchanged = exact_local_pair(plate, request.fine_filament, request.coarse_filament);
    // Compare against the effective value: with a stale project override in play the process
    // preset alone is not what slices.
    const auto effective_height = feature_split_effective_coarse_layer_height(project, process);
    const bool height_unchanged = !request.coarse_layer_height ||
        (effective_height && is_approx(*effective_height, *request.coarse_layer_height));
    // Clearing a stale project override is itself a change even when every requested value already
    // matches: until it goes, the next Process-tab edit is silently shadowed.
    const bool project_override_present = project.has(coarse_cadence_key);
    if (filament_unchanged && height_unchanged && !project_override_present)
        return result;

    if (hooks.snapshot)
        hooks.snapshot();

    // Capture the effective pre-edit value. A legacy project override wins composition, so Undo
    // must move that value into the Process preset rather than restore the override.
    const auto* project_height_before_option = project.option<ConfigOptionFloat>(coarse_cadence_key);
    const auto* process_height_before_option = process.option<ConfigOptionFloat>(coarse_cadence_key);
    const std::optional<double> process_height_before = project_height_before_option != nullptr
        ? std::optional<double>{project_height_before_option->value}
        : process_height_before_option == nullptr
            ? std::nullopt : std::optional<double>{process_height_before_option->value};
    bool changed = false;
    // Scope is deliberately split: the fine/coarse pair stays plate-local (PartPlate config), the
    // cadence is process-wide (print preset). A project/process-wide cadence must not masquerade
    // as a plate-only edit, and a plate's pair must not become a global setting.
    if (!filament_unchanged && plate.set_feature_split_filaments(request.fine_filament, request.coarse_filament,
                                                                  request.logical_filament_count))
        changed = true;
    // Normalize the legacy layer first so the process preset carries the effective value even when
    // this commit only changes the pair; the requested height then overwrites it below.
    if (adopt_project_feature_split_cadence_override(project, process))
        changed = true;
    if (request.coarse_layer_height) {
        const auto* process_height = process.option<ConfigOptionFloat>(coarse_cadence_key);
        if (process_height == nullptr || !is_approx(process_height->value, *request.coarse_layer_height)) {
            process.set_key_value(coarse_cadence_key, new ConfigOptionFloat(*request.coarse_layer_height));
            changed = true;
        }
    }

    result.changed = changed;
    if (changed) {
        const auto* process_height_after_option = process.option<ConfigOptionFloat>(coarse_cadence_key);
        const std::optional<double> process_height_after = process_height_after_option == nullptr
            ? std::nullopt : std::optional<double>{process_height_after_option->value};
        if ((project_override_present || process_height_before != process_height_after) &&
            hooks.record_cadence_transition)
            hooks.record_cadence_transition(process_height_before, process_height_after);
        if (hooks.notify_batch_changed)
            hooks.notify_batch_changed();
    }
    return result;
}

} // namespace Slic3r::GUI
