#pragma once

#include "libslic3r/MixedNozzleConfig.hpp"
#include "MixedNozzleNativeEntry.hpp"
#include "PartPlate.hpp"

#include <functional>
#include <optional>
#include <string>
#include <utility>
#include <vector>

namespace Slic3r::GUI {

enum class FeatureSplitEditorDiagnostic {
    None,
    NotFeatureMode,
    UnsupportedTopology,
    IncompleteRoute,
    LogicalFilamentInvalid,
    DistinctLogicalFilamentsRequired,
    PhysicalToolUnresolved,
    DistinctPhysicalToolsRequired,
    MixedAssignment,
    StaleTarget,
    // commit_feature_split_editor_request() re-validates the coarse cadence against the qualified
    // list (fails closed) rather than trusting the dialog's own filtering.
    CadenceNotQualified,
};

struct FeatureSplitStableTarget {
    int plate_index {-1};
    size_t plate_id {0};
};

struct FeatureSplitSelection {
    int fine_selection {-1};
    int coarse_selection {-1};
};

struct FeatureSplitResolvedRole {
    int logical_filament {0};
    size_t physical_tool {0};
    double nozzle_diameter {0.};
};

struct FeatureSplitEditorState {
    bool visible {false};
    bool can_apply {false};
    FeatureSplitEditorDiagnostic diagnostic {FeatureSplitEditorDiagnostic::None};
    std::optional<std::pair<int, int>> effective_pair;
    std::optional<FeatureSplitResolvedRole> fine;
    std::optional<FeatureSplitResolvedRole> coarse;
    std::string full_status;
};

struct FeatureSplitApplyRequest {
    bool apply {true};
    FeatureSplitStableTarget target;
    int fine_filament {0};
    int coarse_filament {0};
    size_t logical_filament_count {0};
    // Last, so positional aggregate-init call sites keep compiling and default to "no cadence".
    std::optional<double> coarse_layer_height {};
};

struct FeatureSplitBoundedText {
    std::string display;
    std::string tooltip;
};

struct FeatureSplitMutationHooks {
    std::function<void()> snapshot;
    std::function<void()> notify_batch_changed;
    // Called after mutation with the exact process-owned cadence edge captured by the same
    // snapshot. Pair-only edits leave it empty; legacy override normalization is included.
    std::function<void(std::optional<double>, std::optional<double>)> record_cadence_transition;
};

struct FeatureSplitApplyResult {
    bool applied {false};
    bool changed {false};
    FeatureSplitEditorDiagnostic diagnostic {FeatureSplitEditorDiagnostic::None};
};

bool feature_split_editor_visible(MixedNozzleSlicingMode effective_mode);
bool feature_split_target_valid(const FeatureSplitStableTarget& target,
                                int current_plate_index,
                                size_t indexed_plate_id);
std::string feature_split_filament_label(size_t one_based_filament, const std::string& preset_label);
std::optional<std::pair<int, int>> feature_split_pair_from_selections(
    const FeatureSplitSelection& selections, size_t logical_filament_count);
std::optional<FeatureSplitApplyRequest> build_feature_split_apply_request(
    const FeatureSplitStableTarget& target,
    const FeatureSplitSelection& selections,
    size_t logical_filament_count);
FeatureSplitBoundedText bounded_feature_split_text(const std::string& full, size_t max_chars);
FeatureSplitEditorState build_feature_split_editor_state(
    MixedNozzleSlicingMode effective_mode,
    size_t configured_nozzle_count,
    std::optional<std::pair<int, int>> effective_pair,
    size_t logical_filament_count,
    const PrintConfig& resolver_config);
// Body Split's choice list (body_split_cadence_choices()) filtered to ratio >= 2: the Coarse Core
// always runs coarser than the fine shell, so the base-height entry never qualifies here.
std::vector<double> feature_split_coarse_cadence_choices(double base_layer_height,
                                                          const PrintConfig& resolver_config);
// The coarse heights the user may choose: every integer ratio N >= 2 of the fine height whose
// coarse height sits inside the coarse nozzle's resolved minimum and maximum, bounded by the
// nozzle diameter. The wizard cadence page, the sidebar picker and the wizard's Apply check all
// read it. The process's allowed-ratio hint is an engine policy input and never narrows it.
struct MixedNozzleCoarseEnvelope {
    bool resolved {false};
    double minimum {0.};
    double maximum {0.};
    double nozzle_diameter {0.};
};
MixedNozzleCoarseEnvelope mixed_nozzle_coarse_envelope(const PrintConfig& resolver_config);
std::vector<double> mixed_nozzle_admissible_coarse_heights(double fine_height,
                                                           const PrintConfig& resolver_config);
// One plain sentence naming the limit that excluded a height, or empty when it is inside the
// range, so the wizard explains a refusal the same way wherever it happens.
std::string mixed_nozzle_height_limit_sentence(double height, double nozzle_diameter,
                                               double minimum, double maximum);
// Empty when this coarse height is admissible at this fine height, otherwise the reason.
std::string mixed_nozzle_coarse_height_rejection(double fine_height, double coarse_height,
                                                 const PrintConfig& resolver_config);
// The PrintConfig the Plate Settings Feature Split OK path validates against, composed from
// PresetBundle::full_config(). PrintConfig cannot hold the print-object and print-region keys a
// full config carries, so this composes tolerantly instead of with a strict apply.
PrintConfig feature_split_resolver_config_from_full(const DynamicPrintConfig& full_config);

// The composed coarse cadence the slicer will use. construct_full_config() applies project_config
// after the Print preset, so a project-layer copy of this process-owned key wins; reading through
// this helper keeps the panel and the Process tab on one value.
std::optional<double> feature_split_effective_coarse_layer_height(
    const DynamicPrintConfig& project, const DynamicPrintConfig& process);

// Fold a legacy project-layer override of the process-owned cadence into the process config (it
// is the value the user has been slicing with) and erase it from the project layer, so it can no
// longer shadow a later Process edit. Touches no other key. Returns whether anything changed.
bool adopt_project_feature_split_cadence_override(DynamicPrintConfig& project,
                                                  DynamicPrintConfig& process);

FeatureSplitApplyResult commit_feature_split_editor_request(
    // mixed_nozzle_coarse_layer_height is a process option (absent from s_project_options), so
    // `process` receives the value, and any override still in `project` is folded in and erased
    // in the same transaction.
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
    const FeatureSplitMutationHooks& hooks = {});

} // namespace Slic3r::GUI
