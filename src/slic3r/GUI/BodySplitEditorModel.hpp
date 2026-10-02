#pragma once

#include "libslic3r/MixedNozzleConfig.hpp"
#include "libslic3r/Model.hpp"

#include <functional>
#include <optional>
#include <string>
#include <vector>

namespace Slic3r::GUI {

enum class BodySplitEditorDiagnostic {
    None,
    ExactlyTwoModelPartsRequired,
    LayerHeightInvalid,
    LogicalFilamentInvalid,
    DistinctLogicalFilamentsRequired,
    PhysicalToolUnresolved,
    DistinctPhysicalToolsRequired,
    PhysicalNozzleOrderingInvalid,
    CadenceNotQualified,
    StaleTarget,
    DuplicateTarget,
    PaintedFilamentPhysicalToolMismatch,   // appended last so existing ordinals stay stable
};

struct BodySplitFilamentPresentation {
    int logical_filament {0};
    std::string preset_label;
    std::string material_label;
};

struct BodySplitPresentation {
    std::vector<BodySplitFilamentPresentation> filaments;
};

struct BodySplitEditorRow {
    ObjectID object_id;
    ObjectID volume_id;
    std::string object_name;
    std::string volume_name;
    ModelVolumeType type {ModelVolumeType::INVALID};
    int logical_filament {0};
    std::vector<int> painted_logical_filaments;
    bool painted_tool_mismatch {false};
    std::string preset_label;
    std::string material_label;
    bool regional_height_explicit {false};
    double base_layer_height {0.};
    double effective_layer_height {0.};
    double cadence_ratio {0.};
    std::vector<double> cadence_choices;
    MixedNozzleToolResolution resolution;
    // The fine-skin keys are read regardless of applicability, so a row that toggles between fine
    // and coarse cadence keeps its stored value. fine_skin_controls_applicable is true only for a
    // coarse-cadence row (integer cadence_ratio >= 2).
    bool fine_skin_controls_applicable {false};
    bool fine_skins {false};
    int fine_skin_layers {3};
};

struct BodySplitExcludedVolume {
    ObjectID object_id;
    ObjectID volume_id;
    std::string object_name;
    std::string volume_name;
    ModelVolumeType type {ModelVolumeType::INVALID};
};

struct BodySplitEditorModel {
    std::vector<BodySplitEditorRow> rows;
    std::vector<BodySplitExcludedVolume> excluded;
    bool supported {false};
    BodySplitEditorDiagnostic diagnostic {BodySplitEditorDiagnostic::None};
};

BodySplitEditorModel build_body_split_editor_model(const std::vector<ModelObject*>& objects,
                                                    const PrintConfig& resolver_config,
                                                    const BodySplitPresentation& presentation,
                                                    double inherited_layer_height = 0.);

PrintConfig mixed_nozzle_resolver_config_from_full(const DynamicPrintConfig& full_config);

// True when some object has two or more parts, or a painted part, whatever filaments they use.
bool body_split_rows_have_body_object(const std::vector<BodySplitEditorRow>& rows);

// The interlocking beam layer count one object's bodies need: its coarsest body's cadence as a
// whole number of fine layers. `layers` is empty when that is not a whole number of at least 2.
// Setup and the Body Split editor both size beams with it.
struct BodySplitBeamLayers {
    bool has_bodies {false};
    std::optional<int> layers;
};
BodySplitBeamLayers body_split_beam_layers(const std::vector<BodySplitEditorRow>& rows, ObjectID object_id);

template<typename T> struct BodySplitValuePatch {
    bool requested {false};
    std::optional<T> value;

    static BodySplitValuePatch set(T new_value) { return {true, std::move(new_value)}; }
    static BodySplitValuePatch erase() { return {true, std::nullopt}; }
};

struct BodySplitEdit {
    ObjectID object_id;
    ObjectID volume_id;
    BodySplitValuePatch<int> extruder;
    BodySplitValuePatch<double> regional_layer_height;
    // Staged the same way as regional_layer_height above.
    BodySplitValuePatch<bool> fine_skins;
    BodySplitValuePatch<int> fine_skin_layers;
};

struct BodySplitApplyRequest {
    bool apply {true};
    double inherited_layer_height {0.};
    std::vector<ObjectID> editable_object_ids;
    std::vector<BodySplitEdit> edits;
    BodySplitValuePatch<RegionalGridPhaseRule> phase;
};

struct BodySplitApplyResult {
    bool applied {false};
    bool changed {false};
    bool plate_changed {false};
    BodySplitEditorDiagnostic diagnostic {BodySplitEditorDiagnostic::None};
    std::vector<ObjectID> changed_object_ids;
};

struct BodySplitMutationHooks {
    std::function<void()> snapshot;
    std::function<void()> notify_and_invalidate;
    // Hosts that need to compose Body edits with another native publication boundary can stage
    // the pending request through that boundary. The existing two-hook path remains the default.
    std::function<BodySplitApplyResult(const BodySplitApplyRequest&)> transactional_commit;
};

enum class BodySplitPlateSettingsAcceptance {
    TargetInvalid,
    NotBodyMode,
    NoPending,
    CommitRejected,
    Committed,
};

struct BodySplitPlateSettingsAcceptanceResult {
    BodySplitPlateSettingsAcceptance acceptance {BodySplitPlateSettingsAcceptance::NoPending};
    BodySplitApplyResult apply_result;
};

struct BodySplitStableTarget {
    int plate_index {-1};
    size_t plate_id {0};
};

struct BodySplitBoundedText {
    std::string display;
    std::string tooltip;
};

struct BodySplitNativeSelection {
    ObjectID object_id;
    ObjectID volume_id;
    int filament_selection {-1};
    int cadence_selection {-1};
};

struct BodySplitLiveRequest {
    BodySplitEditorDiagnostic diagnostic {BodySplitEditorDiagnostic::None};
    std::optional<BodySplitApplyRequest> request;
};

bool body_split_editor_visible(MixedNozzleSlicingMode effective_mode);
bool body_split_editor_target_valid(const BodySplitStableTarget& target,
                                    int current_plate_index,
                                    size_t indexed_plate_id);
int plate_settings_target_index(int event_plate_index, int current_plate_index, size_t plate_count);
std::string body_split_filament_label(size_t one_based_filament, const std::string& preset_label);
// Physical coarse-height choices, including the ratio-one base entry. Feature Split does
// not require the coarse tool to deposit at the base height. Body Split narrows this below.
std::vector<double> mixed_nozzle_cadence_choices(double base_layer_height, const PrintConfig& resolver_config);
std::vector<double> body_split_cadence_choices(double base_layer_height, const PrintConfig& resolver_config);
BodySplitBoundedText bounded_body_split_text(const std::string& text, size_t max_chars);
// Visible identity for each Body Split row, bounded to max_chars. Sibling bodies share one
// parent object name, so the distinguishing leaf volume name has to survive truncation.
std::vector<BodySplitBoundedText> body_split_row_identities(const std::vector<BodySplitEditorRow>& rows,
                                                            size_t max_chars);
BodySplitEditorDiagnostic body_split_native_selection_diagnostic(
    const std::vector<BodySplitEditorRow>& rows,
    const std::vector<BodySplitNativeSelection>& selections,
    size_t filament_count,
    const PrintConfig& resolver_config);
BodySplitLiveRequest build_body_split_live_request(
    const std::vector<BodySplitEditorRow>& live_rows,
    const std::vector<BodySplitNativeSelection>& captured_selections,
    size_t filament_count,
    const PrintConfig& live_resolver_config,
    RegionalGridPhaseRule phase);
// Fine-skin widgets are free values, not a fixed choice list, so they merge directly onto the
// matching edit. Only a row with fine-skin controls (a coarse-cadence row) may stage a write.
void merge_body_split_fine_skins(BodySplitApplyRequest& request, const std::vector<BodySplitEditorRow>& rows,
                                 size_t widget_count, const std::function<bool(size_t)>& fine_skins,
                                 const std::function<int(size_t)>& fine_skin_layers);

BodySplitApplyResult apply_body_split_editor_request(Model& model,
                                                     DynamicPrintConfig& plate_config,
                                                     const PrintConfig& resolver_config,
                                                     const BodySplitApplyRequest& request,
                                                     const BodySplitMutationHooks& hooks = {});

class BodySplitEditorTransaction {
public:
    bool stage(const BodySplitApplyRequest& request);
    void cancel() { m_pending.reset(); }
    bool has_pending() const { return m_pending.has_value(); }
    BodySplitApplyResult commit(Model& model,
                                DynamicPrintConfig& plate_config,
                                const PrintConfig& resolver_config,
                                const BodySplitMutationHooks& hooks = {});

private:
    std::optional<BodySplitApplyRequest> m_pending;
};

BodySplitPlateSettingsAcceptanceResult accept_body_split_plate_settings(
    const BodySplitStableTarget& target,
    int current_plate_index,
    size_t indexed_plate_id,
    MixedNozzleSlicingMode effective_mode,
    BodySplitEditorTransaction& transaction,
    Model& model,
    DynamicPrintConfig& plate_config,
    const PrintConfig& resolver_config,
    const BodySplitMutationHooks& hooks = {});

} // namespace Slic3r::GUI
