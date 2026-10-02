#pragma once

#include "libslic3r/MixedNozzleRanking.hpp"
#include "libslic3r/Model.hpp"
#include "libslic3r/PresetBundle.hpp"
#include "slic3r/GUI/BodySplitEditorModel.hpp"
#include "slic3r/GUI/FeatureSplitEditorSupport.hpp"
#include "slic3r/GUI/MixedNozzleSetupTransaction.hpp"
#include "slic3r/GUI/MixedNozzleWizardModel.hpp"
#include "slic3r/GUI/PartPlate.hpp"

#include <array>
#include <cstdint>
#include <functional>
#include <optional>
#include <set>
#include <string>
#include <vector>

namespace Slic3r::GUI {

// Compose mapping values and their mode from the same plate, including after material review.
DynamicPrintConfig wizard_plate_effective_config(const PresetBundle &bundle, const PartPlate &plate);

// The process preset every coarse layer row is sliced, ranked and applied on, read the way
// prepare_wizard_apply reads it.
WizardProcessBasis wizard_process_basis_for(const WizardDraft &draft, const PresetBundle &bundle);

// The project's pair and its layer-height limits, whatever printer variant is selected, so the
// pages describe the pair Apply will publish.
WizardPairAlignment wizard_project_pair_alignment(const PresetBundle &bundle);

// The wizard's first page and the setup chooser seed their tower control from this one rule.
inline WizardTowerIntent mixed_nozzle_default_tower_intent(
    const DynamicPrintConfig &project, const std::vector<std::string> &different_settings_to_system,
    MixedNozzleSlicingMode configured_mode)
{
    return mixed_nozzle_tower_default_automatic(project, different_settings_to_system, configured_mode)
        ? WizardTowerIntent::Automatic : WizardTowerIntent::Preserve;
}


// Non-owning handles to the live Plater state. Values are copied only into PreparedWizardApply
// while staging can still fail.
struct WizardNativeOwners {
    PresetBundle *preset_bundle {nullptr};
    Model *model {nullptr};
    std::vector<PartPlate *> plates;
    PartPlate *current_plate {nullptr};
};

struct WizardNativeVolumeState {
    std::size_t object_id {0};
    std::size_t volume_id {0};
    ModelVolumeType type {ModelVolumeType::INVALID};
    const TriangleMesh *mesh_identity {nullptr};
    std::uint64_t config_timestamp {0};
    std::uint64_t supported_facets_timestamp {0};
    std::uint64_t seam_facets_timestamp {0};
    std::uint64_t mmu_segmentation_facets_timestamp {0};
    std::uint64_t fuzzy_skin_facets_timestamp {0};
    std::array<double, 16> transform {};
};

struct WizardNativeInstanceState {
    std::size_t instance_id {0};
    std::array<double, 16> transform {};
    bool printable {true};
    bool auto_drop {true};
    int arrange_order {0};
    size_t loaded_id {0};
    bool use_loaded_id_for_label {false};
    ModelInstanceEPrintVolumeState print_volume_state {ModelInstancePVS_Inside};
};

struct WizardNativeObjectState {
    std::size_t object_id {0};
    std::uint64_t config_timestamp {0};
    bool printable {true};
    std::vector<WizardNativeInstanceState> instances;
};

// A value snapshot of the live owners for stale checking; never a publication target.
struct WizardNativeOwnerState {
    PresetBundle *preset_bundle {nullptr};
    Model *model {nullptr};
    PartPlate *current_plate {nullptr};
    std::vector<PartPlate *> plates;
    std::uint64_t preset_generation {0};
    std::string printer_name;
    std::string process_name;
    // Covers filament names, maps, nozzle diameters and serialized effective values; the preset
    // generation alone does not advance for every such edit.
    MixedNozzleRebindSignature rebind_signature;
    DynamicPrintConfig project_config;
    DynamicPrintConfig printer_config;
    DynamicPrintConfig process_config;
    std::vector<std::size_t> plate_ids;
    // Plate IDs do not identify ordering, so the native index is part of the fingerprint and a
    // reorder cannot redirect a current-plate transaction.
    std::vector<int> plate_indices;
    std::vector<bool> plate_locked;
    std::vector<DynamicPrintConfig> plate_configs;
    std::vector<std::vector<std::pair<size_t, size_t>>> plate_memberships;
    std::vector<WizardNativeObjectState> objects;
    std::vector<WizardNativeVolumeState> volumes;
};

struct WizardPlateConfigDelta {
    PartPlate *owner {nullptr};
    std::size_t plate_id {0};
    DynamicPrintConfig config;
};

struct WizardObjectConfigDelta {
    ModelObject *owner {nullptr};
    std::size_t object_id {0};
    ModelConfig config;
};

struct WizardModelConfigDelta {
    ModelVolume *owner {nullptr};
    std::size_t object_id {0};
    std::size_t volume_id {0};
    ModelConfig config;
};

enum class WizardApplyAction {
    Apply,
    Cancel,
};

struct PreparedWizardApply {
    WizardApplyAction action {WizardApplyAction::Apply};
    WizardDraft draft;
    WizardNativeOwnerState before;
    // Setup staging owns scope-aware mode, preset and tower values; the wizard carries its detached
    // plan.
    MixedNozzleSetupPlan setup_plan;
    std::optional<PreparedPresetSelection> printer_selection;
    std::optional<PreparedPresetSelection> process_selection;
    DynamicPrintConfig project_config;
    // The material preset each logical slot ends on. Written only when a slot moved to the other
    // nozzle resolves to that nozzle's own installed profile.
    std::vector<std::string> filament_presets;
    bool filament_presets_changed {false};
    std::vector<WizardPlateConfigDelta> plate_deltas;
    std::vector<WizardModelConfigDelta> model_deltas;
    std::vector<WizardObjectConfigDelta> object_deltas;
    std::optional<FeatureSplitApplyRequest> feature_request;
    std::optional<BodySplitApplyRequest> body_request;
    std::vector<std::size_t> affected_plate_ids;
    // Notes the review page appends verbatim, for decisions the key deltas cannot express.
    std::vector<std::string> review_notes;
    // What the engine's Body Split admission (Print::validate) refuses on the staged project, in
    // plain words; also in review_notes. Review holds Apply back while it is set. Empty when
    // admitted.
    std::string admission_refusal;
    // The Process setting the refusal names, so Review can open it. Empty when it names none.
    std::string admission_refusal_key;
    // Values Apply turns off or zeroes on its own, one line each. Also in review_notes; Review
    // lists them under "Also changed".
    std::vector<std::string> also_changed;
    bool changed {false};
    std::string diagnostic;
};

// Only boundary notifications belong here: nothing fallible runs after the snapshot. Publication
// consumes the prepared values directly.
struct WizardApplyHooks {
    std::function<void()> take_snapshot;
    // Preferred: the Undo owner captures the prepared transition while opening the action.
    // Returning false means the Plater suppressed it, and nothing live may be published.
    std::function<bool(const MixedNozzleSetupPlan &)> take_snapshot_with_setup_transition;
    std::function<void(std::size_t)> invalidate;
    std::function<void()> notify_batch_changed;
};

struct WizardApplyResult {
    bool applied {false};
    bool changed {false};
    std::vector<std::size_t> affected_plate_ids;
    std::string diagnostic;
};

PreparedWizardApply prepare_wizard_apply(const WizardDraft &draft,
                                          const WizardNativeOwners &owners,
                                          WizardApplyAction action = WizardApplyAction::Apply);

// The review is the only approved-delta input to the transaction.
PreparedWizardApply prepare_wizard_apply(const WizardDraft &draft,
                                          const WizardReview &review,
                                          const WizardNativeOwners &owners,
                                          WizardApplyAction action = WizardApplyAction::Apply);

// The Body objects whose bodies touch across the two nozzles, or carry paint of another material,
// by object id. A body with no nozzle yet counts as its own. Object-space bounding boxes are
// compared, so a touching pair is never missed; the engine adds beams only at a real boundary.
std::set<std::size_t> wizard_touching_assemblies(const std::vector<ModelObject *> &objects,
                                                 const std::vector<BodySplitEditorRow> &rows);

// The sidebar line for a Body Split project on a Feature Split process preset, naming the Body
// Split preset setup would switch to. Empty for every other project.
std::string wizard_body_process_guard_line(const PresetBundle &bundle);

WizardApplyResult commit_wizard_apply(PreparedWizardApply prepared,
                                      const WizardNativeOwners &owners,
                                      const WizardApplyHooks &hooks);

// What a real slice of the current plate would run on if Apply published `prepared`: the detached
// bundle with the prepared values, the plate's maps and settings, and the model with the prepared
// deltas and only this plate's instances printable. Nothing live is changed or referenced.
MixedNozzleRankingSlice wizard_staged_slice(const PreparedWizardApply &prepared,
                                            const WizardNativeOwners &owners);

// The slice for one cadence row: the row's draft, prepared as Apply prepares it, then composed by
// wizard_staged_slice(). Shared process scope is allowed for the estimate only. A row setup would
// refuse comes back with a plain `diagnostic`.
MixedNozzleRankingSlice wizard_candidate_staged_slice(const WizardDraft &draft,
                                                      const WizardCandidate &candidate,
                                                      const std::vector<WizardBodyAssignment> &body,
                                                      const WizardNativeOwners &owners);

} // namespace Slic3r::GUI
