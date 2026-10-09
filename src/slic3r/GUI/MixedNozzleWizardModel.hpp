#pragma once

#include "libslic3r/MixedNozzleBinding.hpp"
#include "libslic3r/MixedNozzleConfig.hpp"

#include <array>
#include <cstddef>
#include <cstdint>
#include <map>
#include <optional>
#include <set>
#include <string>
#include <utility>
#include <vector>

namespace Slic3r::GUI {

// Explicit because a process-owned height change can reach more plates than the one being edited.
enum class WizardScopeKind {
    CurrentPlate,
    ProjectDefault,
};

struct WizardScope {
    WizardScopeKind kind {WizardScopeKind::CurrentPlate};
    size_t current_plate_id {0};
    std::vector<size_t> process_affected_plate_ids;
    bool allow_shared_process_changes {false};
    // Material binding is project-owned and may be reviewed for every affected plate even when the
    // mode stays a current-plate override.
    bool allow_project_binding_changes {false};
};

// Copied values only, no Model/ModelObject/PartPlate pointers, so a signature stays meaningful
// after the model is rebuilt.
struct WizardVolumeSignature {
    size_t object_id {0};
    size_t volume_id {0};
    uint64_t paint_revision {0};
    uint64_t config_revision {0};
    std::array<double, 16> transform {};
};

struct WizardSignature {
    std::vector<size_t> plate_ids;
    std::vector<WizardVolumeSignature> volumes;
    std::string selected_printer_id;
    std::string selected_process_id;
    std::vector<std::string> selected_material_ids;
};

struct WizardPhysicalRole {
    size_t logical_filament {0};
    size_t physical_extruder {0};
    double nozzle_diameter {0.};
    std::string role;
};

struct WizardBodyEdit {
    size_t object_id {0};
    size_t volume_id {0};
    std::map<std::string, std::string> staged_values;
};

struct WizardKeyDelta {
    std::string key;
    std::string old_value;
    std::string new_value;
    std::string source;
    std::string scope;
    std::string unit;
};

struct WizardNativeBindingApproval {
    MixedNozzleRebindPlan plan;
    std::vector<bool> accepted;
};

enum class WizardTowerIntent {
    Preserve,
    Automatic,
    Enabled,
    Disabled,
};

// Never a mixed cadence with N=1: that would imply a coarse owner and skip the assignment and
// material checks needed to put both roles on one tool.
enum class WizardCandidateKind {
    MixedCadence,
    SingleNozzle,
};

struct WizardDraft {
    WizardSignature signature;
    WizardScope scope;
    MixedNozzleSlicingMode mode {MixedNozzleSlicingMode::Off};
    // Copied so review and transaction never infer it from a row's height or ratio. Empty until the
    // user selects a candidate.
    std::optional<WizardCandidateKind> selected_candidate_kind;
    // Set when the picked row names an installed stock process preset; empty for resolver rows.
    // Never inferred from a height or label.
    std::optional<std::string> selected_source_preset_id;
    std::vector<WizardPhysicalRole> resolved_physical_roles;
    std::optional<size_t> fine_logical_filament;
    std::optional<size_t> coarse_logical_filament;
    double chosen_fine_height {0.};
    std::optional<double> chosen_coarse_height;
    std::optional<int> chosen_cadence_ratio;
    std::vector<WizardBodyEdit> staged_body_edits;
    // Object-scoped joining intent, one choice per assembly. An assembly in none of the three sets
    // takes the default at Apply: beam interlocking on when its bodies touch across the two
    // nozzles, its effective settings otherwise.
    std::set<std::size_t> enable_interlocking_objects;
    // Off, for a part meant to move. Writes interlocking_beam = false on the object.
    std::set<std::size_t> disable_interlocking_objects;
    // Keep the effective settings, whatever the default would be.
    std::set<std::size_t> keep_joining_objects;
    std::set<std::string> locked_keys;
    std::vector<WizardKeyDelta> approved_key_deltas;
    std::optional<WizardNativeBindingApproval> native_binding;
    WizardTowerIntent tower_intent {WizardTowerIntent::Preserve};
    // Set only by entry points carrying a new project nozzle pair (the sidebar diameter change and
    // the connected-printer mismatch prompt). The pair and its envelopes travel with the draft so a
    // nozzle change and its setup share one Undo step.
    std::optional<std::vector<double>> requested_nozzle_diameters;
    std::vector<double> requested_min_layer_heights;
    std::vector<double> requested_max_layer_heights;
    std::optional<std::string> requested_printer_name;
    bool resolve_default_process {false};
    // The plate filament map this setup writes (one 1-based physical extruder per logical slot) and
    // its mode, derived from the two chosen materials. Empty leaves the plate's mapping alone.
    std::vector<int> derived_filament_map;
    FilamentMapMode derived_map_mode {fmmDefault};
    // Take the process preset's first layer back. Feature Split and Body Split always do this; the
    // flag only decides for Off.
    bool take_preset_first_layer {false};
    // Rows are sliced, ranked and applied on one process preset, the pair's MN preset for this mode
    // (wizard_process_basis). Set from More options to keep the current preset with the heights on
    // top.
    bool keep_current_process {false};
};

// Copied native process and printer metadata. A display label or equal h/N pair is never enough to
// attach a source preset to a different nozzle/flow pair.
struct WizardNativeSourceCompatibility {
    // Native `mixed_nozzle_process_nozzle_diameters`, in canonical fine/coarse role order.
    std::vector<double> role_nozzle_diameters;
    // Native process variant columns and the selected printer's physical variant inventory.
    std::vector<int> process_extruder_ids;
    std::vector<std::string> process_extruder_variants;
    std::vector<int> physical_extruder_types;
    std::vector<int> physical_nozzle_volume_types;
    std::vector<std::string> physical_variant_lists;
};

// Catalogue provenance only. Legality is resolved by build_wizard_candidates(), never by a label.
struct WizardCatalogueRow {
    std::string stable_id;
    std::optional<std::string> source_preset_id;
    double fine_height {0.};
    double coarse_height {0.};
    int ratio {0};
    std::string tier;
    bool stock_provenance {false};
    std::vector<int> source_allowed_ratios;
    std::optional<WizardNativeSourceCompatibility> source_compatibility;
};

enum class WizardCandidateEligibility {
    Eligible,
    NeedsInput,
    Unsupported,
};

// A row's predicted print time from a full headless slice of what Apply would publish. It only
// orders rows and picks the preselected one; Apply never reads it.
enum class WizardEstimateStatus {
    Pending,
    Estimated,
    // The row's slice ran and failed.
    Failed,
    // No estimate, for the reason in `note`.
    Unavailable,
};

struct WizardEstimate {
    WizardEstimateStatus status {WizardEstimateStatus::Pending};
    double seconds {0.};
    // The interval: seconds plus or minus kWizardEstimateMargin.
    double low {0.};
    double high {0.};
    unsigned int switches {0};
    double tower_mm3 {0.};
    std::string note;
    // An Unavailable row whose slice was refused, with the reason the slicer gave.
    bool refused {false};
    std::string refusal;
};

// True when the row's slice failed or was refused, so Apply would give a plate that cannot slice.
inline bool wizard_estimate_unsliceable(const WizardEstimate &estimate)
{
    return estimate.status == WizardEstimateStatus::Failed || estimate.refused;
}

// The G-code processor's error against a real print, as a fraction of the estimate. Fills the
// interval only; rows are not compared by it.
inline constexpr double kWizardEstimateMargin = 0.05;
// Rows are compared as estimates from the same slicer on the same model, so their errors mostly
// cancel. Two times are the same when the slower is within 2% of the faster. Every row comparison
// goes through the two functions below.
inline constexpr double kWizardRowTieFraction = 0.02;
// True when `other` is no more than 2% slower than `reference`.
inline bool wizard_seconds_tie(double reference, double other)
{
    return other <= reference * (1. + kWizardRowTieFraction);
}
// True when `other` is more than 2% slower than `reference`.
inline bool wizard_seconds_slower(double reference, double other)
{
    return !wizard_seconds_tie(reference, other);
}

// An Estimated value with its interval filled in.
WizardEstimate wizard_estimate(double seconds, unsigned int switches, double tower_mm3);

struct WizardCandidate {
    std::string stable_id;
    WizardCandidateKind kind {WizardCandidateKind::MixedCadence};
    MixedNozzleSlicingMode mode {MixedNozzleSlicingMode::Off};
    double fine_height {0.};
    std::optional<double> coarse_height;
    std::optional<int> ratio;
    std::optional<std::string> source_preset_id;
    std::string tier;
    std::vector<WizardKeyDelta> proposed_deltas;
    WizardCandidateEligibility eligibility {WizardCandidateEligibility::NeedsInput};
    std::vector<std::string> reason_codes;
    // The legal ratio interval reported without materializing every row. Diagnostic only; such a
    // row is never applyable.
    std::optional<std::pair<int, int>> unavailable_ratio_range;
    // Empty until the ranking job has looked at this row.
    std::optional<WizardEstimate> estimate;
};

struct WizardReview {
    bool can_apply {false};
    std::optional<WizardCandidateKind> selected_candidate_kind;
    std::vector<WizardKeyDelta> entries;
    std::vector<size_t> affected_plate_ids;
    std::set<std::string> preserved_locks;
    std::vector<std::string> errors;
    std::vector<std::string> warnings;
    // The heights change a process other plates share while this setup is for one plate. Review
    // asks next to the consequence. Set whether or not the draft already carries consent.
    bool needs_shared_consent {false};
    // How many plates share that process, this one included.
    std::size_t shared_plate_count {0};
};

// Takes a copied FullPrintConfig: the fine layer height lives in PrintObjectConfig and the resolver
// fields in PrintConfig. Pure model operations: no preset, plate or slicing side effects and no
// live pointers.
std::vector<WizardCandidate> build_wizard_candidates(
    const WizardDraft &draft,
    const FullPrintConfig &copied_effective_config,
    const std::vector<WizardCatalogueRow> &catalogue_rows);

WizardReview build_wizard_review(const WizardDraft &draft, const WizardCandidate &candidate);

// Pure helpers for the fine-height and cadence UI, testable without a dialog.
//
// preferred_wizard_candidate() never returns a NeedsInput or Unsupported row. It prefers the
// MixedCadence row at `fine_height` with `previous_ratio`, else the first Eligible row.
std::optional<size_t> preferred_wizard_candidate(
    const std::vector<WizardCandidate> &candidates,
    std::optional<int> previous_ratio,
    double fine_height);

// The fine layer height dropdown: the catalogue's distinct fine heights for the resolved fine
// nozzle plus the current one, clamped to that nozzle's envelope. Legality stays with
// build_wizard_candidates().
struct WizardHeightEnvelope {
    double min_height {0.};
    double max_height {0.};
};

std::vector<double> legal_fine_heights(const FullPrintConfig &copied_effective_config,
                                       const std::vector<WizardCatalogueRow> &catalogue_rows,
                                       const WizardHeightEnvelope &fine_envelope);

// The material a filament preset names, without the vendor's nozzle-variant tail. A user suffix
// after the tail is kept; a name with no tail comes back unchanged.
std::string wizard_material_label(const std::string &preset_name);

// What one material choice resolves to, in the Materials page's words. `coarse_role` also checks
// for a coarse-nozzle profile; without one `needs_input` is set and the text says what to change.
struct WizardMaterialResolution {
    std::string text;
    bool needs_input {false};
};
WizardMaterialResolution wizard_material_resolution(const FullPrintConfig &copied_effective_config,
                                                    std::size_t logical_filament, bool coarse_role);

// One 1-based physical extruder per logical slot: the fine slot takes the smaller nozzle, the
// coarse slot the larger, and every other slot keeps its mapping or extruder 1.
std::vector<int> wizard_derived_filament_map(const std::vector<double> &nozzle_diameters,
                                             const std::vector<int> &current_map,
                                             std::size_t filament_count,
                                             std::optional<std::size_t> fine_logical_filament,
                                             std::optional<std::size_t> coarse_logical_filament);

// The Review sentence for the map above.
std::string wizard_filament_map_review_line(const std::vector<int> &derived_map,
                                            std::optional<std::size_t> fine_logical_filament,
                                            std::optional<std::size_t> coarse_logical_filament);

// Only step 1 (Body Split parts), Materials and Detail and speed may hold Next back, and only for a
// choice the wizard cannot resolve itself. `page` is the MixedNozzleWizardPage ordinal.
bool wizard_page_can_block_next(int page);

// What to change before Materials can continue; empty means it can. A non-Manual plate map is never
// a reason, since the wizard writes that map.
std::string wizard_materials_block_reason(const FullPrintConfig &copied_effective_config,
                                          std::optional<std::size_t> fine_logical_filament,
                                          std::optional<std::size_t> coarse_logical_filament,
                                          bool body_split,
                                          const std::vector<int> &body_logical_filaments);

// The mode cards are radio buttons under separate panels, so wxRB_GROUP cannot group them. This is
// the clear-the-others rule without wx.
struct WizardModeSelection {
    bool feature {false};
    bool body {false};
    // A card only reads as its own choice when it is the one that is on.
    bool off {false};
};
WizardModeSelection wizard_select_mode(MixedNozzleSlicingMode mode);
MixedNozzleSlicingMode wizard_selected_mode(const WizardModeSelection &selection);

// The review line for a mode change, such as "Mixed-Nozzle Slicing: Feature Split to Off". Empty
// when the mode is not changing.
std::string wizard_mode_switch_line(MixedNozzleSlicingMode before, MixedNozzleSlicingMode after);

// Off goes from the mode page straight to Review. `page` is the MixedNozzleWizardPage ordinal:
// Materials 1, Detail and speed 2, Review 3, More options 4.
bool wizard_off_skips_page(int page);

// One row per material slot with the colour the sidebar's filament combos paint. The index is the
// slot; a slot with no colour gets the fallback.
inline constexpr const char *WIZARD_MATERIAL_FALLBACK_COLOUR = "#9E9E9EFF";
struct WizardMaterialRow {
    std::size_t slot {0};
    std::string label;
    std::string colour;
};
std::vector<WizardMaterialRow> wizard_material_rows(const std::vector<std::string> &labels,
                                                    const std::vector<std::string> &filament_colours);

// Only the chosen finish's eligible ratios, each on a short label; excluded ratios are explained in
// one sentence, and the single-nozzle baseline is its own line.
struct WizardCadenceRow {
    std::size_t candidate_index {0};
    double coarse_height {0.};
    int ratio {0};
    std::string label;
    // The row's estimate as ranked. Empty when ranking is not running.
    std::optional<WizardEstimate> estimate;
    // This row's slice is running now.
    bool slicing {false};
    // The coarse layer the project already has, marked "(now)".
    bool current {false};
};
struct WizardCadencePage {
    std::vector<WizardCadenceRow> rows;
    std::string exclusions;
    std::string single_nozzle;
    // The all-fine single-nozzle baseline with its time, shown whenever ranking runs.
    std::string baseline;
    // The live line under the list: estimating, fastest first, no clear fastest, or why unranked.
    std::string status;
    // Every row had an estimate, so the rows are fastest first.
    bool ranked {false};
    // The two fastest rows' intervals meet.
    bool no_clear_fastest {false};
    // Index into `rows` of the clear fastest row.
    std::optional<std::size_t> fastest_row;
    // How many rows, from the first, are tied with the fastest. One with a clear fastest, zero when
    // unranked.
    std::size_t fastest_group {0};
    // The all-fine single-nozzle baseline's estimate.
    std::optional<WizardEstimate> baseline_estimate;
    // The one-nozzle slice is running now.
    bool baseline_slicing {false};
};
// With estimates, times are added to the labels and the rows are ranked.
WizardCadencePage wizard_cadence_page(const std::vector<WizardCandidate> &candidates,
                                      double fine_height,
                                      const FullPrintConfig &copied_effective_config,
                                      const std::optional<WizardEstimate> &baseline = std::nullopt);

// The row to select once estimates arrive. A row picked in this session (`explicit_pick`) stays.
// Otherwise a ranked page selects the clear fastest, and on a tie the tied row with the fewest
// nozzle changes, then the first. An unranked page keeps `kept_candidate`. A row that could not be
// sliced is never kept: the selection moves to the first row that slices.
std::optional<std::size_t> wizard_ranked_selection(const WizardCadencePage &page,
                                                   std::optional<std::size_t> kept_candidate,
                                                   bool explicit_pick = false);

// Which nozzle one Body Split body prints on, as the Materials page chose it.
struct WizardBodyAssignment {
    std::size_t object_id {0};
    std::size_t volume_id {0};
    std::size_t logical_filament {0};
    bool coarse {false};
};

// The draft for one row: kind, source preset, heights and ratio, and for Body Split one staged edit
// per body with the height for that body's nozzle.
WizardDraft wizard_candidate_draft(const WizardDraft &draft, const WizardCandidate &candidate,
                                   const std::vector<WizardBodyAssignment> &body);

// The fine heights are the detected fine nozzle's catalogue, with which nozzle it is, what each
// height is for, and which to keep. Labels are short, such as "0.08 mm (finest)".
struct WizardFineHeightRow {
    double height {0.};
    std::string label;
    bool recommended {false};
};
struct WizardFinishPage {
    std::string nozzle_line;
    std::vector<WizardFineHeightRow> rows;
};
WizardFinishPage wizard_finish_page(const FullPrintConfig &copied_effective_config,
                                    const std::vector<double> &offered_heights,
                                    std::optional<std::size_t> fine_logical_filament,
                                    double current_fine_height);
// Empty when a typed height is inside the detected fine nozzle's range.
std::string wizard_custom_fine_height_rejection(const FullPrintConfig &copied_effective_config,
                                                std::optional<std::size_t> fine_logical_filament,
                                                double height);

// What one nozzle diameter prints at, as an installed machine profile states it.
struct WizardNozzleLimitRow {
    double diameter {0.};
    double min_height {0.};
    double max_height {0.};
};

// The nozzle pair and limits the wizard works from. The project owns the pair, not the printer
// variant; a variant naming one diameter for both tools has limits for the wrong nozzle, so the
// machine profile for the project's diameter supplies them and `assumed_extruders` records which. A
// config already on the pair comes back unchanged.
struct WizardPairAlignment {
    bool aligned {false};
    bool changed {false};
    std::vector<double> nozzle_diameters;
    std::vector<double> min_layer_heights;
    std::vector<double> max_layer_heights;
    std::vector<std::size_t> assumed_extruders;
};
WizardPairAlignment wizard_align_to_project_pair(
    const std::vector<double> &config_nozzle_diameters,
    const std::vector<double> &config_min_layer_heights,
    const std::vector<double> &config_max_layer_heights,
    const std::vector<double> &project_pair,
    const std::vector<WizardNozzleLimitRow> &known_limits);

// The review sentence when a limit came from a machine profile rather than the printer preset.
// Empty when nothing was assumed.
std::string wizard_assumed_limits_line(const WizardPairAlignment &alignment);

// One installed printer preset, reduced to what picking a variant needs.
struct WizardPrinterVariantRow {
    std::string name;
    std::string printer_model;
    std::string printer_variant;
    bool visible {true};
    bool system {false};
};

// The variant of the same machine whose nozzle is the project's fine nozzle. Empty when the
// selected preset already names it or no installed or shipped system variant does.
std::string wizard_printer_variant_for_pair(const std::vector<WizardPrinterVariantRow> &rows,
                                            const std::string &selected_name,
                                            const std::string &selected_model,
                                            const std::string &selected_variant,
                                            double fine_diameter);

// The review line for a preset switch at Apply. Empty when the names are equal or either is empty.
std::string wizard_preset_switch_line(const std::string &kind, const std::string &before,
                                      const std::string &after);

// Whether a catalogue row belongs to the project's pair, by its own role diameters rather than the
// selected printer variant. A row with no pair is always offered.
bool wizard_catalogue_row_fits_pair(const WizardCatalogueRow &row,
                                    const std::vector<double> &project_pair);

// The first layer the slice will use, beside the one the process preset states. Setup always
// applies the preset's first layer, so the review line states the preset's numbers and, when they
// differ, what the project had.
struct WizardFirstLayer {
    bool resolved {false};
    bool differs {false};
    double height {0.};
    double speed {0.};
    double preset_height {0.};
    double preset_speed {0.};
    std::string review_line;
};
WizardFirstLayer wizard_first_layer(double effective_height, double effective_speed,
                                    double preset_height, double preset_speed);

// A process value Print::validate refuses under Feature Split. A correctable entry is one Apply can
// switch off or zero itself; the rest are named for the user to change.
struct WizardProcessFix {
    std::string key;
    std::string line;
    bool correctable {false};
};
std::vector<WizardProcessFix> wizard_process_fixes(const ConfigBase &copied_effective_config,
                                                   MixedNozzleSlicingMode mode);

// Step 1's lead line on a project never set up, also where the "?" page starts.
std::string wizard_intro_lead_line();
// The help under the fine layer dropdown, with what a thinner fine layer costs: coarse rows are
// every whole N up to the coarse nozzle's limit, so a thinner fine layer opens a larger N. Body
// Split names the coarse parts, which the coarse nozzle prints whole.
std::string wizard_fine_layer_help(MixedNozzleSlicingMode mode);
// What closing the wizard does to the first-run introduction. Applied after it was shown, it is
// seen for good; cancelled, it is not shown again this session. The "?" page touches neither.
struct WizardIntroOutcome {
    bool persist_seen {false};
    bool suppress_this_session {false};
};
WizardIntroOutcome wizard_intro_outcome(bool intro_line_shown, bool applied);

// A configured project (non-Off mode with both roles assigned) is never re-prompted, whatever the
// per-install flag says. Reopening setup explicitly never consults this.
bool mixed_nozzle_should_show_intro(bool intro_seen, MixedNozzleSlicingMode effective_mode,
                                    bool assignments_present);

// One engine Body Split refusal in plain words: the object, the engine's sentence with internal
// vocabulary replaced and codes left out, and the setting's GUI name. The key is not in the
// sentence; Review links to it. No refusal list is kept here.
std::string wizard_admission_refusal_line(const std::string &engine_message, const std::string &opt_key,
                                          const std::string &object_name);

// What a single-part object needs before Body Split can use it. Empty when there are no names.
std::string wizard_single_part_hint(const std::vector<std::string> &object_names);

// Process families by the MN preset name pattern, "MN Body <fine>-<coarse> ..." and "MN Feature
// <fine>-<coarse> ...". A user preset joins the family of its parent.
enum class WizardProcessFamily {
    Other,
    FeatureSplit,
    BodySplit,
};
WizardProcessFamily wizard_process_family(const std::string &name, const std::string &inherits);

struct WizardProcessPresetRow {
    std::string name;
    std::string inherits;
    double fine_height {0.};
    double coarse_height {0.};
    // Visible, and compatible with the project's pair as the setup staging reads it.
    bool selectable {false};
};

// A Body preset for the project's pair is kept; otherwise the pair's Body preset at the chosen fine
// height (Standard tier among several, never chosen by the coarse height), else its Standard tier,
// else the first by name. `exact_heights` says whether it already carries both heights. Without a
// Body preset for the pair the current preset stays with the heights on top.
struct WizardBodyProcessChoice {
    // The preset Apply selects. Empty keeps the current one.
    std::string preset;
    bool exact_heights {false};
    bool current_is_body {false};
    bool current_is_feature {false};
    bool no_body_preset_for_pair {false};
};
WizardBodyProcessChoice wizard_body_process_choice(const std::vector<WizardProcessPresetRow> &rows,
                                                   const std::string &current_name,
                                                   const std::string &current_inherits,
                                                   const std::vector<double> &project_pair,
                                                   double fine_height,
                                                   std::optional<double> coarse_height);
// The process preset every row is sliced, ranked and applied on, so rows differ only in their two
// heights. The current preset is kept when it is already an MN preset of this mode for the pair;
// otherwise the pair's MN preset of this mode at this fine layer (Standard tier among several),
// else its Standard tier, else the first by name. Feature Split falls back to the MN Body presets.
// With no MN preset, or when the user keeps the current preset, the heights go on top of it.
struct WizardProcessBasis {
    // The preset Apply selects. Empty keeps the current one.
    std::string preset;
    // The current preset is already an MN preset of this mode for the pair.
    bool keeps_current {false};
    // More options: "Keep my current process preset instead".
    bool kept_by_choice {false};
    // No MN preset for this pair and mode.
    bool no_preset_for_pair {false};
};
WizardProcessBasis wizard_process_basis(const std::vector<WizardProcessPresetRow> &rows,
                                        MixedNozzleSlicingMode mode,
                                        const std::string &current_name,
                                        const std::string &current_inherits,
                                        const std::vector<double> &project_pair,
                                        double fine_height,
                                        bool keep_current);
// The line above the coarse layers: which process preset every time uses, and why.
std::string wizard_process_basis_line(const WizardProcessBasis &basis, const std::string &current_name,
                                      const std::vector<double> &project_pair);
// The Review notes for a Feature Split basis switch. Empty when nothing switches.
std::vector<std::string> wizard_process_basis_lines(const WizardProcessBasis &basis,
                                                    const std::string &current_name,
                                                    const std::vector<double> &project_pair,
                                                    double preset_fine_height,
                                                    std::optional<double> preset_coarse_height,
                                                    double fine_height,
                                                    std::optional<double> coarse_height,
                                                    bool can_switch);
// The Review lines for that choice. `can_switch` is false when this setup may not change the
// process preset every plate shares.
std::vector<std::string> wizard_body_process_lines(const WizardBodyProcessChoice &choice,
                                                   const std::string &current_name,
                                                   const std::vector<double> &project_pair,
                                                   double fine_height,
                                                   std::optional<double> coarse_height,
                                                   bool can_switch);
// The sidebar line for a Body Split project that sits on a Feature Split process preset. Empty
// for every other project.
std::string wizard_body_process_guard_text(const WizardBodyProcessChoice &choice,
                                           const std::string &current_name,
                                           const std::vector<double> &project_pair);

// Every string the wizard shows is built below, so the wording can be tested without a dialog.

// One review reason, such as "SharedProcessChangeOutsideScope", in plain words. The code never
// reaches the text.
std::string wizard_review_reason_text(const std::string &reason);

// A setup diagnostic as the post-Apply error box shows it: one sentence that says what to do.
// Anything else passes through unchanged.
std::string wizard_setup_diagnostic_message(const std::string &diagnostic);

// What the two mode cards say for the project's nozzle pair (any order).
struct WizardModeCardText {
    std::string feature;
    std::string body;
};
WizardModeCardText wizard_mode_card_text(const std::vector<double> &nozzle_diameters);

// The Review consent box for a shared process, for `plate_count` plates sharing it.
std::string wizard_shared_consent_label(std::size_t plate_count);
// More options: the whole-project scope item and the note under the material switch, for a project
// of `plate_count` plates.
std::string wizard_whole_project_label(std::size_t plate_count);
std::string wizard_material_settings_note(std::size_t plate_count);

// The coarse layer rows on Detail and speed, labelled by the coarse layer and its N, each with its
// time as it arrives: "0.56 mm  N=7  about 1 h 45 min, 122 nozzle changes".
std::string wizard_speed_row_label(const WizardCadenceRow &row, MixedNozzleSlicingMode mode);
// The line under "Coarse layer": what N is, what a larger N does, and the coarse nozzle's limit.
std::string wizard_coarse_layer_help(MixedNozzleSlicingMode mode, const FullPrintConfig &copied_effective_config);
// "Fastest" on the clear fastest row, "About as fast" on each tied fastest row, empty otherwise.
std::string wizard_speed_row_tag(const WizardCadencePage &page, std::size_t row);
// The one line about the times while they come in, nothing once every row has one, and a plain
// reason otherwise.
std::string wizard_speed_progress_line(const WizardCadencePage &page, bool ranking_available);
// Progress for the bar: rows with an answer (a time, or why none) out of all rows, and whether any
// is still to come.
struct WizardSpeedProgress {
    std::size_t done {0};
    std::size_t total {0};
    bool working {false};
};
WizardSpeedProgress wizard_speed_progress(const WizardCadencePage &page);
// Why Detail and speed holds Next back: the selected row is only the page's default and its time
// has not come in, so the ranking may still pick another row. A row picked by hand goes on. Empty
// when Next can go on.
std::string wizard_speed_block_reason(const WizardCadencePage &page, std::optional<std::size_t> selected_row,
                                      bool explicit_pick, bool ranking_available);
// The lines under the list: one nozzle only and what the fastest choice saves, or that switching
// saves nothing, plus the selected-row note. While one nozzle only has no time, says where it is.
// Empty only when ranking is not running.
std::string wizard_speed_summary_line(const WizardCadencePage &page, double fine_height,
                                      std::optional<std::size_t> selected_row, bool explicit_pick);
// The row that is the project's existing coarse layer, when step 3 is at the existing fine layer.
// Empty on a fresh setup or at another fine layer.
std::optional<std::size_t> wizard_existing_row(const WizardCadencePage &page, double fine_height,
                                               const std::optional<std::pair<double, double>> &existing);
// Mark the row whose slice is running (by candidate stable id), and whether the one-nozzle slice
// is.
void wizard_mark_slicing(WizardCadencePage &page, const std::vector<WizardCandidate> &candidates,
                         const std::optional<std::string> &slicing_row, bool baseline_slicing);
// Mark the project's own coarse layer (wizard_existing_row) as "(now)".
void wizard_mark_current(WizardCadencePage &page, double fine_height,
                         const std::optional<std::pair<double, double>> &existing);
// Setup usually runs before walls and infill are chosen, so once a row has a time, say the times
// are for the preset as it is now and how to see them again. Empty until a row has a time.
std::string wizard_speed_times_note(const WizardCadencePage &page);
// True when every row is slower than one nozzle only, beyond the margin: the page then offers
// "Print with one nozzle instead".
bool wizard_switching_saves_no_time(const WizardCadencePage &page);
// When no coarse layer works with this fine layer, what to do about it, in one sentence.
std::string wizard_no_coarse_layer_line(double fine_height, const FullPrintConfig &copied_effective_config);

// The More options page. Its defaults are the entry's scope, the seeded tower intent and the
// material settings update on when offered, so a setup that never opens it writes the same draft.
struct WizardMoreOptions {
    WizardScopeKind scope {WizardScopeKind::CurrentPlate};
    bool scope_locked {false};
    WizardTowerIntent tower {WizardTowerIntent::Preserve};
    bool update_material_settings {false};
    bool keep_current_process {false};
};
WizardMoreOptions wizard_more_options(const WizardDraft &draft, bool rebind_offered, bool scope_locked);
void wizard_apply_more_options(WizardDraft &draft, const WizardMoreOptions &options);
// "Left 0.2 mm Standard, right 0.8 mm High Flow. Setup keeps these. Change them in the Printer
// panel." Empty without a pair.
std::string wizard_nozzle_flow_line(const std::vector<double> &nozzle_diameters,
                                    const std::vector<std::string> &flow_types);

// The materials the wizard opens with. Fine: the slot the plate's objects use when it is on the
// fine nozzle, else the first slot there. Coarse: the first coarse-nozzle slot with the fine
// material's type, else the first coarse-nozzle slot, else the first other slot (named in the
// note). "On a nozzle" is the plate's filament map; the synced AMS list is not read. Any pair of
// materials is accepted.
struct WizardMaterialDefaults {
    std::optional<std::size_t> fine;
    std::optional<std::size_t> coarse;
    // Says what setup did when no material sat on the coarse nozzle. Empty otherwise.
    std::string note;
    // That nozzle, as the note names it ("the 0.6 mm nozzle"). Empty when a material sat on it.
    std::string empty_nozzle;
};
WizardMaterialDefaults wizard_default_materials(const std::vector<double> &nozzle_diameters,
                                                const std::vector<int> &filament_map,
                                                const std::vector<std::string> &filament_types,
                                                const std::vector<std::size_t> &object_slots);
// The note for an empty coarse nozzle, naming the slot setup puts there: the coarse material picked.
std::string wizard_empty_nozzle_note(const std::string &empty_nozzle, std::size_t coarse_slot);

// Body Split asks per part whether it prints fine or coarse; its material follows from Materials. A
// part on a third material keeps its slot while that slot is on its nozzle, and More options can
// pin any part to an exact slot.
enum class WizardBodyRole {
    Fine,
    Coarse,
};
struct WizardBodyRoleRow {
    std::size_t object_id {0};
    std::size_t volume_id {0};
    // The part's own material slot, 0-based; -1 when it has none.
    int current_slot {-1};
    // The nozzle that slot prints on with the map setup will write, when it resolves.
    std::optional<std::size_t> current_physical;
    // Its size, for the default when it has no nozzle yet.
    double volume {0.};
    bool painted {false};
};
struct WizardBodySlotOverride {
    std::size_t slot {0};
    std::size_t physical {0};
};
// The More options exact slot choice a Body Split part opens on: its own slot (1-based, as the
// choice lists slots after "Set by Fine or Coarse") when that slot is neither the fine nor the coarse
// material, so an exact slot set before reads as set. Otherwise 0, "Set by Fine or Coarse".
int wizard_body_slot_choice(int part_slot, std::optional<std::size_t> fine_slot,
                            std::optional<std::size_t> coarse_slot, std::size_t slot_count);
// Each part's nozzle as it is now. A part with no nozzle goes coarse; if that leaves every part on
// one nozzle, the smallest part goes fine.
std::vector<WizardBodyRole> wizard_default_body_roles(const std::vector<WizardBodyRoleRow> &rows,
                                                      std::size_t fine_physical);
// The assignments Review and Apply read.
std::vector<WizardBodyAssignment> wizard_body_roles_to_slots(
    const std::vector<WizardBodyRoleRow> &rows, const std::vector<WizardBodyRole> &roles,
    std::size_t fine_slot, std::size_t coarse_slot, std::size_t fine_physical, std::size_t coarse_physical,
    const std::vector<std::optional<WizardBodySlotOverride>> &overrides);
// A Body Split part row on step 1: the part's material, its name (with the object's when several
// are listed) and its size. "4: PETG Basic   Body12   87 x 54 x 3 mm".
std::string wizard_part_row_text(const std::string &material_label, const std::string &object_name,
                                 const std::string &part_name, bool several_objects,
                                 double size_x, double size_y, double size_z);
// The one-nozzle refusal in plain terms, for a Body Split object whose parts all print on one
// nozzle: which parts, which nozzle, and what to change. The part to move is the one with the
// finest own layer height, else the last. Empty unless there are two nozzles and two parts.
struct WizardPartOnNozzle {
    std::string name;
    // 0-based slot and the nozzle it prints on (0 left, 1 right).
    std::size_t slot {0};
    std::size_t physical {0};
    // The part's own layer height; 0 when it has none.
    double layer_height {0.};
};
std::string wizard_one_nozzle_parts_line(const std::vector<WizardPartOnNozzle> &parts,
                                         const std::vector<double> &nozzle_diameters);
// The slot each role's parts print with now: the most common part slot, the lower on a tie. Step 2
// defaults to these in Body Split so accepting them changes no part's material. The coarse one is
// empty when it would equal the fine.
struct WizardRoleSlots {
    std::optional<std::size_t> fine;
    std::optional<std::size_t> coarse;
};
// What step 2's pickers show in Body Split, from the parts' own slots. A role with no part on a
// slot keeps its picker's value unless that is the other role's slot. When both roles share one
// slot, it stays with the role whose nozzle it is on (per `filament_map`), and the other role takes
// the closest slot of the same type, one on its own nozzle first, else empty. `note` says what was
// done; empty when the parts do not share a slot.
struct WizardPickerDefaults {
    std::optional<std::size_t> fine;
    std::optional<std::size_t> coarse;
    std::string note;
};
WizardPickerDefaults wizard_body_picker_defaults(const std::vector<WizardBodyRoleRow> &rows,
                                                 const std::vector<WizardBodyRole> &roles,
                                                 const std::vector<double> &nozzle_diameters,
                                                 const std::vector<int> &filament_map,
                                                 const std::vector<std::string> &filament_types,
                                                 const std::vector<std::string> &filament_labels,
                                                 std::optional<std::size_t> fine_pick,
                                                 std::optional<std::size_t> coarse_pick);
// What one object prints with, 1-based as configs store it, 0 when unset: its own slot and each
// part's.
struct WizardObjectSlots {
    int object_slot {0};
    std::vector<int> part_slots;
};
// The slots objects actually print with, 0-based, most used first. An object slot every part
// overrides is only a fallback at the end.
std::vector<std::size_t> wizard_object_material_slots(const std::vector<WizardObjectSlots> &objects);
// One line per part whose material would change: "Body12: 4: PETG Basic to 1: eSUN PLA+".
// `part_names` is parallel to `rows`; `filament_labels` are indexed by 0-based slot.
std::vector<std::string> wizard_body_material_changes(const std::vector<std::string> &part_names,
                                                      const std::vector<WizardBodyRoleRow> &rows,
                                                      const std::vector<WizardBodyAssignment> &body,
                                                      const std::vector<std::string> &filament_labels);
// "Put at least one part on each nozzle." when every part is on one nozzle and none is painted
// with the other; empty otherwise.
std::string wizard_body_roles_block_reason(const std::vector<WizardBodyRoleRow> &rows,
                                           const std::vector<WizardBodyRole> &roles);
// The line under the Body Split parts about joining. Empty when no assembly is joined.
std::string wizard_joining_summary(std::size_t joined_assemblies);
// The mode step 1 opens on: Body Split when the project already is, or when the plate has a
// multi-part or painted object; otherwise Feature Split.
MixedNozzleSlicingMode wizard_default_mode(MixedNozzleSlicingMode configured, bool split_by_part);

// One material's Review line: its nozzle, that nozzle's flow type and the material's speed limit
// there. "2: PLA Basic on the right 0.8 mm High Flow, speed limit 40 mm³/s".
std::string wizard_material_flow_line(const std::string &material, std::optional<std::size_t> physical_extruder,
                                      double nozzle_diameter, const std::string &flow_type,
                                      std::optional<double> speed_limit);

// Where the step that fixes a Review blocker is.
enum class WizardFixTarget {
    None,
    Materials,
    Speed,
    // The consent box on the Review page itself.
    Consent,
    // A Process tab setting, named by `WizardReviewSummary::option_key`.
    ProcessSetting,
};

struct WizardSummaryRow {
    std::string label;
    std::string value;
};

// One side of the supports on the Review page: the slot that prints it (1-based, 0 = Default), the
// nozzle it resolves to, its material label and its filament type.
struct WizardSupportSide {
    int slot {0};
    std::optional<std::size_t> physical;
    std::string material;
    std::string type;
};

// Everything the Review page says, from values the wizard already has. The dialog only lays it out.
struct WizardReviewSummaryInput {
    MixedNozzleSlicingMode mode {MixedNozzleSlicingMode::Off};
    // The project's pair in physical order, left first.
    std::vector<double> nozzle_diameters;
    std::optional<std::size_t> fine_physical;
    std::optional<std::size_t> coarse_physical;
    double fine_height {0.};
    std::optional<double> coarse_height;
    std::string fine_material;
    std::string coarse_material;
    // Body Split: the parts on each nozzle, and the assemblies joined with beams.
    std::vector<std::string> fine_parts;
    std::vector<std::string> coarse_parts;
    std::vector<std::string> joined_assemblies;
    std::optional<WizardEstimate> estimate;
    std::optional<WizardEstimate> baseline;
    // Supports as Apply leaves them: the base and the interface, and the fine material's type for
    // the interface layer hint.
    bool supports {false};
    WizardSupportSide support_base;
    WizardSupportSide support_interface;
    int support_interface_top_layers {0};
    std::string fine_material_type;
    std::string process_before;
    std::string process_after;
    WizardTowerIntent tower_intent {WizardTowerIntent::Preserve};
    // Values Apply corrects on its own, one plain line each.
    std::vector<std::string> also_changed;
    WizardFirstLayer first_layer;
    // Review errors and warnings as reason codes, the engine's refusal line, and a setup
    // diagnostic.
    std::vector<std::string> blockers;
    std::string engine_refusal;
    // The Process setting the refusal names, which "Open the setting" opens.
    std::string engine_refusal_key;
    std::string setup_diagnostic;
    // For "Every setting this changes".
    std::vector<WizardKeyDelta> changes;
    std::vector<std::string> material_lines;
    std::string map_line;
    std::vector<std::string> notes;
    std::vector<std::string> tower_lines;
};

struct WizardReviewSummary {
    std::string header;
    std::vector<WizardSummaryRow> rows;
    std::vector<std::string> also_changed;
    bool ready {false};
    std::string ready_line;
    WizardFixTarget target {WizardFixTarget::None};
    // The link under the ready line, such as "Go to Materials". Empty when there is none.
    std::string target_label;
    // The Process setting to open for a ProcessSetting target.
    std::string option_key;
    // "Every setting this changes": every value Apply writes, the tower block once.
    std::vector<std::string> details;
};
WizardReviewSummary wizard_review_summary(const WizardReviewSummaryInput &input);

} // namespace Slic3r::GUI
