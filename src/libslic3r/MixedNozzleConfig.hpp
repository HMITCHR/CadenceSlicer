#pragma once

#include "BoundingBox.hpp"
#include "PrintConfig.hpp"

#include <optional>
#include <utility>
#include <set>
#include <string>
#include <vector>

namespace Slic3r {

class ModelObject;

enum class MixedNozzleDiagnosticCode {
    StaticManualMapRequired,
    LogicalFilamentMissing,
    PhysicalExtruderInvalid,
    NozzleDiameterInvalid,
    NozzleIdentityMissing,
    NozzleIdentityInvalid,
    NozzleVolumeMissing,
    NozzleVolumeInvalid,
    CadenceFineHeightInvalid,
    CadenceCoarseHeightInvalid,
    CadenceRatioInvalid,
    CadenceHeightOutsideEnvelope,
    CadencePhysicalToolsNotDistinct,
    CadenceCoarseNozzleNotLarger,
    // No per-nozzle sibling preset resolved; the selected preset's values are used unchanged.
    FilamentVariantUnresolved,
    // Binding metadata is incomplete or malformed; the project is treated as legacy.
    FilamentBindingUnqualified,
};

enum class MixedNozzleResolveScope { PhysicalToolOnly, CompleteNozzleMetadata };

struct MixedNozzleDiagnostic {
    MixedNozzleDiagnosticCode code;
    size_t logical_filament;
    std::string option_key;
    const char *stable_code() const;
    const char *message() const;
};

struct MixedNozzleResolvedTool {
    size_t logical_filament;
    size_t physical_extruder;
    double nozzle_diameter;
    std::optional<int> physical_nozzle_identity;
    std::optional<NozzleVolumeType> nozzle_volume_type;
    // Static variant column into filament_extruder_variant / filament_self_index (via
    // get_config_index_base); nullopt on a rejected resolution. Matches Print::get_filament_config_indx
    // because mixed-nozzle modes forbid per-layer dynamic remapping.
    std::optional<int> variant_column;
};

struct MixedNozzleToolResolution {
    std::optional<MixedNozzleResolvedTool> tool;
    std::optional<MixedNozzleDiagnostic> diagnostic;
    explicit operator bool() const { return tool.has_value(); }
};

MixedNozzleToolResolution resolve_mixed_nozzle_tool(
    const PrintConfig &config, size_t logical_filament,
    MixedNozzleResolveScope scope = MixedNozzleResolveScope::CompleteNozzleMetadata);

// The nozzle this logical filament's roads are laid with: the resolved physical tool in a
// mixed-nozzle mode, the filament's own column otherwise (as PrintRegion::flow uses). Nothing
// when an active mode cannot resolve the filament.
std::optional<double> mixed_nozzle_flow_nozzle_diameter(const PrintConfig &config, size_t logical_filament);

// Average nozzle diameter of the given feature owners; in mixed-nozzle modes resolved through
// the physical map, nothing if any owner is unresolved.
std::optional<double> mixed_nozzle_nozzle_diameter_average(
    const PrintConfig &config, const std::vector<size_t> &logical_filaments);

// The fine nozzle a logical filament prints on: its nozzle when strictly smaller than the widest
// mapped nozzle. Nothing when mixed-nozzle slicing is off or the filament is unresolved.
std::optional<double> mixed_nozzle_fine_nozzle_diameter(const PrintConfig &config, size_t logical_filament);

// Fine wall ceiling (off by default): walls laid by the fine nozzle print no faster than
// speed * d / reference_nozzle_mm, independent of layer height; the flow cap still applies.
// Covers outer, inner, overhang and first-layer walls and the bases small-perimeter and
// overhang slowdowns scale from. Provisional values.
constexpr bool   mixed_nozzle_fine_wall_ceiling_enabled     = false;
constexpr double mixed_nozzle_fine_wall_reference_nozzle_mm = 0.2;
constexpr double mixed_nozzle_fine_outer_wall_speed_mm_s    = 30.;
constexpr double mixed_nozzle_fine_inner_wall_speed_mm_s    = 40.;

struct MixedNozzleFineWallSpeed {
    double outer_wall;
    double inner_wall;
};

// The ceiling for a fine nozzle of this diameter, regardless of mode or switch.
MixedNozzleFineWallSpeed mixed_nozzle_fine_wall_speed_for_nozzle(double fine_nozzle_diameter);

// The ceiling for walls this filament lays, or nothing when the switch is off or it has no fine nozzle.
std::optional<MixedNozzleFineWallSpeed> mixed_nozzle_fine_wall_speed(const PrintConfig &config,
                                                                     size_t logical_filament);

// Automatic small-loop slowdown on the fine side: a fine wall loop no longer than
// 2 * pi * (radius_per_nozzle * d) gets small_perimeter_speed even when the preset threshold is 0
// (a larger preset threshold wins). The speed is scaled from the flow-capped wall speed.
constexpr bool   mixed_nozzle_fine_small_perimeter_enabled           = true;
constexpr double mixed_nozzle_fine_small_perimeter_radius_per_nozzle = 16.;

// Automatic small perimeter threshold (mm, radius as small_perimeter_threshold), or nothing when
// the switch is off or the filament has no fine nozzle.
std::optional<double> mixed_nozzle_fine_small_perimeter_threshold(const PrintConfig &config, size_t logical_filament);

// Cadence ratios the profile allows; scheduling itself is generic over integer N.
std::vector<int> mixed_nozzle_allowed_cadence_ratios(const PrintConfig &config);

// Cadence ratio for one adaptive-combine window: the largest candidate ratio whose sparse-infill
// intersection area (same index in candidate_intersection_area_mm2) is finite and positive,
// or 1 when none qualify or the inputs are empty or mismatched.
int select_adaptive_cadence_ratio(const std::vector<int> &candidate_ratios,
                                   const std::vector<double> &candidate_intersection_area_mm2);

// Whether a geometrically eligible Feature Split window is worth its tool change:
//   V        = candidate_area_mm2 * ratio * fine_height * (sparse_infill_density_percent / 100)
//   saving_s = V * (1 / q_fine_mm3_s - 1 / q_coarse_mm3_s)
// True iff saving_s > switch_cost_s + roof_cost_s. Never vetoes on non-finite or non-positive
// inputs, or when the total cost is not positive.
bool mixed_nozzle_cadence_pays(double candidate_area_mm2, int ratio, double fine_height,
                               double sparse_infill_density_percent, double q_fine_mm3_s,
                               double q_coarse_mm3_s, double switch_cost_s, double roof_cost_s = 0.);

struct MixedNozzleResolvedCadence {
    MixedNozzleResolvedTool fine_tool;
    MixedNozzleResolvedTool coarse_tool;
    double fine_height;
    double coarse_height;
    int ratio;
};

struct MixedNozzleCadenceResolution {
    std::optional<MixedNozzleResolvedCadence> cadence;
    std::optional<MixedNozzleDiagnostic> diagnostic;
    explicit operator bool() const { return cadence.has_value(); }
};

// Resolve the selected cadence through the physical map. Only a safe integral cadence within
// both tools' height envelopes is accepted; profile qualification metadata is not consulted.
MixedNozzleCadenceResolution resolve_mixed_nozzle_cadence(
    const PrintConfig &config, double fine_height, double coarse_height,
    size_t fine_logical_filament, size_t coarse_logical_filament);

// Restore a dissimilar physical nozzle pair and its per-tool height limits from a loaded project
// after its printer preset is selected. Homogeneous printers stay preset-owned.
bool restore_loaded_mixed_nozzle_nozzle_override(DynamicPrintConfig &project,
                                                 const DynamicPrintConfig &loaded);
bool clear_mixed_nozzle_nozzle_override(DynamicPrintConfig &project);
// The per-filament ledger options a new project starts empty.
inline constexpr const char *MIXED_NOZZLE_FILAMENT_LEDGER_KEYS[] = {
    "mixed_nozzle_filament_explicit_keys", "mixed_nozzle_filament_provenance", "mixed_nozzle_filament_binding"};

// File > New: mixed-nozzle setup is per project. The reset keys return to a new project's state
// and any other project-level mixed_nozzle_ key is erased; printer hardware and loaded filaments
// are untouched. Not used when opening a saved project.
const std::vector<std::string> &mixed_nozzle_new_project_reset_keys();
// Resets the setup keys and erases stray mixed_nozzle_ keys. Returns true when anything changed.
bool reset_mixed_nozzle_setup_for_new_project(DynamicPrintConfig &project);
// File > New, after the ordinary project reset: restore the installed nozzle pair and its
// limits from the previous config, then reset the setup keys.
bool apply_mixed_nozzle_new_project(DynamicPrintConfig &project, const DynamicPrintConfig &previous);

// Whether the two physical tools print the same material (type, vendor and colour), for flush
// volume. False when fewer than two tools are populated or one tool carries several filaments.
bool mixed_nozzle_same_material_pair(const PrintConfig &config);

// Stock OrcaSlicer's support-interface recommendation, reused as is. The four keys it writes.
const std::vector<std::string> &mixed_nozzle_support_interface_recommendation_keys();

// Write the four values Tab::on_value_change recommends for support_interface_filament.
// Returns true when anything changed.
bool mixed_nozzle_write_support_interface_recommendation(DynamicPrintConfig &config);

// Apply the recommendation when stock would recommend it for this project. Off mode does nothing
// (the stock sidebar prompt handles it). Returns true when the project changed.
bool mixed_nozzle_apply_support_interface_recommendation(DynamicPrintConfig &project);

// Composite mask indexing and the project-owned per-filament intent ledger.

// different_settings_to_system is [print, filament 0 .. n-1, printer], so logical filament i is
// at i+1. Always index through these helpers.
constexpr size_t mixed_nozzle_mask_index_print() { return 0; }
size_t mixed_nozzle_mask_index_filament(size_t logical_filament);
size_t mixed_nozzle_mask_length(size_t num_filaments);

// The keys the composite mask records for one logical filament, unescaped. Out-of-range or
// short vectors yield an empty set rather than reading a neighbour's entry.
std::set<std::string> mixed_nozzle_mask_keys_for_filament(const std::vector<std::string> &different_settings,
                                                          size_t logical_filament, size_t num_filaments);

// Per-filament provenance. legacy comes only from importing a project without the metadata; only
// the Rebind transaction writes bound. A project with neither option is fresh and composes as bound.
enum class MixedNozzleProvenance { Legacy, Bound };

const char *mixed_nozzle_provenance_token(MixedNozzleProvenance provenance);
std::optional<MixedNozzleProvenance> mixed_nozzle_parse_provenance(const std::string &token);

// Keys the ledger may carry: filament_options_with_variant plus filament_prime_volume and
// filament_change_length.
const std::set<std::string> &mixed_nozzle_ledger_key_set();

std::string mixed_nozzle_join_ledger_keys(const std::set<std::string> &keys);
std::set<std::string> mixed_nozzle_split_ledger_keys(const std::string &joined);

struct MixedNozzleLedger {
    // False when the stored metadata is malformed. An unqualified project behaves exactly as
    // `legacy` for every logical filament and is never promoted.
    bool qualified {true};
    // True when the project carries no ledger metadata at all: a fresh in-session project.
    bool absent {true};
    std::vector<std::set<std::string>> explicit_keys;
    std::vector<MixedNozzleProvenance> provenance;
    std::optional<MixedNozzleDiagnostic> diagnostic;

    // Composition rule: an unqualified or legacy filament keeps every saved value; a bound
    // filament recomposes everything outside its ledger from the resolved sibling.
    bool is_legacy(size_t logical_filament) const;
    bool is_explicit(size_t logical_filament, const std::string &key) const;
};

// A ledger entry holding both hand-off deposit keys plus at least twenty others was written by an
// older legacy Rebind that recorded every vendor value; no deliberate edit produces one.
bool mixed_nozzle_ledger_entry_is_blanket(const std::set<std::string> &explicit_keys);

// Hand-off deposit intent, shared by CLI and GUI: a bound filament's key is intent when the change
// mask names it, or when a non-blanket ledger entry does.
bool mixed_nozzle_deposit_key_is_intent(const PrintConfig &config, const MixedNozzleLedger &ledger,
                                        size_t logical_filament, size_t num_filaments, const std::string &key);

struct MixedNozzlePrimeVolumes {
    float extruder_change;
    float nozzle_change;
};

// Vendor prime volumes, with per-filament fallbacks and Saving mode.
MixedNozzlePrimeVolumes mixed_nozzle_vendor_prime_volumes(const PrintConfig &config, size_t filament_id);

// Conditioning prime for a switch back to a tool that already holds this filament. Returns the
// per-tool mixed_nozzle_extruder_change_prime_volume (unset: the Saving prime, 15 mm^3), or
// nullopt to keep vendor behaviour. Requires mixed-nozzle slicing, two tools, a real tool switch,
// Default prime mode and a bound filament without explicit filament_prime_volume.
std::optional<float> mixed_nozzle_conditioned_extruder_change_prime(const PrintConfig &config,
                                                                    size_t filament_id,
                                                                    size_t destination_extruder,
                                                                    bool physical_tool_switch,
                                                                    bool destination_holds_same_filament,
                                                                    // On a tool with a hotend rack (the H2C's
                                                                    // right extruder), true when the hotend the
                                                                    // switch lands on is the one already mounted.
                                                                    bool mounted_hotend_stays = false);

// Per-nozzle hand-off deposits for an extruder change, instead of one vendor value for every pair.
struct MixedNozzleHandoffDeposits {
    float ram_length_mm;
    float prime_volume_mm3;
};

// Both deposits for one filament on extruder_id, under mixed-nozzle slicing with two tools.
// nullopt otherwise, or when a hotend-rack swap keeps the vendor route. Each field returns the
// user's explicit value when the ledger records intent, else the formula in the .cpp. The
// optional width and deposition height let auto-pad size for its candidate tower; the larger
// of that height and the nozzle's resolved maximum is used.
std::optional<MixedNozzleHandoffDeposits> mixed_nozzle_handoff_deposits(const PrintConfig &config,
                                                                        size_t filament_id,
                                                                        size_t extruder_id,
                                                                        // Candidate tower width; negative uses prime_tower_width.
                                                                        double candidate_width_mm = -1.,
                                                                        // Captured tower deposition height; negative uses the resolved maximum only.
                                                                        double deposition_height_mm = -1.,
                                                                        // On a tool with a hotend rack, true when the mounted hotend stays in place.
                                                                        bool mounted_hotend_stays = false);

// Whether the mixed-nozzle hand-off applies to extruder_id. A single-hotend tool always qualifies;
// a tool with a hotend rack only when its mounted hotend stays (a rack swap keeps the vendor route).
bool mixed_nozzle_tool_handoff_applies(const PrintConfig &config, size_t extruder_id, bool mounted_hotend_stays);

// The flow type a filament prints with on a tool, which picks its variant column. A Hybrid rack
// takes the filament's filament_volume_map entry, Standard when that is missing or not concrete.
NozzleVolumeType mixed_nozzle_filament_volume_type(const PrintConfig &config, size_t physical_extruder,
                                                   size_t logical_filament);

// Prime for a nozzle returning after idle_seconds parked: never below floor_mm3 (the hand-off
// prime), ramping towards the vendor prime. An unknown wait keeps the floor.
float mixed_nozzle_idle_reprime_volume(const PrintConfig &config, size_t filament_id, float floor_mm3,
                                       double idle_seconds);

// Read and validate the persisted vectors. Both empty means a fresh project (all bound); any
// length mismatch, unknown token or unknown key makes the project unqualified.
MixedNozzleLedger mixed_nozzle_read_ledger(const std::vector<std::string> &explicit_keys,
                                           const std::vector<std::string> &provenance,
                                           size_t num_filaments);

MixedNozzleLedger mixed_nozzle_read_ledger(const DynamicPrintConfig &project, size_t num_filaments);

// Keep the per-filament ledger vectors aligned with the filament list (PresetBundle::set_num_filaments,
// update_num_filaments): erase deleted_index, then resize. Absent vectors stay absent.
void mixed_nozzle_realign_filament_ledger(DynamicPrintConfig &project, size_t num_filaments,
                                          std::optional<size_t> deleted_index = std::nullopt);

// A dissimilar project nozzle pair overrides the printer preset for mixed-nozzle decisions.
const ConfigOptionFloats *mixed_nozzle_effective_nozzle_diameters(const DynamicPrintConfig &project,
                                                                  const DynamicPrintConfig &printer);

// Scale an absolute width by resolved/reference nozzle for Body Split. Percent and auto widths
// are returned unchanged; malformed inputs give nullopt.
std::optional<ConfigOptionFloatOrPercent> body_split_effective_width(
    const ConfigOptionFloatOrPercent &configured_width,
    double resolved_nozzle, double reference_nozzle);

// Candidate filaments for the manual grouping dialog. Mixed-nozzle modes need every configured
// filament, since the map is set before bodies carry them; otherwise the list is unchanged.
std::vector<int> mixed_nozzle_prime_tower_filaments(std::vector<int> used,
    MixedNozzleSlicingMode mode, std::optional<std::pair<int, int>> pair,
    const std::vector<int> &filament_map);

std::vector<int> mixed_nozzle_manual_map_candidates(MixedNozzleSlicingMode effective_mode,
                                                    size_t configured_filament_count,
                                                    const std::vector<int> &plate_used_filaments,
                                                    bool forced_static_map);

struct BodySplitRegionAssignment {
    double cadence;
    size_t logical_filament;
};

// Per-body filament/cadence facts for Body Split, one per logical filament actually on each model
// part (its extruder plus painted colours), from ModelVolume::get_extruders(), and one per modifier
// that names a filament. Printing regions would also count painted-colour ghost regions.
std::vector<BodySplitRegionAssignment> collect_body_split_volume_assignments(
    const ModelObject &model_object, double base_cadence);

// Explicit body bindings only; painted colours do not inherit their parent body cadence.
std::vector<BodySplitRegionAssignment> collect_body_split_body_assignments(
    const ModelObject &model_object, double base_cadence);

// The layer height a painted colour or a modifier's filament prints at inside a Body Split body whose
// own filament and cadence are given: the body's cadence while it stays on the body's nozzle, otherwise
// the cadence of the bodies on its nozzle, or, with no such body, the base height when that pairs with
// the body's cadence. Empty when no qualified height exists. Slicing and admission both ask this.
std::optional<double> resolve_body_split_child_cadence(const PrintConfig &config,
                                                       const std::vector<BodySplitRegionAssignment> &bodies,
                                                       double base_cadence, size_t own_logical,
                                                       double own_cadence, size_t target_logical);

// Whether Body Split synchronizes this object: two or more model parts, or one part that is painted
// or has a modifier naming a filament, printing on more than one physical tool. An unresolved
// filament keeps it on the Body Split path so validation reports the missing mapping.
bool is_body_split_object(const PrintConfig &config, const ModelObject &model_object);

// Validation code a GUI caller raises itself.
inline constexpr const char *MIXED_NOZZLE_BEAM_LAYER_MISMATCH_CODE = "SRL-A28";

// Default {fine, coarse} tool pair: the smallest and largest nozzle, lowest index on a tie.
// Empty with fewer than two tools or all nozzles equal.
std::optional<std::pair<size_t, size_t>> mixed_nozzle_default_tool_pair(const std::vector<double> &nozzle_diameters);

// IDEX copy or mirror mode, where both heads print at once: IDEX_COPY, IDEX_MIRROR,
// START_PRINT COPY=1 / MIRROR=1, M605 S2 / S3. Text after ';' is a comment.
// On rendered G-code:
bool mixed_nozzle_gcode_duplicates_heads(const std::string &rendered_gcode);
// On a start G-code template: only lines outside every {if} count; conditional markers are
// checked on the rendered G-code.
bool mixed_nozzle_start_gcode_duplicates_heads(const std::string &start_gcode);

// Auto brim for a mixed-nozzle prime tower from its height (mm). Steps up faster than Orca's
// linear rule because the tower takes every purge and ram:
//   under 20 mm   3 mm
//   20 to 60 mm   5 mm
//   60 mm and up  8 mm
// Takes no config, so engine, preview and tests agree.
float mixed_nozzle_tower_brim_width(float tower_height_mm);

// Minimum printed pad size (mm) for a mixed-nozzle tower of this height, rib wall included:
//   up to 60 mm    none
//   60 to 100 mm   30 mm
//   over 100 mm    40 mm
// Takes no config, like the brim rule.
float mixed_nozzle_tower_footprint_floor(float tower_height_mm);

// The nominal rectangle needed for the printed pad to reach pad_extent_floor, for square-cornered
// rib tips. WipeTower::rect_floor_for_pad_extent() corrects this for filleted ribs.
float mixed_nozzle_tower_rect_floor(float pad_extent_floor, float rib_width_mm, bool rib_wall);

// Thickness (mm) of the solid base under a mixed-nozzle tower of this height:
//   under 60 mm    1.0 mm
//   60 mm and up   2.0 mm
// Takes no config.
float mixed_nozzle_tower_base_height(float tower_height_mm);

// How far below the part a lagging tower may sit (mm), keeping a fixed margin under the
// nozzle-to-gantry-rod height. Zero means no lag (the tower keeps pace).
constexpr float mixed_nozzle_tower_lag_headroom = 7.f;
float mixed_nozzle_tower_lag_max(float height_to_rod_mm);

// Whether the toolhead can descend over the tower: no object within extruder_clearance_radius of
// the tower footprint (plate-local mm). An undefined footprint or unusable radius answers no.
bool mixed_nozzle_tower_lag_placement_clear(const BoundingBoxf &tower_footprint,
                                            const std::vector<BoundingBoxf> &object_footprints,
                                            float clearance_radius_mm);

// The filament on the widest installed nozzle (the coarse tool that builds a lagging tower), or -1
// when there is none.
int mixed_nozzle_tower_coarse_filament(const PrintConfig &config);

// The physical extruder of the widest installed nozzle, or -1. The lagging schedule asks this,
// since a nozzle may carry several filaments.
int mixed_nozzle_tower_coarse_extruder(const PrintConfig &config);

// Whether the tower deposits only where it has to (wipe_tower_no_sparse_layers). Feature Split
// always does, whatever the project saved; other modes read the option. Never writes it back.
bool mixed_nozzle_compact_tower(const PrintConfig &config);

// True only where the mode is what turned the compact tower on, so the export can say so.
bool mixed_nozzle_compact_tower_applied_by_mode(const PrintConfig &config);

// The same config with the mode set to Off, for planning and writing the prime tower of a Feature
// Split plate that never reaches the coarse nozzle: its tower is built exactly as Off builds it.
PrintConfig mixed_nozzle_tower_config_as_off(const PrintConfig &config);

// Whether this config gets the lagging prime tower; scheduler and tower must agree. Requires the
// compact tower, and is unavailable with smooth timelapse or wrapping detection.
bool mixed_nozzle_tower_lagging(const PrintConfig &config);

// Whether a Body Split plate gets the lagging tower: with a prime tower, when the coarse nozzle
// cannot lay the base or first layer (OwnFirstCell). base_layer_height is the objects' layer height.
bool mixed_nozzle_body_tower_lags(const PrintConfig &config, double base_layer_height);

// Whether a lagging tower is possible at all: not with smooth timelapse or wrapping detection,
// and not without a coarse tool.
bool mixed_nozzle_tower_lagging_available(const PrintConfig &config);

// The minimum layer height of the coarse tool, or 0 when there is none.
double mixed_nozzle_tower_coarse_minimum(const PrintConfig &config);

// Whether one Body Split tool can lay every cell of its body:
//   Shared:       it lays the shared first layer and base rows.
//   OwnFirstCell: its minimum is above the base or first layer, so its body starts with a thicker
//                 first cell of whole base rows and holds later cells to that minimum. With a
//                 prime tower this needs the lagging tower.
//   Unsupported:  no legal plan within the tool's limits.
enum class BodySplitToolEnvelope { Shared, OwnFirstCell, Unsupported };

// The smallest first_layer + j * base (j >= 0) at or above nozzle_minimum: the height of a body's
// first cell when its nozzle cannot lay the shared first layer. first_layer itself when it can.
double body_split_first_cell_height(double first_layer, double base, double nozzle_minimum);

// cadence is the body's own regional layer height; a body at the base cadence is a fine body.
BodySplitToolEnvelope body_split_tool_envelope(const PrintConfig &config, size_t physical_nozzle,
                                               double base, double cadence, double first_layer);

// Shared rule for validation, wizard and editor: OwnFirstCell is legal without a prime tower, or
// with one when the lagging tower is available.
bool body_split_tool_envelope_admitted(BodySplitToolEnvelope envelope, bool with_prime_tower,
                                       bool lagging_tower_available);

// Reduced-flow lead-in for the first bead after the fine tool returns from the tower (Feature
// Split only), taking off the restart blob.
constexpr double mixed_nozzle_tower_return_lead_in_mm   = 1.0;
constexpr double mixed_nozzle_tower_return_lead_in_flow = 0.6;

// The base-cadence (fine) physical nozzle from the Body Split rows: exactly one base row and a
// coarse row on a strictly larger nozzle.
std::optional<double> resolve_body_split_reference_nozzle(
    const PrintConfig &config, double base_cadence,
    const std::vector<BodySplitRegionAssignment> &regions);

// Body Split reference nozzle of one object, from its body bindings plus painted colours
// qualifying at the base cadence. Not thread-safe: get_extruders() may refresh a mutable cache.
std::optional<double> body_split_object_reference_nozzle(
    const PrintConfig &config, const ModelObject &model_object, double base_cadence);

} // namespace Slic3r
