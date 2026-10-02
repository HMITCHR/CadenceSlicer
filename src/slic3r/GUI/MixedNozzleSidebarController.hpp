#ifndef slic3r_GUI_MixedNozzleSidebarController_hpp_
#define slic3r_GUI_MixedNozzleSidebarController_hpp_

// The headless half of the "Mixed Nozzle" sidebar section: when to rebuild, which sub-editor to
// show and which commit route a click takes, with no wxWidgets dependency. It adds no mutation
// primitive; BodySplitEditorModel.hpp and FeatureSplitEditorSupport.hpp own the writes.

#include "BodySplitEditorModel.hpp"
#include "FeatureSplitEditorSupport.hpp"
#include "MixedNozzleNativeEntry.hpp"
#include "MixedNozzleSetupController.hpp"
#include "libslic3r/GCode/GCodeProcessor.hpp"
#include "libslic3r/PresetBundle.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <iomanip>
#include <optional>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

namespace Slic3r::GUI {

// Each logical filament's preset label, or its preset name when the preset is missing.
inline std::vector<std::string> filament_preset_labels(const PresetBundle &bundle)
{
    std::vector<std::string> labels;
    const auto &names = bundle.filament_presets;
    labels.reserve(names.size());
    for (const std::string &name : names) {
        const Preset *preset = bundle.filaments.find_preset(name);
        labels.push_back(preset == nullptr ? name : preset->label(false));
    }
    return labels;
}

// The filament_type of a logical filament's preset, empty when unknown.
inline std::string filament_material_label(const PresetBundle &bundle, size_t logical_index)
{
    const auto &names = bundle.filament_presets;
    if (logical_index >= names.size())
        return {};
    const Preset *preset = bundle.filaments.find_preset(names[logical_index]);
    if (preset == nullptr || !preset->config.has("filament_type"))
        return {};
    return preset->config.opt_string("filament_type", 0);
}

// Which block of the sidebar section is live. Hidden also hides the section title and separator,
// so a single-nozzle printer never shows an empty "Mixed Nozzle" header.
enum class MixedNozzleSidebarSection { Hidden, ModeOnly, Feature, Body };

// Which commit primitive one Apply click routes to. None means "do not mutate, fire no hook": the
// sidebar has no outer OK to fall back on, so a stale target fails closed here.
enum class MixedNozzleSidebarCommitRoute { None, Body, Feature };

// Signature doubles are compared exactly, since the signature detects any change. A non-finite
// height compares equal to itself so the panel does not rebuild on every idle tick.
inline bool mixed_nozzle_sidebar_same_height(double lhs, double rhs)
{
    if (!std::isfinite(lhs) || !std::isfinite(rhs))
        return std::isnan(lhs) == std::isnan(rhs) && (std::isnan(lhs) || lhs == rhs);
    return lhs == rhs;
}

// Per-volume values and ownership revisions used by the editor. Excluded volumes also
// participate so their count/type changes refresh the read-only exclusion caption.
struct MixedNozzleSidebarRowSignature {
    std::size_t object_id {0};
    std::size_t volume_id {0};
    int logical_filament {0};
    double effective_layer_height {0.};
    bool fine_skins {false};
    int fine_skin_layers {0};
    // Names, type, paint revision and explicit/inherited config ownership.
    std::vector<std::string> context;

    bool operator==(const MixedNozzleSidebarRowSignature &rhs) const
    {
        return object_id == rhs.object_id && volume_id == rhs.volume_id &&
               logical_filament == rhs.logical_filament &&
               mixed_nozzle_sidebar_same_height(effective_layer_height, rhs.effective_layer_height) &&
               fine_skins == rhs.fine_skins && fine_skin_layers == rhs.fine_skin_layers &&
               context == rhs.context;
    }
    bool operator!=(const MixedNozzleSidebarRowSignature &rhs) const { return !(*this == rhs); }
};

inline std::vector<MixedNozzleSidebarRowSignature> mixed_nozzle_sidebar_row_signatures(const std::vector<ModelObject *> &objects)
{
    std::vector<MixedNozzleSidebarRowSignature> rows;
    for (const ModelObject *object : objects) {
        if (object == nullptr)
            continue;
        const double base_height = object->config.has("layer_height")
            ? object->config.opt_float("layer_height") : 0.;
        for (const ModelVolume *volume : object->volumes) {
            if (volume == nullptr)
                continue;
            MixedNozzleSidebarRowSignature row;
            row.object_id = object->id().id;
            row.volume_id = volume->id().id;
            row.logical_filament = volume->config.has("extruder") ? volume->config.extruder() : 0;
            const bool has_height = volume->config.has("regional_layer_height");
            const double configured = has_height ? volume->config.opt_float("regional_layer_height") : 0.;
            row.effective_layer_height = (has_height && configured != 0.) ? configured : base_height;
            row.fine_skins = volume->config.has("mixed_nozzle_body_fine_skins") &&
                             volume->config.get().opt_bool("mixed_nozzle_body_fine_skins");
            row.fine_skin_layers = volume->config.has("mixed_nozzle_body_fine_skin_layers")
                ? volume->config.opt_int("mixed_nozzle_body_fine_skin_layers") : 3;
            row.context = {object->name, volume->name, std::to_string(int(volume->type())),
                           std::to_string(volume->mmu_segmentation_facets.timestamp())};
            for (const char *key : {"extruder", "layer_height"})
                row.context.push_back(object->config.has(key) ? object->config.opt_serialize(key) : "<inherit>");
            for (const char *key : {"extruder", "regional_layer_height", "mixed_nozzle_body_fine_skins",
                                   "mixed_nozzle_body_fine_skin_layers"})
                row.context.push_back(volume->config.has(key) ? volume->config.opt_serialize(key) : "<inherit>");
            rows.push_back(row);
        }
    }
    return rows;
}

// Bounded idle input collection: no tool resolution, cadence enumeration or painted-facet scan.
inline std::vector<std::string> mixed_nozzle_sidebar_config_signature(const DynamicPrintConfig &full_config)
{
    std::vector<std::string> values;
    for (const char *key : {"nozzle_diameter", "min_layer_height", "max_layer_height",
                          "extruder_type", "nozzle_volume_type", "filament_extruder_variant",
                          "filament_self_index", "filament_nozzle_map", "filament_volume_map",
                          "mixed_nozzle_allowed_cadence_ratios", "mixed_nozzle_coarse_layer_height",
                          "initial_layer_print_height", "initial_layer_speed", "regional_grid_phase_rule",
                          "enable_prime_tower", "layer_height",
                          "filament_type", "filament_vendor", "filament_colour"})
        values.push_back(full_config.has(key) ? full_config.opt_serialize(key) : "<unset>");
    return values;
}

struct MixedNozzleSidebarSignatureInput {
    BodySplitStableTarget target;
    int current_plate_index {-1};
    std::size_t indexed_plate_id {0};
    MixedNozzleModeSource mode;
    std::size_t filament_preset_count {0};
    std::size_t nozzle_count {0};
    double base_layer_height {0.};
    std::vector<MixedNozzleSidebarRowSignature> rows;
    std::vector<std::string> effective_inputs;
    // What the two tower lines are read from; neither reaches effective_inputs.
    std::size_t slice_revision {0};
    std::vector<std::string> tower_inputs;
    // The section is hidden while every plate is off, so another plate turning on counts.
    bool any_plate_on {true};
};

struct MixedNozzleSidebarSignature {
    bool target_valid {false};
    int plate_index {-1};
    std::size_t plate_id {0};
    MixedNozzleSlicingMode effective {MixedNozzleSlicingMode::Off};
    bool inherits_project {true};
    std::size_t filament_preset_count {0};
    std::size_t nozzle_count {0};
    double base_layer_height {0.};
    std::vector<MixedNozzleSidebarRowSignature> rows;
    std::vector<std::string> effective_inputs;
    std::size_t slice_revision {0};
    std::vector<std::string> tower_inputs;
    bool any_plate_on {true};

    // Every stale target compares equal, so the idle poll is a no-op while the panel is stranded.
    bool operator==(const MixedNozzleSidebarSignature &rhs) const
    {
        if (target_valid != rhs.target_valid)
            return false;
        if (!target_valid)
            return true;
        return plate_index == rhs.plate_index && plate_id == rhs.plate_id &&
               effective == rhs.effective && inherits_project == rhs.inherits_project &&
               filament_preset_count == rhs.filament_preset_count && nozzle_count == rhs.nozzle_count &&
               effective_inputs == rhs.effective_inputs && slice_revision == rhs.slice_revision &&
               tower_inputs == rhs.tower_inputs && any_plate_on == rhs.any_plate_on &&
               mixed_nozzle_sidebar_same_height(base_layer_height, rhs.base_layer_height) &&
               rows == rhs.rows;
    }
    bool operator!=(const MixedNozzleSidebarSignature &rhs) const { return !(*this == rhs); }
};

// The one value every stale/invalid target collapses to.
inline MixedNozzleSidebarSignature mixed_nozzle_sidebar_stale_signature() { return {}; }

// Gates the panel's idle rebuild: equal signature, no rebuild. The target check is the same
// body_split_editor_target_valid() the dialog uses.
inline MixedNozzleSidebarSignature mixed_nozzle_sidebar_signature(const MixedNozzleSidebarSignatureInput &input)
{
    if (!body_split_editor_target_valid(input.target, input.current_plate_index, input.indexed_plate_id))
        return mixed_nozzle_sidebar_stale_signature();
    MixedNozzleSidebarSignature signature;
    signature.target_valid = true;
    signature.plate_index = input.target.plate_index;
    signature.plate_id = input.target.plate_id;
    signature.effective = input.mode.effective;
    signature.inherits_project = input.mode.inherits_project;
    signature.filament_preset_count = input.filament_preset_count;
    signature.nozzle_count = input.nozzle_count;
    signature.effective_inputs = input.effective_inputs;
    signature.slice_revision = input.slice_revision;
    signature.tower_inputs = input.tower_inputs;
    signature.any_plate_on = input.any_plate_on;
    signature.base_layer_height = input.base_layer_height;
    signature.rows = input.rows;
    return signature;
}

// Composed from the dialog's own visibility predicates rather than re-deciding the mode mapping,
// so the sidebar can never show a Body editor where Plate Settings would show a Feature one.
inline MixedNozzleSidebarSection mixed_nozzle_sidebar_section(MixedNozzleSlicingMode effective_mode,
                                                              bool topology_supported)
{
    if (!topology_supported)
        return MixedNozzleSidebarSection::Hidden;
    if (body_split_editor_visible(effective_mode))
        return MixedNozzleSidebarSection::Body;
    if (feature_split_editor_visible(effective_mode))
        return MixedNozzleSidebarSection::Feature;
    return MixedNozzleSidebarSection::ModeOnly;
}

// Fails closed on a stale target: None means the panel must not reach a mutation primitive at
// all, so no snapshot and no notify hook fires. body_split_editor_target_valid() and
// feature_split_target_valid() are the same predicate, so one check covers both routes.
inline MixedNozzleSidebarCommitRoute mixed_nozzle_sidebar_commit_route(const BodySplitStableTarget &target,
                                                                       int current_plate_index,
                                                                       std::size_t indexed_plate_id,
                                                                       MixedNozzleSlicingMode effective_mode)
{
    if (!body_split_editor_target_valid(target, current_plate_index, indexed_plate_id))
        return MixedNozzleSidebarCommitRoute::None;
    if (body_split_editor_visible(effective_mode))
        return MixedNozzleSidebarCommitRoute::Body;
    if (feature_split_editor_visible(effective_mode))
        return MixedNozzleSidebarCommitRoute::Feature;
    return MixedNozzleSidebarCommitRoute::None;
}

// Read-only state used when Reload seeds the applied Feature identities. Keep grouping
// project/plate-owned while the composed process supplies inherited role IDs and capabilities.
// Plate-local applied role IDs still win; no read or Reload writes them back.
inline FeatureSplitEditorState mixed_nozzle_sidebar_applied_feature_state(
    const DynamicPrintConfig &project, const DynamicPrintConfig &full_config,
    const PartPlate &plate, size_t filament_count)
{
    PrintConfig resolver = feature_split_resolver_config_from_full(full_config);
    resolver.filament_map_mode.value = plate.get_real_filament_map_mode(project);
    resolver.filament_map.values = plate.get_real_filament_maps(project);
    const auto *nozzles = full_config.option<ConfigOptionFloats>("nozzle_diameter");
    return build_feature_split_editor_state(plate.get_real_mixed_nozzle_slicing_mode(project),
        nozzles == nullptr ? 0 : nozzles->values.size(),
        plate.get_effective_feature_split_filaments(full_config, filament_count), filament_count, resolver);
}

// Preserve a draft only within its original editor context. Conflicts in that context
// remain visible for explicit Reload, but cannot reach a mutation route.
inline bool mixed_nozzle_sidebar_preserve_draft(const MixedNozzleSidebarSignature &seed,
                                                 const MixedNozzleSidebarSignature &live,
                                                 bool dirty)
{
    return dirty && seed.target_valid == live.target_valid && seed.plate_index == live.plate_index &&
           seed.plate_id == live.plate_id && seed.effective == live.effective &&
           seed.nozzle_count == live.nozzle_count && seed.filament_preset_count == live.filament_preset_count;
}

// Apply is bound to the state from which the controls were seeded.
inline MixedNozzleSidebarCommitRoute mixed_nozzle_sidebar_commit_route(
    const MixedNozzleSidebarSignature &seed, const MixedNozzleSidebarSignature &live)
{
    if (seed != live || !mixed_nozzle_configured_topology_supported(seed.nozzle_count))
        return MixedNozzleSidebarCommitRoute::None;
    return mixed_nozzle_sidebar_commit_route({seed.plate_index, seed.plate_id},
        live.plate_index, live.plate_id, seed.effective);
}

// The lines above the editor are read from the project's tower ledger, the dirty-key mask and
// the current plate's slice result, none of which reaches the signature above.

// Cheap stand-in for "which slice result is on screen". The bounded poll runs it every tick, so
// it counts moves and rounds the estimate rather than walking the move stream the readout walks.
// Zero means this plate has no valid slice.
inline std::size_t mixed_nozzle_sidebar_slice_revision(bool slice_result_valid, int plate_index,
                                                       std::size_t move_count, double estimate_seconds)
{
    if (!slice_result_valid || plate_index < 0)
        return 0;
    return (std::size_t(plate_index) + 1) * 1000003u + move_count +
           std::size_t(std::max(0., estimate_seconds) * 1000.);
}

// The tower status line is a pure function of the ledger option and the dirty-key mask, so
// observing those two is the same as observing the line.
inline std::vector<std::string> mixed_nozzle_sidebar_tower_signature(
    const DynamicPrintConfig &project, const std::vector<std::string> &different_settings_to_system)
{
    std::vector<std::string> values;
    for (const char *key : {MIXED_NOZZLE_TOWER_POLICY_OPTION, MIXED_NOZZLE_TOWER_POLICY_FRESH_OPTION})
        values.push_back(project.has(key) ? project.opt_serialize(key) : "<unset>");
    values.insert(values.end(), different_settings_to_system.begin(),
                  different_settings_to_system.end());
    return values;
}

struct MixedNozzleSidebarTexts {
    // The one-line summary, "Feature Split: fine 0.10 mm on the left 0.2 mm, ...".
    std::string mode_status;
    // Shown only when they apply.
    std::string plate_line;
    std::string guard_line;
    std::string tower_status;
    // "Not sliced yet." until this plate has a slice, then the prime tower readout.
    std::string tower_readout;
    bool sliced {false};
    std::string sliced_line;
    std::string coarse_line;
    std::string rebind_line;

    // For a valid plate every line the section shows has text. A stale target is the one case
    // with nothing to say, and the panel shows its conflict status there instead.
    bool complete() const
    {
        return !mode_status.empty() && !tower_status.empty() && !tower_readout.empty();
    }
};

// A finished slice and a tower write change what the readouts say, not what the editor below
// them is editing. They refresh the lines; they never take an unapplied draft away.
inline bool mixed_nozzle_sidebar_readout_only_change(const MixedNozzleSidebarSignature &seed,
                                                     const MixedNozzleSidebarSignature &live)
{
    if (!seed.target_valid || !live.target_valid)
        return false;
    MixedNozzleSidebarSignature without_readouts = live;
    without_readouts.slice_revision = seed.slice_revision;
    without_readouts.tower_inputs = seed.tower_inputs;
    return without_readouts == seed;
}

// The section is hidden while mixed-nozzle slicing is off on every plate; the Printer row is the
// one place to start. When on, it says what was set up and, after slicing, what the slice did.

inline constexpr const char *MIXED_NOZZLE_SIDEBAR_NOT_SLICED = "Not sliced yet.";

// True when any plate slices with two nozzles. With no plates the project default decides.
inline bool mixed_nozzle_any_plate_on(const DynamicPrintConfig &project, const std::vector<PartPlate *> &plates)
{
    if (plates.empty())
        return mixed_nozzle_project_default(project) != MixedNozzleSlicingMode::Off;
    for (const PartPlate *plate : plates)
        if (plate != nullptr && effective_mixed_nozzle_mode(project, *plate).effective != MixedNozzleSlicingMode::Off)
            return true;
    return false;
}

inline MixedNozzleSidebarSection mixed_nozzle_sidebar_section(MixedNozzleSlicingMode effective_mode,
                                                              bool topology_supported, bool any_plate_on)
{
    if (!any_plate_on)
        return MixedNozzleSidebarSection::Hidden;
    return mixed_nozzle_sidebar_section(effective_mode, topology_supported);
}

// Heights as the wizard prints them: two decimals, "0.10 mm".
inline std::string mixed_nozzle_sidebar_height_text(double height)
{
    std::ostringstream text;
    text << std::fixed << std::setprecision(2) << height;
    return text.str();
}

// A height a user reads at a glance: whole millimetres from 10 mm up, one decimal below.
inline std::string mixed_nozzle_sidebar_plain_mm(double value)
{
    std::ostringstream text;
    text << std::fixed << std::setprecision(value >= 10. ? 0 : 1) << value;
    return text.str();
}

inline std::string mixed_nozzle_sidebar_duration_text(double seconds)
{
    const long minutes = std::max(0L, std::lround(seconds / 60.));
    if (minutes < 60)
        return "about " + std::to_string(std::max(1L, minutes)) + " min";
    const long hours = minutes / 60;
    const long rest = minutes % 60;
    return "about " + std::to_string(hours) + " h" + (rest > 0 ? " " + std::to_string(rest) + " min" : std::string());
}

// The inline editors' layer rows: "Fine layer: 0.08 mm", "Coarse layer: 0.36 mm".
inline std::string mixed_nozzle_sidebar_layer_choice_label(double height, bool fine)
{
    return std::string(fine ? "Fine layer: " : "Coarse layer: ") + mixed_nozzle_sidebar_height_text(height) + " mm";
}

// After slicing: "Prime tower: 36.7 x 37.3 mm, 14.5 cm³, about 21 min".
inline std::string mixed_nozzle_sidebar_tower_readout_line(double width_mm, double depth_mm,
                                                           double volume_mm3, double seconds)
{
    std::ostringstream text;
    text << std::fixed << std::setprecision(1);
    text << "Prime tower: " << width_mm << " x " << depth_mm << " mm, " << volume_mm3 / 1000.
         << " cm\xc2\xb3, " << mixed_nozzle_sidebar_duration_text(seconds);
    return text.str();
}

struct MixedNozzleSidebarBodyCounts {
    std::size_t fine_parts {0};
    std::size_t coarse_parts {0};
    // The coarse parts' layer height; the tallest when parts differ. Zero when there is none.
    double coarse_height {0.};
};

// Body Split rows as (base height, effective height). A part printing at its base height is fine.
inline MixedNozzleSidebarBodyCounts mixed_nozzle_sidebar_body_counts(
    const std::vector<std::pair<double, double>> &base_and_effective)
{
    MixedNozzleSidebarBodyCounts counts;
    for (const auto &[base, effective] : base_and_effective) {
        if (!(effective > base + 1e-6)) {
            ++counts.fine_parts;
            continue;
        }
        ++counts.coarse_parts;
        counts.coarse_height = std::max(counts.coarse_height, effective);
    }
    return counts;
}

// What the lines read beyond the signature. The panel fills it from live state; tests fill it
// by hand.
struct MixedNozzleSidebarFacts {
    std::vector<double> nozzle_diameters;
    double fine_height {0.};
    double coarse_height {0.};
    // Body Split only; both zero when unknown.
    std::size_t fine_parts {0};
    std::size_t coarse_parts {0};
    bool tower_enabled {true};
    // A Body Split project on a Feature Split preset.
    std::string guard_line;
    bool rebind_offered {false};
    // After slicing. The readout comes from mixed_nozzle_sidebar_tower_readout_line.
    std::string tower_readout;
    std::optional<double> print_seconds;
    std::optional<std::size_t> nozzle_changes;
    bool coarse_known {false};
    std::optional<double> coarse_z_low;
    std::optional<double> coarse_z_high;
    std::optional<double> model_height;
};

// The "Sliced:" line's time and nozzle changes, read from the finished G-code the same way for
// Feature and Body Split.
inline void mixed_nozzle_sidebar_read_slice_statistics(const PrintEstimatedStatistics &statistics,
                                                       MixedNozzleSidebarFacts &facts)
{
    facts.print_seconds = double(statistics.modes[0].time);
    facts.nozzle_changes = statistics.total_physical_tool_changes;
}

inline std::string mixed_nozzle_sidebar_nozzle_phrase(const std::vector<double> &nozzles, bool fine)
{
    if (nozzles.size() != 2 || nozzles[0] == nozzles[1] || nozzles[0] <= 0. || nozzles[1] <= 0.)
        return {};
    const std::size_t fine_index = nozzles[0] < nozzles[1] ? 0 : 1;
    const std::size_t index = fine ? fine_index : 1 - fine_index;
    return std::string(" on the ") + (index == 0 ? "left " : "right ") +
           mixed_nozzle_diameter_text(nozzles[index]) + " mm";
}

// "Feature Split: fine 0.10 mm on the left 0.2 mm, coarse 0.40 mm on the right 0.8 mm"
// "Body Split: 2 parts fine at 0.08 mm on the left 0.2 mm, 1 part coarse at 0.16 mm on the right 0.8 mm"
inline std::string mixed_nozzle_sidebar_summary_line(MixedNozzleSlicingMode mode,
                                                     const MixedNozzleSidebarFacts &facts)
{
    if (mode == MixedNozzleSlicingMode::Off)
        return "Off on this plate.";
    const auto height = [](double value, const char *lead) {
        return value > 0. ? std::string(lead) + mixed_nozzle_sidebar_height_text(value) + " mm" : std::string();
    };
    std::string line = std::string(mixed_nozzle_mode_name(mode)) + ": ";
    if (mode == MixedNozzleSlicingMode::FeatureSplit)
        return line + "fine" + height(facts.fine_height, " ") + mixed_nozzle_sidebar_nozzle_phrase(facts.nozzle_diameters, true) +
               ", coarse" + height(facts.coarse_height, " ") + mixed_nozzle_sidebar_nozzle_phrase(facts.nozzle_diameters, false);
    const bool counted = facts.fine_parts + facts.coarse_parts > 0;
    const auto parts = [counted](std::size_t count, const char *role) {
        if (!counted)
            return std::string(role) + " parts";
        return std::to_string(count) + (count == 1 ? " part " : " parts ") + role;
    };
    return line + parts(facts.fine_parts, "fine") + height(facts.fine_height, " at ") +
           mixed_nozzle_sidebar_nozzle_phrase(facts.nozzle_diameters, true) + ", " +
           parts(facts.coarse_parts, "coarse") + height(facts.coarse_height, " at ") +
           mixed_nozzle_sidebar_nozzle_phrase(facts.nozzle_diameters, false);
}

// Empty when the plate does what the project does.
inline std::string mixed_nozzle_sidebar_plate_line(const MixedNozzleModeSource &plate,
                                                   MixedNozzleSlicingMode project_default)
{
    if (plate.inherits_project || plate.effective == project_default)
        return {};
    if (project_default == MixedNozzleSlicingMode::Off)
        return std::string("Only this plate uses ") + mixed_nozzle_mode_name(plate.effective) + ".";
    return std::string("This plate: ") + mixed_nozzle_mode_name(plate.effective) + ". The project uses " +
           mixed_nozzle_mode_name(project_default) + ".";
}

inline std::string mixed_nozzle_sidebar_tower_line(const DynamicPrintConfig &project,
                                                   const std::vector<std::string> &different_settings_to_system,
                                                   bool tower_enabled)
{
    if (!tower_enabled)
        return "Prime tower: off. Two nozzles need it; open Change... to turn it on.";
    const auto ledger = mixed_nozzle_tower_ledger(project);
    if (ledger && !ledger->managed_keys.empty())
        return "Prime tower: set up automatically";
    const std::size_t changed = mixed_nozzle_tower_user_changed_keys(project, different_settings_to_system).size();
    return changed == 0 ? std::string("Prime tower: your own settings")
                        : "Prime tower: your own settings (" + std::to_string(changed) + " changed)";
}

inline MixedNozzleSidebarTexts mixed_nozzle_sidebar_lines(
    const MixedNozzleSidebarSignature &signature, const DynamicPrintConfig &project,
    const std::vector<std::string> &different_settings_to_system, const MixedNozzleSidebarFacts &facts)
{
    MixedNozzleSidebarTexts texts;
    if (!signature.target_valid)
        return texts;
    texts.mode_status = mixed_nozzle_sidebar_summary_line(signature.effective, facts);
    texts.plate_line = mixed_nozzle_sidebar_plate_line({signature.inherits_project, signature.effective},
                                                       mixed_nozzle_project_default(project));
    texts.guard_line = facts.guard_line;
    texts.tower_status = mixed_nozzle_sidebar_tower_line(project, different_settings_to_system, facts.tower_enabled);
    texts.sliced = signature.slice_revision != 0 && !facts.tower_readout.empty();
    texts.tower_readout = texts.sliced ? facts.tower_readout : std::string(MIXED_NOZZLE_SIDEBAR_NOT_SLICED);
    if (texts.sliced) {
        if (facts.print_seconds) {
            texts.sliced_line = "Sliced: " + mixed_nozzle_sidebar_duration_text(*facts.print_seconds);
            if (facts.nozzle_changes)
                texts.sliced_line += ", " + std::to_string(*facts.nozzle_changes) +
                                     (*facts.nozzle_changes == 1 ? " nozzle change" : " nozzle changes");
        }
        if (facts.coarse_known) {
            if (facts.coarse_z_low && facts.coarse_z_high) {
                texts.coarse_line = "Coarse layers from " + mixed_nozzle_sidebar_plain_mm(*facts.coarse_z_low) +
                                    " mm to " + mixed_nozzle_sidebar_plain_mm(*facts.coarse_z_high) + " mm";
                if (facts.model_height && *facts.model_height > 0.)
                    texts.coarse_line += " of " + mixed_nozzle_sidebar_plain_mm(*facts.model_height) + " mm";
            } else
                texts.coarse_line = "No coarse layers in this slice.";
        }
    }
    if (facts.rebind_offered)
        texts.rebind_line = "Some materials still have another nozzle's settings.";
    return texts;
}

// The Printer panel row: "Mixed-nozzle slicing: Off [?] [Set up...]" or
// "Mixed-nozzle slicing: Feature Split [?] [Change...]", with the summary as the tooltip when on.
struct MixedNozzleEntryRowTexts {
    std::string label;
    std::string status;
    std::string button;
    std::string tooltip;
    // True when the button opens setup on its detail step with the current values.
    bool change {false};
};

inline MixedNozzleEntryRowTexts mixed_nozzle_entry_row_texts(const std::vector<MixedNozzleSlicingMode> &plate_modes,
                                                            MixedNozzleSlicingMode project_default,
                                                            const std::string &summary_line)
{
    std::vector<MixedNozzleSlicingMode> modes = plate_modes;
    if (modes.empty())
        modes.push_back(project_default);
    const bool all_same = std::all_of(modes.begin(), modes.end(),
                                      [&modes](MixedNozzleSlicingMode mode) { return mode == modes.front(); });
    MixedNozzleEntryRowTexts texts;
    texts.label = "Mixed-nozzle slicing";
    texts.change = std::any_of(modes.begin(), modes.end(),
                               [](MixedNozzleSlicingMode mode) { return mode != MixedNozzleSlicingMode::Off; });
    texts.status = all_same ? mixed_nozzle_mode_name(modes.front()) : "Differs by plate";
    texts.button = texts.change ? "Change..." : "Set up...";
    if (texts.change)
        texts.tooltip = summary_line;
    return texts;
}

// The Plate Settings row, worded like the Printer row but for one plate.
inline MixedNozzleEntryRowTexts mixed_nozzle_plate_row_texts(const MixedNozzleModeSource &plate,
                                                            MixedNozzleSlicingMode project_default)
{
    MixedNozzleEntryRowTexts texts;
    texts.label = "Mixed-nozzle slicing";
    const std::string differs = mixed_nozzle_sidebar_plate_line(plate, project_default);
    texts.status = differs.empty() ? std::string(mixed_nozzle_mode_name(plate.effective)) : differs;
    texts.change = plate.effective != MixedNozzleSlicingMode::Off;
    texts.button = texts.change ? "Change..." : "Set up...";
    return texts;
}

// The "More" menu, in order. Edit here shows the inline editor below the summary.
struct MixedNozzleSidebarMoreMenu {
    std::string grouping {"Filament grouping..."};
    std::string update_materials {"Update material settings..."};
    std::string edit_here {"Edit here"};
    std::string about {"About mixed-nozzle slicing"};
};

} // namespace Slic3r::GUI

#endif
