#include "MixedNozzleConfig.hpp"
#include "MixedNozzleProcessConfig.hpp"
#include "Model.hpp"
#include "Slicing.hpp"

#include <cassert>
#include <cmath>
#include <algorithm>
#include <array>
#include <limits>
#include <numeric>
#include <set>
#include <regex>
#include <sstream>

namespace Slic3r {
namespace {
MixedNozzleToolResolution reject(MixedNozzleDiagnosticCode code, size_t logical_filament, const char *option_key)
{
    MixedNozzleToolResolution result;
    result.diagnostic = MixedNozzleDiagnostic{code, logical_filament, option_key};
    return result;
}

MixedNozzleCadenceResolution reject_cadence(MixedNozzleDiagnosticCode code, size_t logical_filament,
                                            const char *option_key)
{
    MixedNozzleCadenceResolution result;
    result.diagnostic = MixedNozzleDiagnostic{code, logical_filament, option_key};
    return result;
}

bool is_within_resolved_height_envelope(const PrintConfig &config, const MixedNozzleResolvedTool &tool,
                                        double height)
{
    const double minimum = resolved_min_layer_height(config, tool.physical_extruder);
    const double maximum = resolved_max_layer_height(config, tool.physical_extruder);
    return std::isfinite(minimum) && std::isfinite(maximum) &&
           (height > minimum || is_approx(height, minimum)) &&
           (height < maximum || is_approx(height, maximum)) &&
           (height < tool.nozzle_diameter || is_approx(height, tool.nozzle_diameter));
}
} // namespace

// Composite mask indexing and the project-owned per-filament intent ledger.

namespace {
const char *kExplicitKeysOption = "mixed_nozzle_filament_explicit_keys";
const char *kProvenanceOption   = "mixed_nozzle_filament_provenance";
const char *kBindingOption      = "mixed_nozzle_filament_binding";

// Erase one entry (when it exists) and then resize to the logical filament count. `fill` is the
// value new trailing entries take.
void realign_string_vector(DynamicPrintConfig &project, const char *key, size_t num_filaments,
                           std::optional<size_t> deleted_index, const std::string &fill)
{
    auto *opt = project.option<ConfigOptionStrings>(key, false);
    if (opt == nullptr || opt->values.empty())
        // The project never wrote this metadata; leave it absent rather than inventing entries.
        return;
    if (deleted_index && *deleted_index < opt->values.size())
        opt->values.erase(opt->values.begin() + *deleted_index);
    opt->values.resize(num_filaments, fill);
}
} // namespace

size_t mixed_nozzle_mask_index_filament(size_t logical_filament)
{
    // The print entry occupies slot 0, so logical filament i is at i+1.
    return logical_filament + 1;
}

size_t mixed_nozzle_mask_length(size_t num_filaments) { return num_filaments + 2; }

std::set<std::string> mixed_nozzle_mask_keys_for_filament(const std::vector<std::string> &different_settings,
                                                          size_t logical_filament, size_t num_filaments)
{
    std::set<std::string> keys;
    if (logical_filament >= num_filaments)
        return keys;
    const size_t index = mixed_nozzle_mask_index_filament(logical_filament);
    if (index >= different_settings.size())
        return keys;
    std::vector<std::string> parsed;
    if (!unescape_strings_cstyle(different_settings[index], parsed))
        return keys;
    for (std::string &key : parsed)
        if (!key.empty())
            keys.insert(std::move(key));
    return keys;
}

const char *mixed_nozzle_provenance_token(MixedNozzleProvenance provenance)
{
    return provenance == MixedNozzleProvenance::Bound ? "bound" : "legacy";
}

std::optional<MixedNozzleProvenance> mixed_nozzle_parse_provenance(const std::string &token)
{
    if (token == "legacy")
        return MixedNozzleProvenance::Legacy;
    if (token == "bound")
        return MixedNozzleProvenance::Bound;
    return std::nullopt;
}

const std::set<std::string> &mixed_nozzle_ledger_key_set()
{
    static const std::set<std::string> keys = [] {
        std::set<std::string> set = filament_options_with_variant;
        // Ordinary per-filament options (no variant overlay) that still need recorded intent, so an
        // explicit value equal to the vendor default survives save, reopen and a vendor default change.
        set.insert("filament_prime_volume");
        set.insert("filament_change_length");
        return set;
    }();
    return keys;
}

std::string mixed_nozzle_join_ledger_keys(const std::set<std::string> &keys)
{
    std::string joined;
    for (const std::string &key : keys) {
        if (!joined.empty())
            joined += ';';
        joined += key;
    }
    return joined;
}

std::set<std::string> mixed_nozzle_split_ledger_keys(const std::string &joined)
{
    std::set<std::string> keys;
    size_t start = 0;
    while (start <= joined.size()) {
        const size_t end = joined.find(';', start);
        const std::string key = joined.substr(start, end == std::string::npos ? std::string::npos : end - start);
        if (!key.empty())
            keys.insert(key);
        if (end == std::string::npos)
            break;
        start = end + 1;
    }
    return keys;
}

bool MixedNozzleLedger::is_legacy(size_t logical_filament) const
{
    if (!qualified)
        return true;
    if (absent)
        return false; // fresh in-session project: bound from the first composition
    return logical_filament >= provenance.size() ||
           provenance[logical_filament] == MixedNozzleProvenance::Legacy;
}

bool MixedNozzleLedger::is_explicit(size_t logical_filament, const std::string &key) const
{
    return logical_filament < explicit_keys.size() && explicit_keys[logical_filament].count(key) > 0;
}

// A value the user changed from the system preset is explicit intent even when the ledger's GUI
// write path never recorded it (3MF load, non-interactive preset apply). The composite
// different_settings_to_system mask is built from config alone, so this needs no GUI path.
static bool mixed_nozzle_key_differs_from_system(const PrintConfig &config, size_t filament_id, size_t num_filaments,
                                                 const char *key)
{
    return mixed_nozzle_mask_keys_for_filament(config.different_settings_to_system.values, filament_id,
                                               num_filaments)
        .count(key) > 0;
}

// Bambu's own reduced prime for this hardware: the "Saving" prime mode value. It is the automatic
// conditioning volume for a switch back to a nozzle that already holds the same filament, and is
// shared with the Saving mode so the two never drift apart.
static constexpr float kMixedNozzleSavingPrimeVolume = 15.f;

// The arrays read below change length depending on who composed the config: a saved project keeps
// per extruder/variant and per filament/variant columns, while full_fff_config and Print::apply
// collapse them. Positional indexing by filament id is therefore wrong; these helpers resolve the
// slot from the layout the array was expanded against, so every composition gives the same answer.

// The slot of a `values` long array that belongs to `id_1based` on the tool at `extruder_id`, given
// the pair of layout vectors the array was expanded against. Nothing when those vectors do not
// describe an array of this length.
static std::optional<size_t> mixed_nozzle_variant_slot(const PrintConfig &config, size_t values,
                                                       size_t extruder_id, int id_1based,
                                                       const std::vector<int> &ids,
                                                       const std::vector<std::string> &variants,
                                                       size_t filament_id)
{
    if (values == 0 || ids.size() != values || variants.size() != values)
        return std::nullopt;
    const ExtruderType extruder_type = extruder_id < config.extruder_type.size() ?
        ExtruderType(config.extruder_type.get_at(extruder_id)) : etDirectDrive;
    // A Hybrid rack takes the column for this filament's own flow type.
    const NozzleVolumeType volume_type = mixed_nozzle_filament_volume_type(config, extruder_id, filament_id);
    const int column = get_config_index_base(volume_type, extruder_type, id_1based, variants, ids);
    if (column < 0 || size_t(column) >= values)
        return std::nullopt;
    return size_t(column);
}

// A printer-owned key (retraction_length, wipe_distance, retraction_distances_when_cut). Its column
// belongs to the physical tool the hand-off happens on. The per extruder and the per extruder and
// variant shapes both carry the printer layout vectors, so one lookup covers them; the third shape
// is the one Print::apply's filament override leaves behind, one entry per filament, and only that
// one is read by the filament.
static size_t mixed_nozzle_tool_slot(const PrintConfig &config, size_t values, size_t filament_id,
                                     size_t num_filaments, size_t extruder_id)
{
    if (values <= 1)
        return 0;
    if (const std::optional<size_t> slot = mixed_nozzle_variant_slot(config, values, extruder_id,
            int(extruder_id) + 1, config.printer_extruder_id.values, config.printer_extruder_variant.values, filament_id))
        return *slot;
    if (values == num_filaments && filament_id < values)
        return filament_id;
    return std::min(extruder_id, values - 1);
}

// A filament-owned key that carries a nozzle-variant overlay: the flow caps this resolver reads. One
// entry per filament is already this filament's own value; anything longer is the per filament and
// variant expansion a project stores, whose column is chosen by the tool the filament prints
// through, exactly as PresetBundle and Print::apply choose it when they collapse the same key.
static size_t mixed_nozzle_material_slot(const PrintConfig &config, size_t values, size_t filament_id,
                                         size_t num_filaments, size_t extruder_id)
{
    if (values <= 1)
        return 0;
    if (values != num_filaments)
        if (const std::optional<size_t> slot = mixed_nozzle_variant_slot(config, values, extruder_id,
                int(filament_id) + 1, config.filament_self_index.values, config.filament_extruder_variant.values,
                filament_id))
            return *slot;
    return std::min(filament_id, values - 1);
}


bool mixed_nozzle_ledger_entry_is_blanket(const std::set<std::string> &explicit_keys)
{
    if (explicit_keys.count("filament_prime_volume") == 0 || explicit_keys.count("filament_change_length") == 0)
        return false;
    const std::set<std::string> &ledger_keys = mixed_nozzle_ledger_key_set();
    size_t others = 0;
    for (const std::string &key : explicit_keys)
        if (key != "filament_prime_volume" && key != "filament_change_length" && ledger_keys.count(key) != 0)
            ++others;
    return others >= 20;
}

bool mixed_nozzle_deposit_key_is_intent(const PrintConfig &config, const MixedNozzleLedger &ledger,
                                        size_t logical_filament, size_t num_filaments, const std::string &key)
{
    const bool bound = ledger.qualified && !ledger.is_legacy(logical_filament) &&
                       logical_filament < ledger.provenance.size() &&
                       ledger.provenance[logical_filament] == MixedNozzleProvenance::Bound;
    if (!bound)
        return false;
    // A value changed from the system preset is intent whatever the ledger says.
    if (mixed_nozzle_key_differs_from_system(config, logical_filament, num_filaments, key.c_str()))
        return true;
    if (!ledger.is_explicit(logical_filament, key))
        return false;
    // An older legacy Rebind recorded every vendor value as intent. Such a blanket entry is not a
    // choice, so it does not make a vendor deposit explicit.
    if (logical_filament < ledger.explicit_keys.size() &&
        mixed_nozzle_ledger_entry_is_blanket(ledger.explicit_keys[logical_filament]))
        return false;
    return true;
}

MixedNozzlePrimeVolumes mixed_nozzle_vendor_prime_volumes(const PrintConfig &config, size_t filament_id)
{
    float wipe_volume_ec = filament_id < config.filament_prime_volume.values.size()
        ? config.filament_prime_volume.values[filament_id]
        : (float) config.prime_volume;
    float wipe_volume_nc = filament_id < config.filament_prime_volume_nc.values.size()
        ? config.filament_prime_volume_nc.values[filament_id]
        : (float) config.prime_volume;
    if (config.prime_volume_mode == PrimeVolumeMode::pvmSaving) {
        wipe_volume_ec = kMixedNozzleSavingPrimeVolume;
        wipe_volume_nc = kMixedNozzleSavingPrimeVolume;
    }
    return {wipe_volume_ec, wipe_volume_nc};
}

NozzleVolumeType mixed_nozzle_filament_volume_type(const PrintConfig &config, size_t physical_extruder,
                                                   size_t logical_filament)
{
    const NozzleVolumeType tool = physical_extruder < config.nozzle_volume_type.size() ?
        NozzleVolumeType(config.nozzle_volume_type.get_at(physical_extruder)) : nvtStandard;
    if (tool != nvtHybrid)
        return tool;
    if (logical_filament < config.filament_volume_map.size()) {
        const int own = config.filament_volume_map.get_at(logical_filament);
        if (own >= int(nvtStandard) && own <= int(nvtMaxNozzleVolumeType) && own != int(nvtHybrid))
            return NozzleVolumeType(own);
    }
    return nvtStandard;
}

bool mixed_nozzle_tool_handoff_applies(const PrintConfig &config, size_t extruder_id, bool mounted_hotend_stays)
{
    const auto &counts = config.extruder_max_nozzle_count;
    if (counts.values.empty())
        return true;
    // One value covers every tool; otherwise each tool has its own.
    const size_t slot = counts.values.size() == 1 ? 0 : extruder_id;
    if (slot >= counts.values.size() || counts.is_nil(slot) || counts.values[slot] <= 1)
        return true;
    return mounted_hotend_stays;
}

std::optional<float> mixed_nozzle_conditioned_extruder_change_prime(const PrintConfig &config,
                                                                    size_t filament_id,
                                                                    size_t destination_extruder,
                                                                    bool physical_tool_switch,
                                                                    bool destination_holds_same_filament,
                                                                    bool mounted_hotend_stays)
{
    if (!physical_tool_switch || !destination_holds_same_filament)
        return std::nullopt;
    if (!is_mixed_nozzle_slicing_enabled(config))
        return std::nullopt;
    if (config.nozzle_diameter.values.size() != 2 || destination_extruder >= 2)
        return std::nullopt;
    // A hotend swap on a rack keeps the vendor nozzle-change route; the other tool is not affected.
    if (!mixed_nozzle_tool_handoff_applies(config, destination_extruder, mounted_hotend_stays))
        return std::nullopt;
    if (config.prime_volume_mode.value != PrimeVolumeMode::pvmDefault)
        return std::nullopt;
    const size_t num_filaments = config.filament_diameter.values.size();
    if (filament_id >= num_filaments)
        return std::nullopt;
    const MixedNozzleLedger ledger = mixed_nozzle_read_ledger(config.mixed_nozzle_filament_explicit_keys.values,
                                                              config.mixed_nozzle_filament_provenance.values,
                                                              num_filaments);
    if (!ledger.qualified || ledger.is_legacy(filament_id) ||
        filament_id >= ledger.provenance.size() ||
        ledger.provenance[filament_id] != MixedNozzleProvenance::Bound ||
        mixed_nozzle_deposit_key_is_intent(config, ledger, filament_id, num_filaments, "filament_prime_volume"))
        return std::nullopt;
    const auto &conditioning = config.mixed_nozzle_extruder_change_prime_volume;
    // Unset: the automatic default is the vendor Saving prime. An explicit per-tool value overrides it.
    if (destination_extruder >= conditioning.values.size() || conditioning.is_nil(destination_extruder))
        return kMixedNozzleSavingPrimeVolume;
    const double value = conditioning.values[destination_extruder];
    if (!std::isfinite(value) || value < 0.)
        return std::nullopt;
    return float(value);
}

// Per-nozzle hand-off deposits:
//   ram_length [mm filament] = clamp((W*h*w + k*Q + (pi/2)*d^3) / A, one-line floor, ceiling)
//   prime_volume [mm^3]      = max(two-line floor, retraction_length*A + k*Q)
// d = nozzle diameter, h = max(resolved max layer height, captured tower deposition height),
// w = d * 1.25 (WipeTower's nozzle-change width ratio), A = filament cross-section, W = wipe_distance,
// k = pressure advance (0 when off), Q = ramming volumetric speed (falls back to max volumetric speed).
// The ceiling is retraction_distances_when_cut: the firmware withdraws that much at the cut anyway.
std::optional<MixedNozzleHandoffDeposits> mixed_nozzle_handoff_deposits(const PrintConfig &config,
                                                                        size_t filament_id,
                                                                        size_t extruder_id,
                                                                        double candidate_width_mm,
                                                                        double deposition_height_mm,
                                                                        bool mounted_hotend_stays)
{
    if (!is_mixed_nozzle_slicing_enabled(config))
        return std::nullopt;
    if (config.nozzle_diameter.values.size() != 2 || extruder_id >= 2)
        return std::nullopt;
    // A hotend swap on a rack keeps the vendor route; single-nozzle setups are untouched.
    if (!mixed_nozzle_tool_handoff_applies(config, extruder_id, mounted_hotend_stays))
        return std::nullopt;
    const size_t num_filaments = config.filament_diameter.values.size();
    // filament_diameter and filament_change_length are ordinary per filament keys and keep one
    // entry per filament in every composition, so a filament past their end is a malformed project.
    if (filament_id >= num_filaments || filament_id >= config.filament_change_length.values.size())
        return std::nullopt;
    // The three printer-owned keys are read through the tool the hand-off happens on, so their
    // length is not a statement about filaments and an eight slot project is not out of range on a
    // two or four column printer array. Empty is still nothing to read.
    if (config.retraction_distances_when_cut.values.empty() || config.retraction_length.values.empty() ||
        config.wipe_distance.values.empty())
        return std::nullopt;
    const size_t cut_slot = mixed_nozzle_tool_slot(config, config.retraction_distances_when_cut.values.size(),
                                                   filament_id, num_filaments, extruder_id);
    const size_t retraction_slot = mixed_nozzle_tool_slot(config, config.retraction_length.values.size(),
                                                          filament_id, num_filaments, extruder_id);
    const size_t wipe_slot = mixed_nozzle_tool_slot(config, config.wipe_distance.values.size(),
                                                    filament_id, num_filaments, extruder_id);

    const MixedNozzleLedger ledger = mixed_nozzle_read_ledger(config.mixed_nozzle_filament_explicit_keys.values,
                                                              config.mixed_nozzle_filament_provenance.values,
                                                              num_filaments);
    const bool ram_is_explicit   = mixed_nozzle_deposit_key_is_intent(config, ledger, filament_id, num_filaments,
                                                                      "filament_change_length");
    const bool prime_is_explicit = mixed_nozzle_deposit_key_is_intent(config, ledger, filament_id, num_filaments,
                                                                      "filament_prime_volume");

    const double A = M_PI / 4. * std::pow(config.filament_diameter.get_at(filament_id), 2.);
    if (!(A > 0.))
        return std::nullopt;
    const double tower_width = std::isfinite(candidate_width_mm) && candidate_width_mm > 0.
        ? candidate_width_mm : config.prime_tower_width.value;
    if (!(tower_width > 0.) || !std::isfinite(tower_width))
        return std::nullopt;
    const double d = config.nozzle_diameter.get_at(extruder_id);
    // Never size a deposited row for a smaller height than the tower step the plan will emit.
    const double resolved_height = resolved_max_layer_height(config, extruder_id);
    const double h = std::isfinite(deposition_height_mm) && deposition_height_mm > 0.
        ? std::max(resolved_height, deposition_height_mm) : resolved_height;
    if (!(h > 0.) || !std::isfinite(h))
        return std::nullopt;
    const double W = config.wipe_distance.get_at(wipe_slot);
    const size_t speed_slot = mixed_nozzle_material_slot(config, config.filament_max_volumetric_speed.values.size(),
                                                         filament_id, num_filaments, extruder_id);
    const size_t ramming_slot = mixed_nozzle_material_slot(config, config.filament_ramming_volumetric_speed.values.size(),
                                                           filament_id, num_filaments, extruder_id);
    double max_vol_speed     = config.filament_max_volumetric_speed.get_at(speed_slot);
    double ramming_vol_speed = config.filament_ramming_volumetric_speed.get_at(ramming_slot);
    if (config.filament_ramming_volumetric_speed.is_nil(ramming_slot) || is_approx(ramming_vol_speed, -1.))
        ramming_vol_speed = max_vol_speed;
    const double Q = ramming_vol_speed;
    const double k = (filament_id < config.enable_pressure_advance.values.size() &&
                       config.enable_pressure_advance.values[filament_id] &&
                       filament_id < config.pressure_advance.values.size())
                         ? config.pressure_advance.values[filament_id]
                         : 0.;
    // Nozzle-change line width: the same width-to-nozzle convention WipeTower's own
    // m_nozzle_change_perimeter_width sizing follows.
    constexpr double kWidthToNozzleRatio = 1.25;
    const double w = d * kWidthToNozzleRatio;
    // One tower line's volume at this layer height and width: the floor for every block.
    const double one_line_volume_mm3 = w * h * tower_width;
    const double one_line_length_mm  = one_line_volume_mm3 / A;

    MixedNozzleHandoffDeposits result{};
    if (ram_is_explicit) {
        result.ram_length_mm = config.filament_change_length.get_at(filament_id);
    } else {
        const double physics_term = (W * h * w + k * Q + (M_PI / 2.) * d * d * d) / A;
        const double ceiling      = config.retraction_distances_when_cut.get_at(cut_slot);
        const double floor_mm     = one_line_length_mm;
        const double clamped      = floor_mm > ceiling ? ceiling : std::clamp(physics_term, floor_mm, ceiling);
        result.ram_length_mm = float(std::max(0., clamped));
    }
    if (prime_is_explicit) {
        result.prime_volume_mm3 = config.filament_prime_volume.get_at(filament_id);
    } else {
        const double retract_length = config.retraction_length.get_at(retraction_slot);
        const double physics_term   = retract_length * A + k * Q; // Ooze volume is unmeasured, left at 0.
        const double floor_mm3      = 2. * one_line_volume_mm3;
        result.prime_volume_mm3     = float(std::max(floor_mm3, physics_term));
    }
    return result;
}

// A parked nozzle loses melt to ooze while it waits. The hand-off prime is sized for a nozzle that
// left a moment ago, the vendor filament_prime_volume for one parked indefinitely; the reprime ramps
// between them from 15 s to 90 s of idle time (thresholds from print testing).
static constexpr double kMixedNozzleReprimeIdleStartSeconds = 15.;
static constexpr double kMixedNozzleReprimeIdleFullSeconds  = 90.;

float mixed_nozzle_idle_reprime_volume(const PrintConfig &config, size_t filament_id, float floor_mm3,
                                       double idle_seconds)
{
    if (!std::isfinite(floor_mm3) || !is_mixed_nozzle_slicing_enabled(config))
        return floor_mm3;
    if (!std::isfinite(idle_seconds) || idle_seconds <= kMixedNozzleReprimeIdleStartSeconds)
        return floor_mm3;
    const float vendor_mm3 = mixed_nozzle_vendor_prime_volumes(config, filament_id).extruder_change;
    if (!std::isfinite(vendor_mm3) || vendor_mm3 <= floor_mm3)
        return floor_mm3;
    const double share = std::min(1., (idle_seconds - kMixedNozzleReprimeIdleStartSeconds) /
                                          (kMixedNozzleReprimeIdleFullSeconds - kMixedNozzleReprimeIdleStartSeconds));
    return floor_mm3 + float(double(vendor_mm3 - floor_mm3) * share);
}

MixedNozzleLedger mixed_nozzle_read_ledger(const std::vector<std::string> &explicit_keys,
                                           const std::vector<std::string> &provenance,
                                           size_t num_filaments)
{
    MixedNozzleLedger ledger;
    ledger.explicit_keys.assign(num_filaments, {});
    ledger.provenance.assign(num_filaments, MixedNozzleProvenance::Bound);
    if (explicit_keys.empty() && provenance.empty()) {
        ledger.absent = true;
        return ledger;
    }
    ledger.absent = false;

    auto unqualify = [&ledger] {
        ledger.qualified = false;
        ledger.diagnostic = MixedNozzleDiagnostic{MixedNozzleDiagnosticCode::FilamentBindingUnqualified, 0,
                                                  kProvenanceOption};
        return ledger;
    };

    // Validation is independent of key presence: a short or over-long vector, an unknown
    // provenance token or an unknown ledger key makes the whole project unqualified.
    if (provenance.size() != num_filaments)
        return unqualify();
    if (!explicit_keys.empty() && explicit_keys.size() != num_filaments)
        return unqualify();

    for (size_t i = 0; i < num_filaments; ++i) {
        const auto parsed = mixed_nozzle_parse_provenance(provenance[i]);
        if (!parsed)
            return unqualify();
        ledger.provenance[i] = *parsed;
    }
    for (size_t i = 0; i < explicit_keys.size(); ++i) {
        std::set<std::string> keys = mixed_nozzle_split_ledger_keys(explicit_keys[i]);
        for (const std::string &key : keys)
            if (mixed_nozzle_ledger_key_set().count(key) == 0)
                return unqualify();
        ledger.explicit_keys[i] = std::move(keys);
    }
    return ledger;
}

MixedNozzleLedger mixed_nozzle_read_ledger(const DynamicPrintConfig &project, size_t num_filaments)
{
    static const std::vector<std::string> empty;
    const auto *keys_opt = project.option<ConfigOptionStrings>(kExplicitKeysOption);
    const auto *prov_opt = project.option<ConfigOptionStrings>(kProvenanceOption);
    return mixed_nozzle_read_ledger(keys_opt ? keys_opt->values : empty,
                                    prov_opt ? prov_opt->values : empty, num_filaments);
}

void mixed_nozzle_realign_filament_ledger(DynamicPrintConfig &project, size_t num_filaments,
                                          std::optional<size_t> deleted_index)
{
    // A newly appended logical filament inherits the project's prevailing provenance: a project
    // still carrying any legacy filament stays legacy for the new one too, so an insert can never
    // be a back door to `bound`.
    std::string provenance_fill = mixed_nozzle_provenance_token(MixedNozzleProvenance::Bound);
    if (const auto *prov = project.option<ConfigOptionStrings>(kProvenanceOption, false))
        for (const std::string &entry : prov->values)
            if (entry != mixed_nozzle_provenance_token(MixedNozzleProvenance::Bound)) {
                provenance_fill = mixed_nozzle_provenance_token(MixedNozzleProvenance::Legacy);
                break;
            }

    realign_string_vector(project, kExplicitKeysOption, num_filaments, deleted_index, std::string());
    realign_string_vector(project, kProvenanceOption, num_filaments, deleted_index, provenance_fill);
    realign_string_vector(project, kBindingOption, num_filaments, deleted_index, std::string());
}

const char *MixedNozzleDiagnostic::stable_code() const
{
    switch (code) {
    case MixedNozzleDiagnosticCode::StaticManualMapRequired: return "MNS-MAP-A01";
    case MixedNozzleDiagnosticCode::LogicalFilamentMissing: return "MNS-MAP-A02";
    case MixedNozzleDiagnosticCode::PhysicalExtruderInvalid: return "MNS-MAP-A03";
    case MixedNozzleDiagnosticCode::NozzleDiameterInvalid: return "MNS-MAP-A04";
    case MixedNozzleDiagnosticCode::NozzleIdentityMissing: return "MNS-MAP-A05";
    case MixedNozzleDiagnosticCode::NozzleIdentityInvalid: return "MNS-MAP-A06";
    case MixedNozzleDiagnosticCode::NozzleVolumeMissing: return "MNS-MAP-A07";
    case MixedNozzleDiagnosticCode::NozzleVolumeInvalid: return "MNS-MAP-A08";
    case MixedNozzleDiagnosticCode::CadenceFineHeightInvalid: return "MNS-CAD-A01";
    case MixedNozzleDiagnosticCode::CadenceCoarseHeightInvalid: return "MNS-CAD-A02";
    case MixedNozzleDiagnosticCode::CadenceRatioInvalid: return "MNS-CAD-A03";
    case MixedNozzleDiagnosticCode::CadenceHeightOutsideEnvelope: return "MNS-CAD-A04";
    case MixedNozzleDiagnosticCode::CadencePhysicalToolsNotDistinct: return "MNS-CAD-A05";
    case MixedNozzleDiagnosticCode::CadenceCoarseNozzleNotLarger: return "MNS-CAD-A06";
    case MixedNozzleDiagnosticCode::FilamentVariantUnresolved: return "MNS-BIND-A01";
    case MixedNozzleDiagnosticCode::FilamentBindingUnqualified: return "MNS-BIND-A02";
    }
    return "MNS-MAP-UNKNOWN";
}

const char *MixedNozzleDiagnostic::message() const
{
    switch (code) {
    case MixedNozzleDiagnosticCode::StaticManualMapRequired: return "Each filament must be assigned to a nozzle by hand before slicing.";
    case MixedNozzleDiagnosticCode::LogicalFilamentMissing: return "This filament is not assigned to a nozzle.";
    case MixedNozzleDiagnosticCode::PhysicalExtruderInvalid: return "This filament is assigned to a nozzle this printer does not have.";
    case MixedNozzleDiagnosticCode::NozzleDiameterInvalid: return "The nozzle this filament prints on has no valid nozzle diameter.";
    case MixedNozzleDiagnosticCode::NozzleIdentityMissing: return "Filaments are assigned to nozzles by hand, but this filament has no nozzle.";
    case MixedNozzleDiagnosticCode::NozzleIdentityInvalid: return "This filament is assigned to a nozzle that does not exist.";
    case MixedNozzleDiagnosticCode::NozzleVolumeMissing: return "Filaments are assigned to nozzles by hand, but this filament has no nozzle flow type.";
    case MixedNozzleDiagnosticCode::NozzleVolumeInvalid: return "This filament has an unknown nozzle flow type.";
    case MixedNozzleDiagnosticCode::CadenceFineHeightInvalid: return "The selected fine layer height must be finite and positive.";
    case MixedNozzleDiagnosticCode::CadenceCoarseHeightInvalid: return "The selected coarse layer height must be finite and positive.";
    case MixedNozzleDiagnosticCode::CadenceRatioInvalid: return "The coarse layer height must be a whole multiple of the fine layer height, at least twice it.";
    case MixedNozzleDiagnosticCode::CadenceHeightOutsideEnvelope: return "A layer height is outside the range one of the nozzles can print.";
    case MixedNozzleDiagnosticCode::CadencePhysicalToolsNotDistinct: return "The fine and coarse materials must print on different nozzles.";
    case MixedNozzleDiagnosticCode::CadenceCoarseNozzleNotLarger: return "The coarse material must print on the larger nozzle.";
    case MixedNozzleDiagnosticCode::FilamentVariantUnresolved: return "No version of this filament preset was found for its nozzle, so the selected preset's values are used unchanged.";
    case MixedNozzleDiagnosticCode::FilamentBindingUnqualified: return "This project's filament to nozzle settings are incomplete, so they are read as an older project.";
    }
    return "The filament to nozzle assignment is not valid.";
}

MixedNozzleToolResolution resolve_mixed_nozzle_tool(const PrintConfig &config, size_t logical_filament,
                                                     MixedNozzleResolveScope scope)
{
    if (!is_static_manual_filament_map_mode(config.filament_map_mode.value))
        return reject(MixedNozzleDiagnosticCode::StaticManualMapRequired, logical_filament, "filament_map_mode");
    if (logical_filament >= config.filament_map.size())
        return reject(MixedNozzleDiagnosticCode::LogicalFilamentMissing, logical_filament, "filament_map");

    const int physical_extruder_1based = config.filament_map.get_at(logical_filament);
    if (physical_extruder_1based <= 0 || size_t(physical_extruder_1based) > config.nozzle_diameter.size())
        return reject(MixedNozzleDiagnosticCode::PhysicalExtruderInvalid, logical_filament, "filament_map");

    const size_t physical_extruder = size_t(physical_extruder_1based - 1);
    const double nozzle_diameter = config.nozzle_diameter.get_at(physical_extruder);
    if (!std::isfinite(nozzle_diameter) || nozzle_diameter <= 0.)
        return reject(MixedNozzleDiagnosticCode::NozzleDiameterInvalid, logical_filament, "nozzle_diameter");

    // Names the invariant: the physical tool comes from filament_map alone (Print.hpp includes this
    // header, so Print::get_extruder_id is out of reach).
    assert(size_t(config.filament_map.get_at(logical_filament) - 1) == physical_extruder);
    MixedNozzleResolvedTool tool{logical_filament, physical_extruder, nozzle_diameter, std::nullopt, std::nullopt, std::nullopt};
    // Static variant column, as Print::get_config_index computes it; configs without variant arrays
    // fall back to the filament index, as get_config_index does.
    const ExtruderType tool_extruder_type = physical_extruder < config.extruder_type.size() ?
        ExtruderType(config.extruder_type.get_at(physical_extruder)) : etDirectDrive;
    const NozzleVolumeType tool_volume_type =
        mixed_nozzle_filament_volume_type(config, physical_extruder, logical_filament);
    tool.variant_column = config.filament_extruder_variant.values.empty() ? int(logical_filament) :
        get_config_index_base(tool_volume_type, tool_extruder_type, int(logical_filament) + 1,
                              config.filament_extruder_variant.values, config.filament_self_index.values);
    if (scope == MixedNozzleResolveScope::CompleteNozzleMetadata && config.filament_map_mode.value == fmmNozzleManual) {
        if (logical_filament >= config.filament_nozzle_map.size())
            return reject(MixedNozzleDiagnosticCode::NozzleIdentityMissing, logical_filament, "filament_nozzle_map");
        const int nozzle_identity = config.filament_nozzle_map.get_at(logical_filament);
        if (nozzle_identity < 0)
            return reject(MixedNozzleDiagnosticCode::NozzleIdentityInvalid, logical_filament, "filament_nozzle_map");
        if (logical_filament >= config.filament_volume_map.size())
            return reject(MixedNozzleDiagnosticCode::NozzleVolumeMissing, logical_filament, "filament_volume_map");
        const int volume_type = config.filament_volume_map.get_at(logical_filament);
        if (volume_type < int(nvtStandard) || volume_type > int(nvtMaxNozzleVolumeType))
            return reject(MixedNozzleDiagnosticCode::NozzleVolumeInvalid, logical_filament, "filament_volume_map");
        tool.physical_nozzle_identity = nozzle_identity;
        tool.nozzle_volume_type = NozzleVolumeType(volume_type);
    }

    MixedNozzleToolResolution result;
    result.tool = std::move(tool);
    return result;
}

std::optional<double> mixed_nozzle_flow_nozzle_diameter(const PrintConfig &config, size_t logical_filament)
{
    if (!is_mixed_nozzle_slicing_enabled(config))
        return config.nozzle_diameter.get_at(logical_filament);
    const MixedNozzleToolResolution resolved = resolve_mixed_nozzle_tool(
        config, logical_filament, MixedNozzleResolveScope::PhysicalToolOnly);
    if (!resolved)
        return std::nullopt;
    return resolved.tool->nozzle_diameter;
}

std::optional<double> mixed_nozzle_nozzle_diameter_average(
    const PrintConfig &config, const std::vector<size_t> &logical_filaments)
{
    if (logical_filaments.empty())
        return std::nullopt;

    double sum = 0.;
    for (const size_t logical_filament : logical_filaments) {
        if (!is_mixed_nozzle_slicing_enabled(config)) {
            sum += config.nozzle_diameter.get_at(logical_filament);
            continue;
        }

        const MixedNozzleToolResolution resolved = resolve_mixed_nozzle_tool(
            config, logical_filament, MixedNozzleResolveScope::PhysicalToolOnly);
        if (!resolved)
            return std::nullopt;
        sum += resolved.tool->nozzle_diameter;
    }
    return sum / double(logical_filaments.size());
}

MixedNozzleFineWallSpeed mixed_nozzle_fine_wall_speed_for_nozzle(double fine_nozzle_diameter)
{
    const double scale = fine_nozzle_diameter / mixed_nozzle_fine_wall_reference_nozzle_mm;
    return {mixed_nozzle_fine_outer_wall_speed_mm_s * scale, mixed_nozzle_fine_inner_wall_speed_mm_s * scale};
}

std::optional<double> mixed_nozzle_fine_nozzle_diameter(const PrintConfig &config, size_t logical_filament)
{
    if (!is_mixed_nozzle_slicing_enabled(config))
        return std::nullopt;
    const MixedNozzleToolResolution own = resolve_mixed_nozzle_tool(
        config, logical_filament, MixedNozzleResolveScope::PhysicalToolOnly);
    if (!own)
        return std::nullopt;
    double widest = 0.;
    for (size_t filament = 0; filament < config.filament_map.size(); ++filament)
        if (const MixedNozzleToolResolution other = resolve_mixed_nozzle_tool(
                config, filament, MixedNozzleResolveScope::PhysicalToolOnly))
            widest = std::max(widest, other.tool->nozzle_diameter);
    if (!(own.tool->nozzle_diameter < widest - EPSILON))
        return std::nullopt;
    return own.tool->nozzle_diameter;
}

std::optional<MixedNozzleFineWallSpeed> mixed_nozzle_fine_wall_speed(const PrintConfig &config,
                                                                     size_t logical_filament)
{
    if (!mixed_nozzle_fine_wall_ceiling_enabled)
        return std::nullopt;
    const std::optional<double> fine = mixed_nozzle_fine_nozzle_diameter(config, logical_filament);
    if (!fine)
        return std::nullopt;
    return mixed_nozzle_fine_wall_speed_for_nozzle(*fine);
}

std::optional<double> mixed_nozzle_fine_small_perimeter_threshold(const PrintConfig &config, size_t logical_filament)
{
    if (!mixed_nozzle_fine_small_perimeter_enabled)
        return std::nullopt;
    const std::optional<double> fine = mixed_nozzle_fine_nozzle_diameter(config, logical_filament);
    if (!fine)
        return std::nullopt;
    return mixed_nozzle_fine_small_perimeter_radius_per_nozzle * *fine;
}

std::vector<int> mixed_nozzle_allowed_cadence_ratios(const PrintConfig &config)
{
    std::vector<int> ratios;
    ratios.reserve(config.mixed_nozzle_allowed_cadence_ratios.size());
    for (int ratio : config.mixed_nozzle_allowed_cadence_ratios.values)
        if (ratio >= 2)
            ratios.emplace_back(ratio);
    std::sort(ratios.begin(), ratios.end());
    ratios.erase(std::unique(ratios.begin(), ratios.end()), ratios.end());
    return ratios;
}

int select_adaptive_cadence_ratio(const std::vector<int> &candidate_ratios,
                                   const std::vector<double> &candidate_intersection_area_mm2)
{
    if (candidate_ratios.size() != candidate_intersection_area_mm2.size() || candidate_ratios.empty())
        return 1;
    std::vector<size_t> order(candidate_ratios.size());
    std::iota(order.begin(), order.end(), size_t(0));
    std::sort(order.begin(), order.end(), [&](size_t a, size_t b) { return candidate_ratios[a] > candidate_ratios[b]; });
    for (size_t idx : order)
        if (candidate_ratios[idx] > 1 && std::isfinite(candidate_intersection_area_mm2[idx]) &&
            candidate_intersection_area_mm2[idx] > 0.)
            return candidate_ratios[idx];
    return 1;
}

bool mixed_nozzle_cadence_pays(double candidate_area_mm2, int ratio, double fine_height,
                               double sparse_infill_density_percent, double q_fine_mm3_s,
                               double q_coarse_mm3_s, double switch_cost_s, double roof_cost_s)
{
    if (!std::isfinite(candidate_area_mm2) || candidate_area_mm2 <= 0. || ratio <= 0 ||
        !std::isfinite(fine_height) || fine_height <= 0. ||
        !std::isfinite(sparse_infill_density_percent) || sparse_infill_density_percent <= 0. ||
        !std::isfinite(q_fine_mm3_s) || q_fine_mm3_s <= 0. ||
        !std::isfinite(q_coarse_mm3_s) || q_coarse_mm3_s <= 0. ||
        !std::isfinite(switch_cost_s) || !std::isfinite(roof_cost_s))
        // No break-even from unresolved or non-physical inputs, so do not veto.
        return true;
    // A non-positive hand-off cost disables the veto, even when the saving is exactly 0.
    if (switch_cost_s + roof_cost_s <= 0.)
        return true;
    const double volume_mm3 = candidate_area_mm2 * double(ratio) * fine_height * (sparse_infill_density_percent / 100.);
    const double saving_s = volume_mm3 * (1. / q_fine_mm3_s - 1. / q_coarse_mm3_s);
    return saving_s > (switch_cost_s + roof_cost_s);
}

MixedNozzleCadenceResolution resolve_mixed_nozzle_cadence(
    const PrintConfig &config, double fine_height, double coarse_height,
    size_t fine_logical_filament, size_t coarse_logical_filament)
{
    if (!std::isfinite(fine_height) || fine_height <= 0.)
        return reject_cadence(MixedNozzleDiagnosticCode::CadenceFineHeightInvalid,
                              fine_logical_filament, "layer_height");
    if (!std::isfinite(coarse_height) || coarse_height <= 0.)
        return reject_cadence(MixedNozzleDiagnosticCode::CadenceCoarseHeightInvalid,
                              coarse_logical_filament, "mixed_nozzle_coarse_layer_height");

    const double raw_ratio = coarse_height / fine_height;
    if (!std::isfinite(raw_ratio) || raw_ratio > double(std::numeric_limits<int>::max()))
        return reject_cadence(MixedNozzleDiagnosticCode::CadenceRatioInvalid,
                              coarse_logical_filament, "mixed_nozzle_coarse_layer_height");
    const int ratio = int(std::lround(raw_ratio));
    if (ratio < 2 || !is_approx(raw_ratio, double(ratio)))
        return reject_cadence(MixedNozzleDiagnosticCode::CadenceRatioInvalid,
                              coarse_logical_filament, "mixed_nozzle_coarse_layer_height");

    const MixedNozzleToolResolution fine = resolve_mixed_nozzle_tool(
        config, fine_logical_filament, MixedNozzleResolveScope::PhysicalToolOnly);
    if (!fine) {
        MixedNozzleCadenceResolution result;
        result.diagnostic = fine.diagnostic;
        return result;
    }
    const MixedNozzleToolResolution coarse = resolve_mixed_nozzle_tool(
        config, coarse_logical_filament, MixedNozzleResolveScope::PhysicalToolOnly);
    if (!coarse) {
        MixedNozzleCadenceResolution result;
        result.diagnostic = coarse.diagnostic;
        return result;
    }
    if (fine.tool->physical_extruder == coarse.tool->physical_extruder)
        return reject_cadence(MixedNozzleDiagnosticCode::CadencePhysicalToolsNotDistinct,
                              coarse_logical_filament, "filament_map");
    if (coarse.tool->nozzle_diameter <= fine.tool->nozzle_diameter)
        return reject_cadence(MixedNozzleDiagnosticCode::CadenceCoarseNozzleNotLarger,
                              coarse_logical_filament, "nozzle_diameter");
    if (!is_within_resolved_height_envelope(config, *fine.tool, fine_height) ||
        !is_within_resolved_height_envelope(config, *coarse.tool, coarse_height))
        return reject_cadence(MixedNozzleDiagnosticCode::CadenceHeightOutsideEnvelope,
                              coarse_logical_filament, "max_layer_height");

    MixedNozzleCadenceResolution result;
    result.cadence = MixedNozzleResolvedCadence{*fine.tool, *coarse.tool, fine_height, coarse_height, ratio};
    return result;
}

bool clear_mixed_nozzle_nozzle_override(DynamicPrintConfig &project)
{
    bool changed = false;
    for (const char *key : {"nozzle_diameter", "min_layer_height", "max_layer_height"})
        changed = project.erase(key) || changed;
    return changed;
}

const std::vector<std::string> &mixed_nozzle_new_project_reset_keys()
{
    // Everything setup (the wizard and the sidebar) writes into the project as setup state:
    // the mode, the automatic tower ownership ledger and its fresh marker, and the per-filament
    // intent ledger, its provenance and the composition report.
    static const std::vector<std::string> keys {
        "mixed_nozzle_slicing_mode",
        "mixed_nozzle_tower_policy_state",
        "mixed_nozzle_tower_policy_fresh",
        "mixed_nozzle_filament_explicit_keys",
        "mixed_nozzle_filament_provenance",
        "mixed_nozzle_filament_binding",
    };
    return keys;
}

bool reset_mixed_nozzle_setup_for_new_project(DynamicPrintConfig &project)
{
    bool changed = false;
    auto put = [&](const std::string &key, ConfigOption *value) {
        std::unique_ptr<ConfigOption> owned(value);
        const ConfigOption *current = project.option(key);
        if (current != nullptr && *current == *owned)
            return;
        project.set_key_value(key, owned.release());
        changed = true;
    };
    put("mixed_nozzle_slicing_mode", new ConfigOptionEnum<MixedNozzleSlicingMode>(MixedNozzleSlicingMode::Off));
    // A new project owns no tower settings yet and is fresh, the same state
    // PresetBundle::reset_project_embedded_presets() leaves.
    changed = project.erase("mixed_nozzle_tower_policy_state") || changed;
    put("mixed_nozzle_tower_policy_fresh", new ConfigOptionBool(true));
    for (const char *key : MIXED_NOZZLE_FILAMENT_LEDGER_KEYS)
        put(key, new ConfigOptionStrings());
    // A stray project-level mixed-nozzle key (for example a legacy project-layer copy of the
    // coarse height) would shadow the process preset, so it is erased rather than defaulted.
    const auto &reset = mixed_nozzle_new_project_reset_keys();
    std::vector<std::string> stray;
    for (const std::string &key : project.keys())
        if (key.rfind("mixed_nozzle_", 0) == 0 && std::find(reset.begin(), reset.end(), key) == reset.end())
            stray.push_back(key);
    for (const std::string &key : stray)
        changed = project.erase(key) || changed;
    return changed;
}

bool apply_mixed_nozzle_new_project(DynamicPrintConfig &project, const DynamicPrintConfig &previous)
{
    // PresetBundle::reset_project_embedded_presets() clears the pair on every reset, which is
    // right before a load (the file brings its own pair) but not for a new project on the same
    // printer: put the installed pair and its limits back. An invalid or absent pair stays absent.
    bool changed = false;
    const auto *before = project.option<ConfigOptionFloats>("nozzle_diameter");
    const std::vector<double> before_pair = before ? before->values : std::vector<double>{};
    restore_loaded_mixed_nozzle_nozzle_override(project, previous);
    const auto *after = project.option<ConfigOptionFloats>("nozzle_diameter");
    changed = (after ? after->values : std::vector<double>{}) != before_pair;
    changed = reset_mixed_nozzle_setup_for_new_project(project) || changed;
    return changed;
}

bool restore_loaded_mixed_nozzle_nozzle_override(DynamicPrintConfig &project,
                                                 const DynamicPrintConfig &loaded)
{
    const auto *loaded_nozzles = loaded.option<ConfigOptionFloats>("nozzle_diameter");
    if (loaded_nozzles == nullptr || loaded_nozzles->values.size() != 2)
        return clear_mixed_nozzle_nozzle_override(project);
    const double first = loaded_nozzles->values[0];
    const double second = loaded_nozzles->values[1];
    if (!std::isfinite(first) || first <= 0. || !std::isfinite(second) || second <= 0. || first == second)
        return clear_mixed_nozzle_nozzle_override(project);

    bool changed = false;
    const auto *project_nozzles = project.option<ConfigOptionFloats>("nozzle_diameter");
    if (project_nozzles == nullptr || project_nozzles->values != loaded_nozzles->values) {
        project.set_key_value("nozzle_diameter", new ConfigOptionFloats(loaded_nozzles->values));
        changed = true;
    }
    for (const char *key : {"min_layer_height", "max_layer_height"}) {
        const auto *limits = loaded.option<ConfigOptionFloats>(key);
        if (limits == nullptr || limits->values.empty()) {
            changed = project.erase(key) || changed;
            continue;
        }
        const auto *previous = project.option<ConfigOptionFloats>(key);
        if (previous == nullptr || previous->values != limits->values) {
            project.set_key_value(key, limits->clone());
            changed = true;
        }
    }
    return changed;
}

const ConfigOptionFloats *mixed_nozzle_effective_nozzle_diameters(const DynamicPrintConfig &project,
                                                                   const DynamicPrintConfig &printer)
{
    const auto *printer_nozzles = printer.option<ConfigOptionFloats>("nozzle_diameter");
    if (printer_nozzles == nullptr || printer_nozzles->values.size() != 2)
        return printer_nozzles;
    if (const auto *project_nozzles = project.option<ConfigOptionFloats>("nozzle_diameter");
        project_nozzles != nullptr && project_nozzles->values.size() == 2) {
        const double first = project_nozzles->values[0];
        const double second = project_nozzles->values[1];
        if (std::isfinite(first) && first > 0. && std::isfinite(second) && second > 0. && first != second)
            return project_nozzles;
    }
    return printer_nozzles;
}

namespace {

constexpr const char *kMixedNozzleProcessRoleMetadata = "mixed_nozzle_process_nozzle_diameters";

MixedNozzleProcessRoleResolution reject_process_roles(const char *diagnostic)
{
    MixedNozzleProcessRoleResolution result;
    result.diagnostic = diagnostic;
    return result;
}

bool valid_nozzle_pair(const std::vector<double> &values)
{
    return values.size() == 2 && std::isfinite(values[0]) && values[0] > 0. &&
           std::isfinite(values[1]) && values[1] > 0. && !is_approx(values[0], values[1]);
}

std::vector<std::string> split_variant_tokens(const std::string &value)
{
    std::vector<std::string> tokens;
    size_t start = 0;
    while (start <= value.size()) {
        const size_t end = value.find(',', start);
        std::string token = value.substr(start, end == std::string::npos ? std::string::npos : end - start);
        const size_t first = token.find_first_not_of(" \t\r\n");
        const size_t last = token.find_last_not_of(" \t\r\n");
        if (first == std::string::npos)
            token.clear();
        else
            token = token.substr(first, last - first + 1);
        tokens.emplace_back(std::move(token));
        if (end == std::string::npos)
            break;
        start = end + 1;
    }
    return tokens;
}

bool is_known_variant_token(const std::string &token)
{
    for (int type = etDirectDrive; type <= etMaxExtruderType; ++type)
        for (const NozzleVolumeType volume_type : get_valid_nozzle_volume_type())
            if (token == get_extruder_variant_string(ExtruderType(type), volume_type))
                return true;
    return false;
}

ExtruderType process_physical_extruder_type(const DynamicPrintConfig &printer, size_t physical_extruder)
{
    const auto *types = printer.option<ConfigOptionEnumsGeneric>("extruder_type");
    return types != nullptr && physical_extruder < types->values.size() ?
        ExtruderType(types->values[physical_extruder]) : etDirectDrive;
}

} // namespace

MixedNozzleProcessRoleResolution resolve_mixed_nozzle_process_roles(
    const DynamicPrintConfig &process, const DynamicPrintConfig &project,
    const DynamicPrintConfig &printer, MixedNozzleProcessColumnOrder column_order)
{
    const auto *metadata = process.option<ConfigOptionFloats>(kMixedNozzleProcessRoleMetadata);
    if (metadata == nullptr || metadata->values.size() != 2)
        return reject_process_roles("metadata missing or not exactly two values; using legacy physical columns");
    if (!valid_nozzle_pair(metadata->values))
        return reject_process_roles("metadata must contain two finite, positive, distinct diameters; using legacy physical columns");

    const ConfigOptionFloats *physical_nozzles = mixed_nozzle_effective_nozzle_diameters(project, printer);
    if (physical_nozzles == nullptr || !valid_nozzle_pair(physical_nozzles->values))
        return reject_process_roles("effective physical nozzle pair is missing or invalid; using legacy physical columns");

    MixedNozzleProcessRoleMapping mapping{};
    for (size_t role = 0; role < 2; ++role) {
        size_t match = 2;
        for (size_t physical = 0; physical < 2; ++physical) {
            if (!is_approx(metadata->values[role], physical_nozzles->values[physical]))
                continue;
            if (match != 2)
                return reject_process_roles("metadata matches more than one physical nozzle; using legacy physical columns");
            match = physical;
        }
        if (match == 2)
            return reject_process_roles("metadata does not match the effective physical nozzle pair; using legacy physical columns");
        mapping.role_to_physical[role] = match;
        mapping.physical_to_role[match] = role;
    }

    // The source process may contain an extra, known variant that is inactive on the selected
    // machine (for example TPU High Flow on the coarse source). Validate such columns without
    // requiring them to occur in the currently selected printer inventory; only the active volume
    // type(s) below are applicable to this composition.
    const auto *physical_variants = printer.option<ConfigOptionStrings>("extruder_variant_list");
    if (physical_variants != nullptr) {
        for (size_t physical = 0; physical < 2 && physical < physical_variants->values.size(); ++physical) {
            const auto tokens = split_variant_tokens(physical_variants->values[physical]);
            if (tokens.empty() || std::any_of(tokens.begin(), tokens.end(), [](const std::string &token) { return token.empty(); }))
                return reject_process_roles("physical variant inventory contains an empty token; using legacy physical columns");
            std::set<std::string> seen;
            for (const std::string &token : tokens) {
                if (!is_known_variant_token(token))
                    return reject_process_roles("physical variant inventory contains an unknown token; using legacy physical columns");
                if (!seen.insert(token).second)
                    return reject_process_roles("physical variant inventory contains duplicate tokens; using legacy physical columns");
            }
        }
    }

    std::array<std::set<std::string>, 2> applicable;
    const auto *volume_types = project.option<ConfigOptionEnumsGeneric>("nozzle_volume_type");
    if (volume_types == nullptr || volume_types->values.size() < 2)
        volume_types = printer.option<ConfigOptionEnumsGeneric>("nozzle_volume_type");
    for (size_t physical = 0; physical < 2; ++physical) {
        NozzleVolumeType volume_type = nvtStandard;
        if (volume_types != nullptr && physical < volume_types->values.size()) {
            const int raw_volume_type = volume_types->values[physical];
            if (raw_volume_type < int(nvtStandard) || raw_volume_type > int(nvtMaxNozzleVolumeType))
                return reject_process_roles("physical nozzle volume type is invalid; using legacy physical columns");
            volume_type = NozzleVolumeType(raw_volume_type);
        }
        const ExtruderType extruder_type = process_physical_extruder_type(printer, physical);
        if (volume_type == nvtHybrid) {
            applicable[mapping.physical_to_role[physical]].insert(
                get_extruder_variant_string(extruder_type, nvtStandard));
            applicable[mapping.physical_to_role[physical]].insert(
                get_extruder_variant_string(extruder_type, nvtHighFlow));
        } else {
            applicable[mapping.physical_to_role[physical]].insert(
                get_extruder_variant_string(extruder_type, volume_type));
        }
    }

    const auto *ids = process.option<ConfigOptionInts>("print_extruder_id");
    const auto *variants = process.option<ConfigOptionStrings>("print_extruder_variant");
    if (ids == nullptr || variants == nullptr || ids->values.empty() || ids->values.size() != variants->values.size())
        return reject_process_roles("process role columns are missing or have mismatched lengths; using legacy physical columns");

    std::array<std::set<std::string>, 2> present;
    for (size_t column = 0; column < ids->values.size(); ++column) {
        const int role_id = ids->values[column];
        if (role_id < 1 || role_id > 2)
            return reject_process_roles("process role IDs must be one or two; using legacy physical columns");
        const size_t role = column_order == MixedNozzleProcessColumnOrder::PhysicalTools ?
            mapping.physical_to_role[size_t(role_id - 1)] : size_t(role_id - 1);
        std::string token = variants->values[column];
        const size_t first = token.find_first_not_of(" \t\r\n");
        const size_t last = token.find_last_not_of(" \t\r\n");
        if (first == std::string::npos)
            return reject_process_roles("process role columns contain an empty token; using legacy physical columns");
        token = token.substr(first, last - first + 1);
        if (!is_known_variant_token(token))
            return reject_process_roles("process role columns contain an unknown token; using legacy physical columns");
        if (!present[role].insert(token).second)
            return reject_process_roles("process role columns contain duplicate tokens; using legacy physical columns");
    }

    for (size_t role = 0; role < 2; ++role)
        for (const std::string &token : applicable[role])
            if (present[role].count(token) == 0)
                return reject_process_roles("process role columns are incomplete for the active physical variants; using legacy physical columns");

    MixedNozzleProcessRoleResolution result;
    result.mapping = mapping;
    return result;
}

std::optional<size_t> mixed_nozzle_process_role_for_physical_tool(
    const DynamicPrintConfig &process, const DynamicPrintConfig &project,
    const DynamicPrintConfig &printer, size_t physical_extruder)
{
    if (physical_extruder >= 2)
        return std::nullopt;
    const MixedNozzleProcessRoleResolution resolved = resolve_mixed_nozzle_process_roles(process, project, printer);
    if (!resolved)
        return std::nullopt;
    return resolved.mapping->physical_to_role[physical_extruder];
}

bool mixed_nozzle_remap_process_extruder_ids(
    DynamicPrintConfig &process, const MixedNozzleProcessRoleMapping &mapping)
{
    auto *ids = process.option<ConfigOptionInts>("print_extruder_id", false);
    if (ids == nullptr || ids->values.empty())
        return false;
    for (int &role_id : ids->values) {
        if (role_id < 1 || role_id > 2)
            return false;
        role_id = int(mapping.role_to_physical[size_t(role_id - 1)] + 1);
    }
    return true;
}

std::optional<ConfigOptionFloatOrPercent> body_split_effective_width(
    const ConfigOptionFloatOrPercent &configured_width,
    double resolved_nozzle, double reference_nozzle)
{
    if (!std::isfinite(configured_width.value) || configured_width.value < 0. ||
        !std::isfinite(resolved_nozzle) || resolved_nozzle <= 0. ||
        !std::isfinite(reference_nozzle) || reference_nozzle <= 0.)
        return std::nullopt;
    if (configured_width.percent || configured_width.value == 0.)
        return configured_width;
    return ConfigOptionFloatOrPercent(
        configured_width.value * resolved_nozzle / reference_nozzle, false);
}

std::vector<int> mixed_nozzle_prime_tower_filaments(std::vector<int> used,
    MixedNozzleSlicingMode mode, std::optional<std::pair<int, int>> pair,
    const std::vector<int> &filament_map)
{
    if (mode == MixedNozzleSlicingMode::FeatureSplit && pair && pair->first > 0 && pair->second > 0 &&
        size_t(pair->first) <= filament_map.size() && size_t(pair->second) <= filament_map.size()) {
        const int fine = filament_map[pair->first - 1], coarse = filament_map[pair->second - 1];
        if (fine >= 1 && fine <= 2 && coarse >= 1 && coarse <= 2 && fine != coarse) {
            used.push_back(pair->first);
            used.push_back(pair->second);
            std::sort(used.begin(), used.end());
            used.erase(std::unique(used.begin(), used.end()), used.end());
        }
    }
    return used;
}

std::vector<int> mixed_nozzle_manual_map_candidates(MixedNozzleSlicingMode effective_mode,
                                                    size_t configured_filament_count,
                                                    const std::vector<int> &plate_used_filaments,
                                                    bool forced_static_map)
{
    // Ordinary Orca slicing keeps the caller's list byte-for-byte. Only an active Mixed-Nozzle
    // setup, on the forced pre-slice mapping path, needs candidates the plate does not yet use.
    if (!forced_static_map || effective_mode == MixedNozzleSlicingMode::Off || configured_filament_count == 0)
        return plate_used_filaments;

    std::vector<int> candidates(configured_filament_count);
    std::iota(candidates.begin(), candidates.end(), 1);
    return candidates;
}

std::vector<BodySplitRegionAssignment> collect_body_split_volume_assignments(
    const ModelObject &model_object, double base_cadence)
{
    std::vector<BodySplitRegionAssignment> assignments;
    for (const ModelVolume *volume : model_object.volumes) {
        if (volume == nullptr || volume->type() != ModelVolumeType::MODEL_PART)
            continue;
        const bool has_height = volume->config.has("regional_layer_height");
        const double configured_height = has_height ? volume->config.opt_float("regional_layer_height") : 0.;
        const double cadence = (has_height && configured_height != 0.) ? configured_height : base_cadence;
        for (int filament_1based : volume->get_extruders())
            if (filament_1based > 0)
                assignments.push_back({cadence, size_t(filament_1based - 1)});
    }
    return assignments;
}

std::vector<BodySplitRegionAssignment> collect_body_split_body_assignments(
    const ModelObject &model_object, double base_cadence)
{
    std::vector<BodySplitRegionAssignment> assignments;
    for (const ModelVolume *volume : model_object.volumes) {
        if (volume == nullptr || !volume->is_model_part() || volume->extruder_id() <= 0)
            continue;
        const double height = volume->config.has("regional_layer_height") ?
            volume->config.opt_float("regional_layer_height") : 0.;
        assignments.push_back({height == 0. ? base_cadence : height,
                               size_t(volume->extruder_id() - 1)});
    }
    return assignments;
}

bool is_body_split_object(const PrintConfig &config, const ModelObject &model_object)
{
    size_t model_parts = 0;
    const ModelVolume *only_part = nullptr;
    // The physical tools the parts and their painted colours print on. On a toolchanger these
    // can be any two of its tools; an object on one tool has nothing to split.
    std::set<size_t> tools_used;
    bool unresolved = false;
    for (const ModelVolume *volume : model_object.volumes) {
        if (volume == nullptr || !volume->is_model_part())
            continue;
        ++model_parts;
        only_part = volume;
        std::vector<int> filaments = volume->get_extruders();
        filaments.push_back(volume->extruder_id());
        for (int filament : filaments) {
            if (filament <= 0)
                continue;
            const std::optional<size_t> tool = physical_extruder_for_filament(config, unsigned(filament - 1));
            if (!tool)
                unresolved = true;
            else
                tools_used.insert(*tool);
        }
    }
    if (model_parts == 0 || (model_parts == 1 && !only_part->is_mm_painted()))
        return false;
    return unresolved || tools_used.size() >= 2;
}

std::optional<std::pair<size_t, size_t>> mixed_nozzle_default_tool_pair(const std::vector<double> &nozzle_diameters)
{
    std::optional<size_t> fine, coarse;
    for (size_t tool = 0; tool < nozzle_diameters.size(); ++tool) {
        const double diameter = nozzle_diameters[tool];
        if (!std::isfinite(diameter) || diameter <= 0.)
            continue;
        if (!fine || diameter < nozzle_diameters[*fine])
            fine = tool;
        if (!coarse || diameter > nozzle_diameters[*coarse])
            coarse = tool;
    }
    if (!fine || !coarse || is_approx(nozzle_diameters[*fine], nozzle_diameters[*coarse]))
        return std::nullopt;
    return std::make_pair(*fine, *coarse);
}

namespace {
bool mixed_nozzle_line_duplicates_heads(const std::string &code)
{
    static const std::regex markers(R"((^|\s)(IDEX_COPY|IDEX_MIRROR)\b|\b(COPY|MIRROR)=1\b|\bM605\s+S[23]\b)");
    return std::regex_search(code, markers);
}
} // namespace

bool mixed_nozzle_gcode_duplicates_heads(const std::string &rendered_gcode)
{
    std::istringstream in(rendered_gcode);
    std::string line;
    while (std::getline(in, line))
        if (mixed_nozzle_line_duplicates_heads(line.substr(0, line.find(';'))))
            return true;
    return false;
}

bool mixed_nozzle_start_gcode_duplicates_heads(const std::string &start_gcode)
{
    static const std::regex opens(R"(\{if\b)");
    static const std::regex closes(R"(\{endif\})");
    std::istringstream in(start_gcode);
    std::string line;
    long depth = 0;
    while (std::getline(in, line)) {
        const std::string code = line.substr(0, line.find(';'));
        const long depth_before = depth;
        depth += long(std::distance(std::sregex_iterator(code.begin(), code.end(), opens), std::sregex_iterator())) -
                 long(std::distance(std::sregex_iterator(code.begin(), code.end(), closes), std::sregex_iterator()));
        if (depth_before == 0 && code.find("{if") == std::string::npos && mixed_nozzle_line_duplicates_heads(code))
            return true;
    }
    return false;
}

std::optional<double> resolve_body_split_reference_nozzle(
    const PrintConfig &config, double base_cadence,
    const std::vector<BodySplitRegionAssignment> &regions)
{
    if (!std::isfinite(base_cadence) || base_cadence <= 0. || regions.size() < 2)
        return std::nullopt;

    std::optional<double> reference_nozzle;
    std::optional<double> coarse_nozzle;
    for (const BodySplitRegionAssignment &region : regions) {
        if (!std::isfinite(region.cadence) || region.cadence <= 0.)
            return std::nullopt;
        const double raw_ratio = region.cadence / base_cadence;
        const int ratio = int(std::lround(raw_ratio));
        if (ratio < 1 || !is_approx(raw_ratio, double(ratio)))
            return std::nullopt;

        const MixedNozzleToolResolution resolved = resolve_mixed_nozzle_tool(
            config, region.logical_filament, MixedNozzleResolveScope::PhysicalToolOnly);
        if (!resolved)
            return std::nullopt;

        if (ratio == 1) {
            if (reference_nozzle && !is_approx(*reference_nozzle, resolved.tool->nozzle_diameter))
                return std::nullopt;
            reference_nozzle = resolved.tool->nozzle_diameter;
        } else {
            // Qualification metadata describes evidence and preset promotion; it must not hide a
            // cadence that is integral and legal for the resolved physical nozzle envelope.
            if (coarse_nozzle && !is_approx(*coarse_nozzle, resolved.tool->nozzle_diameter))
                return std::nullopt;
            coarse_nozzle = resolved.tool->nozzle_diameter;
        }
    }

    if (!reference_nozzle || !coarse_nozzle || *reference_nozzle >= *coarse_nozzle)
        return std::nullopt;
    return reference_nozzle;
}

std::optional<double> body_split_object_reference_nozzle(
    const PrintConfig &config, const ModelObject &model_object, double base_cadence)
{
    std::vector<BodySplitRegionAssignment> assignments =
        collect_body_split_body_assignments(model_object, base_cadence);
    // A fine painted region on a lone coarse body has no separate fine body binding.
    // Qualify its base cadence through the native resolver, as the ownership projector
    // does, rather than attributing the coarse parent's height to the painted colour.
    const bool has_base = std::any_of(assignments.begin(), assignments.end(),
        [base_cadence](const BodySplitRegionAssignment &entry) {
            return is_approx(entry.cadence, base_cadence);
        });
    if (!has_base && !assignments.empty()) {
        const BodySplitRegionAssignment coarse = assignments.front();
        const auto used = collect_body_split_volume_assignments(model_object, base_cadence);
        for (const BodySplitRegionAssignment &painted : used) {
            if (resolve_mixed_nozzle_cadence(config, base_cadence, coarse.cadence,
                                            painted.logical_filament, coarse.logical_filament))
                assignments.push_back({base_cadence, painted.logical_filament});
        }
    }
    return resolve_body_split_reference_nozzle(config, base_cadence, assignments);
}

bool mixed_nozzle_same_material_pair(const PrintConfig &config)
{
    std::array<std::optional<size_t>, 2> per_tool_filament;
    for (size_t logical_filament = 0; logical_filament < config.filament_map.size(); ++logical_filament) {
        const MixedNozzleToolResolution resolved = resolve_mixed_nozzle_tool(
            config, logical_filament, MixedNozzleResolveScope::PhysicalToolOnly);
        if (!resolved || resolved.tool->physical_extruder >= per_tool_filament.size())
            return false;
        std::optional<size_t> &slot = per_tool_filament[resolved.tool->physical_extruder];
        if (slot.has_value())
            return false; // more than one logical filament on this physical tool: ambiguous
        slot = logical_filament;
    }
    if (!per_tool_filament[0].has_value() || !per_tool_filament[1].has_value())
        return false;
    const size_t a = *per_tool_filament[0], b = *per_tool_filament[1];
    if (a >= config.filament_type.size() || b >= config.filament_type.size() ||
        a >= config.filament_vendor.size() || b >= config.filament_vendor.size() ||
        a >= config.filament_colour.size() || b >= config.filament_colour.size())
        return false;
    return config.filament_type.get_at(a) == config.filament_type.get_at(b) &&
           config.filament_vendor.get_at(a) == config.filament_vendor.get_at(b) &&
           config.filament_colour.get_at(a) == config.filament_colour.get_at(b);
}

namespace {

// Config-only equivalent of GUI_App.cpp is_support_filament(extruder_id, strict_check = false):
// PETG counts against PLA on the plate, PLA against PETG or TPU, otherwise filament_is_support decides.
bool stock_support_interface_material(const DynamicPrintConfig &config, size_t interface_filament)
{
    const ConfigOptionStrings *types = config.option<ConfigOptionStrings>("filament_type");
    if (types != nullptr && interface_filament < types->values.size()) {
        std::vector<std::string> counts_as_support;
        if (types->values[interface_filament] == "PETG")
            counts_as_support = {"PLA"};
        else if (types->values[interface_filament] == "PLA")
            counts_as_support = {"PETG", "TPU", "TPU-AMS"};
        for (size_t filament = 0; filament < types->values.size(); ++filament)
            if (filament != interface_filament &&
                std::find(counts_as_support.begin(), counts_as_support.end(), types->values[filament]) !=
                    counts_as_support.end())
                return true;
    }
    const ConfigOptionBools *is_support = config.option<ConfigOptionBools>("filament_is_support");
    return is_support != nullptr && interface_filament < is_support->values.size() &&
           is_support->values[interface_filament] != 0;
}

bool stock_soluble_filament(const DynamicPrintConfig &config, size_t filament)
{
    const ConfigOptionBools *soluble = config.option<ConfigOptionBools>("filament_soluble");
    return soluble != nullptr && filament < soluble->values.size() && soluble->values[filament] != 0;
}

bool stock_plate_carries_type(const DynamicPrintConfig &config, const std::vector<std::string> &wanted)
{
    const ConfigOptionStrings *types = config.option<ConfigOptionStrings>("filament_type");
    if (types == nullptr)
        return false;
    for (const std::string &type : types->values)
        if (std::find(wanted.begin(), wanted.end(), type) != wanted.end())
            return true;
    return false;
}

} // namespace

const std::vector<std::string> &mixed_nozzle_support_interface_recommendation_keys()
{
    static const std::vector<std::string> keys{
        "support_top_z_distance", "support_interface_spacing",
        "support_interface_pattern", "independent_support_layer_height"};
    return keys;
}

bool mixed_nozzle_write_support_interface_recommendation(DynamicPrintConfig &config)
{
    // The same four values Tab::on_value_change applies for support_interface_filament.
    bool changed = false;
    const auto write = [&](const char *key, ConfigOption *value) {
        const ConfigOption *current = config.option(key);
        if (current != nullptr && *current == *value) {
            delete value;
            return;
        }
        config.set_key_value(key, value);
        changed = true;
    };
    write("support_top_z_distance", new ConfigOptionFloat(0));
    write("support_interface_spacing", new ConfigOptionFloat(0));
    write("support_interface_pattern", new ConfigOptionEnum<SupportMaterialInterfacePattern>(
        SupportMaterialInterfacePattern::smipRectilinearInterlaced));
    write("independent_support_layer_height", new ConfigOptionBool(false));
    return changed;
}

bool mixed_nozzle_apply_support_interface_recommendation(DynamicPrintConfig &project)
{
    // In Off mode the stock Tab.cpp prompt owns this.
    if (!is_mixed_nozzle_slicing_enabled(project))
        return false;
    const ConfigOptionInt *interface_opt = project.option<ConfigOptionInt>("support_interface_filament");
    const ConfigOptionInt *base_opt      = project.option<ConfigOptionInt>("support_filament");
    if (interface_opt == nullptr || base_opt == nullptr || interface_opt->value <= 0)
        return false;
    const size_t interface_filament = size_t(interface_opt->value - 1);

    // Same conditions as Tab.cpp: a support-material interface not yet at the recommended values,
    // or a soluble interface over a non-soluble base.
    const ConfigOptionFloat *top_z   = project.option<ConfigOptionFloat>("support_top_z_distance");
    const ConfigOptionFloat *spacing = project.option<ConfigOptionFloat>("support_interface_spacing");
    const ConfigOptionEnum<SupportMaterialInterfacePattern> *pattern =
        project.option<ConfigOptionEnum<SupportMaterialInterfacePattern>>("support_interface_pattern");
    const bool already_recommended =
        top_z != nullptr && top_z->value == 0. && spacing != nullptr && spacing->value == 0. &&
        pattern != nullptr && pattern->value == SupportMaterialInterfacePattern::smipRectilinearInterlaced;
    const bool soluble_interface_non_soluble_base =
        stock_soluble_filament(project, interface_filament) &&
        !(base_opt->value > 0 && stock_soluble_filament(project, size_t(base_opt->value - 1)));
    if (!((stock_support_interface_material(project, interface_filament) && !already_recommended) ||
          soluble_interface_non_soluble_base))
        return false;

    bool changed = mixed_nozzle_write_support_interface_recommendation(project);
    // As in Tab.cpp, PLA against TPU or a soluble interface also moves the base onto the interface filament.
    const ConfigOptionStrings *types = project.option<ConfigOptionStrings>("filament_type");
    const bool pla_against_tpu = types != nullptr && interface_filament < types->values.size() &&
        types->values[interface_filament] == "PLA" &&
        stock_plate_carries_type(project, {"TPU", "TPU-AMS"});
    if ((pla_against_tpu || soluble_interface_non_soluble_base) &&
        base_opt->value != interface_opt->value) {
        project.set_key_value("support_filament", new ConfigOptionInt(interface_opt->value));
        changed = true;
    }
    return changed;
}

float mixed_nozzle_tower_brim_width(float tower_height_mm)
{
    if (tower_height_mm < 20.f) return 3.f;
    if (tower_height_mm < 60.f) return 5.f;
    return 8.f;
}

float mixed_nozzle_tower_footprint_floor(float tower_height_mm)
{
    if (tower_height_mm <= 60.f) return 0.f;
    if (tower_height_mm <= 100.f) return 30.f;
    return 40.f;
}

float mixed_nozzle_tower_rect_floor(float pad_extent_floor, float rib_width_mm, bool rib_wall)
{
    if (!std::isfinite(pad_extent_floor) || pad_extent_floor <= 0.f)
        return 0.f;
    if (!rib_wall || !std::isfinite(rib_width_mm) || rib_width_mm <= 0.f)
        return pad_extent_floor;
    // A diagonal rib reaches past the corner by its width over sqrt(2) on each axis, so the
    // rectangle itself only needs the remainder.
    return std::max(0.f, pad_extent_floor - rib_width_mm / std::sqrt(2.f));
}

float mixed_nozzle_tower_base_height(float tower_height_mm)
{
    if (tower_height_mm < 60.f) return 1.f;
    return 2.f;
}

float mixed_nozzle_tower_lag_max(float height_to_rod_mm)
{
    if (!std::isfinite(height_to_rod_mm))
        return 0.f;
    return std::max(0.f, height_to_rod_mm - mixed_nozzle_tower_lag_headroom);
}

bool mixed_nozzle_tower_lag_placement_clear(const BoundingBoxf &tower_footprint,
                                            const std::vector<BoundingBoxf> &object_footprints,
                                            float clearance_radius_mm)
{
    if (!std::isfinite(clearance_radius_mm) || clearance_radius_mm <= 0.f)
        return false;
    if (!tower_footprint.defined)
        return false;
    for (const BoundingBoxf &object : object_footprints) {
        if (!object.defined)
            continue;
        // Shortest gap between two axis-aligned footprints, zero where they overlap.
        const double dx = std::max({0., object.min.x() - tower_footprint.max.x(),
                                    tower_footprint.min.x() - object.max.x()});
        const double dy = std::max({0., object.min.y() - tower_footprint.max.y(),
                                    tower_footprint.min.y() - object.max.y()});
        if (std::hypot(dx, dy) < double(clearance_radius_mm))
            return false;
    }
    return true;
}

int mixed_nozzle_tower_coarse_filament(const PrintConfig &config)
{
    int    coarse    = -1;
    double widest    = 0.;
    double narrowest = std::numeric_limits<double>::max();
    for (size_t filament = 0; filament < config.filament_map.values.size(); ++filament) {
        const int nozzle = config.filament_map.values[filament] - 1;
        if (nozzle < 0 || size_t(nozzle) >= config.nozzle_diameter.values.size())
            continue;
        const double diameter = config.nozzle_diameter.values[size_t(nozzle)];
        if (!std::isfinite(diameter) || diameter <= 0.)
            continue;
        narrowest = std::min(narrowest, diameter);
        if (diameter > widest) {
            widest = diameter;
            coarse = int(filament);
        }
    }
    // One nozzle size is not a mixed nozzle print and has no coarse tool.
    return (coarse >= 0 && widest > narrowest) ? coarse : -1;
}

int mixed_nozzle_tower_coarse_extruder(const PrintConfig &config)
{
    // Derived from the coarse filament so the two helpers always agree.
    const int coarse_filament = mixed_nozzle_tower_coarse_filament(config);
    if (coarse_filament < 0)
        return -1;
    const std::optional<size_t> extruder =
        physical_extruder_for_filament(config, unsigned(coarse_filament));
    return extruder ? int(*extruder) : -1;
}

bool mixed_nozzle_compact_tower(const PrintConfig &config)
{
    return is_mixed_nozzle_feature_split(config) || config.wipe_tower_no_sparse_layers.value;
}

bool mixed_nozzle_compact_tower_applied_by_mode(const PrintConfig &config)
{
    return is_mixed_nozzle_feature_split(config) && !config.wipe_tower_no_sparse_layers.value;
}

double body_split_first_cell_height(double first_layer, double base, double nozzle_minimum)
{
    if (!std::isfinite(first_layer) || !std::isfinite(base) || base <= 0. || first_layer + EPSILON >= nozzle_minimum)
        return first_layer;
    const double rows = std::ceil((nozzle_minimum - first_layer - EPSILON) / base);
    return first_layer + std::max(0., rows) * base;
}

BodySplitToolEnvelope body_split_tool_envelope(const PrintConfig &config, size_t physical_nozzle,
                                               double base, double cadence, double first_layer)
{
    if (physical_nozzle >= config.nozzle_diameter.size() || !std::isfinite(base) || base <= 0. ||
        !std::isfinite(cadence) || cadence <= 0. || !std::isfinite(first_layer) || first_layer <= 0.)
        return BodySplitToolEnvelope::Unsupported;
    const double minimum  = resolved_min_layer_height(config, physical_nozzle);
    const double maximum  = std::min(resolved_max_layer_height(config, physical_nozzle),
                                     config.nozzle_diameter.get_at(physical_nozzle));
    if (!std::isfinite(minimum) || !std::isfinite(maximum) || minimum <= 0. || maximum < minimum)
        return BodySplitToolEnvelope::Unsupported;
    if (minimum > cadence + EPSILON || cadence > maximum + EPSILON)
        return BodySplitToolEnvelope::Unsupported;
    const bool base_body = std::abs(cadence - base) <= EPSILON;
    if (base_body)
        // A fine body lays the shared first layer and every base row itself.
        return minimum <= first_layer + EPSILON && first_layer <= maximum + EPSILON
            ? BodySplitToolEnvelope::Shared : BodySplitToolEnvelope::Unsupported;
    if (minimum <= base + EPSILON && minimum <= first_layer + EPSILON)
        return first_layer <= maximum + EPSILON ? BodySplitToolEnvelope::Shared : BodySplitToolEnvelope::Unsupported;
    return body_split_first_cell_height(first_layer, base, minimum) <= maximum + EPSILON
        ? BodySplitToolEnvelope::OwnFirstCell : BodySplitToolEnvelope::Unsupported;
}

bool body_split_tool_envelope_admitted(BodySplitToolEnvelope envelope, bool with_prime_tower,
                                       bool lagging_tower_available)
{
    switch (envelope) {
    case BodySplitToolEnvelope::Shared:       return true;
    case BodySplitToolEnvelope::OwnFirstCell: return !with_prime_tower || lagging_tower_available;
    case BodySplitToolEnvelope::Unsupported:  return false;
    }
    return false;
}

bool mixed_nozzle_tower_lagging_available(const PrintConfig &config)
{
    if (config.timelapse_type.value == TimelapseType::tlSmooth || config.enable_wrapping_detection.value)
        return false;
    return mixed_nozzle_tower_coarse_filament(config) >= 0;
}

double mixed_nozzle_tower_coarse_minimum(const PrintConfig &config)
{
    const int coarse = mixed_nozzle_tower_coarse_filament(config);
    if (coarse < 0 || size_t(coarse) >= config.filament_map.values.size())
        return 0.;
    const int nozzle = config.filament_map.values[size_t(coarse)] - 1;
    if (nozzle < 0 || size_t(nozzle) >= config.nozzle_diameter.values.size())
        return 0.;
    const double minimum = resolved_min_layer_height(config, size_t(nozzle));
    return std::isfinite(minimum) && minimum > 0. ? minimum : 0.;
}

bool mixed_nozzle_body_tower_lags(const PrintConfig &config, double base_layer_height)
{
    if (!is_mixed_nozzle_body_split(config) || !config.enable_prime_tower.value)
        return false;
    if (!std::isfinite(base_layer_height) || base_layer_height <= 0.)
        return false;
    if (!mixed_nozzle_tower_lagging_available(config))
        return false;
    const double first_layer = config.initial_layer_print_height.value > 0. ?
        config.initial_layer_print_height.value : base_layer_height;
    const double coarse_minimum = mixed_nozzle_tower_coarse_minimum(config);
    return coarse_minimum > std::min(base_layer_height, first_layer) + EPSILON;
}

bool mixed_nozzle_tower_lagging(const PrintConfig &config)
{
    if (!is_mixed_nozzle_feature_split(config))
        return false;
    // A lagging tower needs the compact tower to skip levels; Feature Split always supplies it.
    // Smooth timelapse and wrapping detection lay a leaving-tool wall first on every level, which a
    // level at the arriving tool's height cannot have, so those prints keep the following tower.
    if (config.timelapse_type.value == TimelapseType::tlSmooth || config.enable_wrapping_detection.value)
        return false;
    if (!(config.mixed_nozzle_coarse_layer_height.value > 0.))
        return false;
    return mixed_nozzle_tower_coarse_filament(config) >= 0;
}

} // namespace Slic3r
