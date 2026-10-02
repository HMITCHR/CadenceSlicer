#pragma once

// Write paths and the Rebind transaction for the per-filament intent ledger declared in
// MixedNozzleConfig.hpp. Kept separate because MixedNozzleConfig.hpp reaches every translation
// unit through Print.hpp, and only a few consumers mutate the binding state.

#include "MixedNozzleConfig.hpp"

#include <algorithm>
#include <string>
#include <vector>

namespace Slic3r {

inline constexpr const char *MIXED_NOZZLE_EXPLICIT_KEYS_OPTION = "mixed_nozzle_filament_explicit_keys";
inline constexpr const char *MIXED_NOZZLE_PROVENANCE_OPTION    = "mixed_nozzle_filament_provenance";
inline constexpr const char *MIXED_NOZZLE_BINDING_OPTION       = "mixed_nozzle_filament_binding";

// The stored ledger, read as sets and sized to the logical filament count. Absent metadata reads
// as an empty ledger on a `bound` project, which is what a fresh in-session project is.
inline std::vector<std::set<std::string>> mixed_nozzle_ledger_sets(const DynamicPrintConfig &project,
                                                                   size_t num_filaments)
{
    std::vector<std::set<std::string>> sets(num_filaments);
    if (const auto *opt = project.option<ConfigOptionStrings>(MIXED_NOZZLE_EXPLICIT_KEYS_OPTION))
        for (size_t i = 0; i < num_filaments && i < opt->values.size(); ++i)
            sets[i] = mixed_nozzle_split_ledger_keys(opt->values[i]);
    return sets;
}

inline void mixed_nozzle_write_ledger_sets(DynamicPrintConfig &project,
                                           const std::vector<std::set<std::string>> &sets,
                                           const std::vector<MixedNozzleProvenance> &provenance)
{
    std::vector<std::string> keys;
    keys.reserve(sets.size());
    for (const auto &set : sets)
        keys.emplace_back(mixed_nozzle_join_ledger_keys(set));
    std::vector<std::string> tokens;
    tokens.reserve(provenance.size());
    for (MixedNozzleProvenance value : provenance)
        tokens.emplace_back(mixed_nozzle_provenance_token(value));
    project.option<ConfigOptionStrings>(MIXED_NOZZLE_EXPLICIT_KEYS_OPTION, true)->values = std::move(keys);
    project.option<ConfigOptionStrings>(MIXED_NOZZLE_PROVENANCE_OPTION, true)->values = std::move(tokens);
}

// Write path (1): a live edit of a variant key in the Filament tab. The key becomes the explicit
// intent of every logical filament using that preset, so the choice survives even when its value
// happens to equal the parent's - which is exactly what a value diff cannot record.
// Returns true when the ledger changed.
inline bool mixed_nozzle_ledger_add_key(DynamicPrintConfig &project, size_t num_filaments,
                                        const std::vector<size_t> &logical_filaments, const std::string &key)
{
    if (num_filaments == 0 || logical_filaments.empty() || mixed_nozzle_ledger_key_set().count(key) == 0)
        return false;
    const MixedNozzleLedger ledger = mixed_nozzle_read_ledger(project, num_filaments);
    // Malformed metadata is never edited into a qualified state by a side effect.
    if (!ledger.qualified)
        return false;
    auto sets = mixed_nozzle_ledger_sets(project, num_filaments);
    bool changed = false;
    for (size_t i : logical_filaments)
        if (i < num_filaments)
            changed |= sets[i].insert(key).second;
    if (!changed)
        return false;
    mixed_nozzle_write_ledger_sets(project, sets, ledger.provenance);
    return true;
}

// Write path (4): import. A project without provenance is legacy for every filament and keeps
// every saved value until an explicit Rebind. Written unconditionally, since apply_only() would
// otherwise keep a previous project's vectors. Malformed metadata is left as found so its
// diagnostic survives.
inline void mixed_nozzle_set_legacy_provenance(DynamicPrintConfig &project, size_t num_filaments)
{
    project.option<ConfigOptionStrings>(MIXED_NOZZLE_PROVENANCE_OPTION, true)->values.assign(
        num_filaments, mixed_nozzle_provenance_token(MixedNozzleProvenance::Legacy));
    // A legacy import has no per-key intent to record; the ledger itself stays empty.
    project.option<ConfigOptionStrings>(MIXED_NOZZLE_EXPLICIT_KEYS_OPTION, true)->values.assign(num_filaments,
                                                                                                std::string());
}

// Which saved filament values survive import. Legacy, unqualified or metadata-less filaments keep
// all of them. A bound filament keeps every non-variant key plus the variant keys its ledger
// names; the rest are recomposed for its tool.
inline bool mixed_nozzle_import_preserves_key(const MixedNozzleLedger &ledger, size_t logical_filament,
                                              const std::string &key)
{
    if (ledger.absent || !ledger.qualified || ledger.is_legacy(logical_filament))
        return true;
    if (filament_options_with_variant.count(key) == 0 && key != "filament_prime_volume")
        return true;
    return ledger.is_explicit(logical_filament, key);
}

// The explicit Rebind transaction.

struct MixedNozzleRebindEntry {
    size_t      logical_filament {0};
    std::string key;
    // True when the value equals the ancestor's value for the selected column, so it was probably
    // derived from the wrong sibling; candidates start selected. Other offered keys are unselected
    // Resets of values the user appears to own.
    bool        candidate {false};
    std::string current_value;
    std::string sibling_value;
    std::string current_columns;
    std::string sibling_columns;
};

// The reset destination is part of the row's meaning. Ordinary filament prime volume belongs
// to the material ancestor; variant-bound settings resolve through the nozzle sibling.
enum class MixedNozzleRebindDestination {
    MaterialAncestor,
    NozzleVariant,
};

// Destination is a property of the setting key, never of display text.
inline MixedNozzleRebindDestination mixed_nozzle_rebind_destination(const std::string &key)
{
    return key == "filament_prime_volume" ? MixedNozzleRebindDestination::MaterialAncestor
                                          : MixedNozzleRebindDestination::NozzleVariant;
}

// The state the preview was built from. Apply is refused when it no longer matches, the same way
// the sidebar binds an Apply to the signature its controls were seeded from.
struct MixedNozzleRebindSignature {
    std::vector<std::string> filament_presets;
    std::vector<double>      nozzle_diameters;
    std::vector<int>         filament_map;
    std::vector<std::string> provenance;
    std::vector<std::string> effective_values;
    bool operator==(const MixedNozzleRebindSignature &rhs) const
    {
        return filament_presets == rhs.filament_presets && nozzle_diameters == rhs.nozzle_diameters &&
               filament_map == rhs.filament_map && provenance == rhs.provenance && effective_values == rhs.effective_values;
    }
    bool operator!=(const MixedNozzleRebindSignature &rhs) const { return !(*this == rhs); }
};

struct MixedNozzleRebindPlan {
    // Offered on legacy and unqualified projects only; a bound project has nothing to convert.
    bool                                offered {false};
    size_t                              num_filaments {0};
    MixedNozzleRebindSignature          signature;
    std::vector<MixedNozzleRebindEntry> entries;
    std::vector<std::set<std::string>> preserved_keys;
    std::vector<MixedNozzleProvenance> result_provenance;

    // Candidates start selected; a Reset of a value the user appears to own does not.
    std::vector<bool> default_selection() const
    {
        std::vector<bool> selection(entries.size(), false);
        for (size_t i = 0; i < entries.size(); ++i)
            selection[i] = entries[i].candidate;
        return selection;
    }
};

// What one accepted Rebind writes. Every offered key the user did *not* accept stays the
// filament's own explicit intent; every accepted entry is dropped from the ledger so the next
// composition takes the resolved sibling's value.
inline bool mixed_nozzle_rebind_write(DynamicPrintConfig &project, const MixedNozzleRebindPlan &plan,
                                      const std::vector<bool> &accepted)
{
    if (!plan.offered || plan.num_filaments == 0 || accepted.size() != plan.entries.size())
        return false;
    auto sets = plan.preserved_keys;
    if (sets.size() != plan.num_filaments)
        sets = mixed_nozzle_ledger_sets(project, plan.num_filaments);
    for (size_t i = 0; i < plan.entries.size(); ++i) {
        const auto &entry = plan.entries[i];
        if (entry.logical_filament >= plan.num_filaments)
            return false;
        if (accepted[i])
            sets[entry.logical_filament].erase(entry.key);
        else
            sets[entry.logical_filament].insert(entry.key);
    }
    auto provenance = plan.result_provenance;
    if (provenance.size() != plan.num_filaments)
        provenance.assign(plan.num_filaments, MixedNozzleProvenance::Bound);
    mixed_nozzle_write_ledger_sets(project, sets, provenance);
    return true;
}

} // namespace Slic3r
