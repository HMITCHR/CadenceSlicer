#pragma once

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <iomanip>
#include <memory>
#include <optional>
#include <set>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

#include <nlohmann/json.hpp>

#include "libslic3r/MixedNozzleBinding.hpp"
#include "libslic3r/PrintConfig.hpp"

namespace Slic3r::GUI {

inline constexpr const char *MIXED_NOZZLE_TOWER_POLICY_OPTION = "mixed_nozzle_tower_policy_state";
inline constexpr const char *MIXED_NOZZLE_TOWER_POLICY_FRESH_OPTION = "mixed_nozzle_tower_policy_fresh";
// v2 manages the whole prime-tower profile with typed serialized values; v1 payloads stay readable
// with their three boolean keys and every newer key unmanaged. v3 adds the brim. Any other version
// fails closed (preserve-only), so a build that predates a key refuses a payload carrying it rather
// than dropping the ownership row.
inline constexpr int MIXED_NOZZLE_TOWER_POLICY_VERSION = 3;
inline constexpr int MIXED_NOZZLE_TOWER_POLICY_MIN_VERSION = 1;

// Shared process settings, owned in one project ledger because every plate consumes the same value;
// a plate never captures a competing "before" value. MIXED_NOZZLE_TOWER_CHANGE_PRIME_KEY is the one
// printer-preset member, so the ledger also records a printer scope.
inline constexpr const char *MIXED_NOZZLE_TOWER_ENABLE_KEY = "enable_prime_tower";
inline constexpr const char *MIXED_NOZZLE_TOWER_NO_SPARSE_KEY = "wipe_tower_no_sparse_layers";
inline constexpr const char *MIXED_NOZZLE_TOWER_AUTO_PAD_KEY = "mixed_nozzle_auto_pad";
inline constexpr const char *MIXED_NOZZLE_TOWER_WIDTH_KEY = "prime_tower_width";
inline constexpr const char *MIXED_NOZZLE_TOWER_WALL_TYPE_KEY = "wipe_tower_wall_type";
// The user-facing name is "prime_tower_rib_width"; PrintConfigDef::handle_legacy renames it to this
// canonical key, which is the one a ledger must address.
inline constexpr const char *MIXED_NOZZLE_TOWER_RIB_WIDTH_KEY = "wipe_tower_rib_width";
inline constexpr const char *MIXED_NOZZLE_TOWER_INFILL_GAP_KEY = "prime_tower_infill_gap";
inline constexpr const char *MIXED_NOZZLE_TOWER_FLAT_IRONING_KEY = "prime_tower_flat_ironing";
inline constexpr const char *MIXED_NOZZLE_TOWER_BRIM_KEY = "prime_tower_brim_width";
inline constexpr const char *MIXED_NOZZLE_TOWER_PRIME_MODE_KEY = "prime_volume_mode";
inline constexpr const char *MIXED_NOZZLE_TOWER_CHANGE_PRIME_KEY = "mixed_nozzle_extruder_change_prime_volume";

// AutoPad sizes the pad up to the configured tower width, so the policy caps the width instead of
// setting it.
inline constexpr double MIXED_NOZZLE_TOWER_WIDTH_CAP = 35.;

enum class MixedNozzleTowerKeyTarget { Process, Printer };

// Value: write `value`. Cap: write min(current, value). Preserve: keep the current value but record
// it as owned. NilPerElement: one nil per existing element (nil means "use the handoff resolver").
enum class MixedNozzleTowerRecommendation { Value, Cap, Preserve, NilPerElement };

struct MixedNozzleTowerManagedKey {
    const char *key;
    MixedNozzleTowerKeyTarget target;
    MixedNozzleTowerRecommendation recommendation;
    const char *value;          // Value/Cap only
    int since_version;
    const char *label;          // short user-facing name for review lines
    const char *pending_note;   // nullptr, or why the value is preserved instead of set
};

// The recommended profile. Wall type, rib width and infill gap are Preserve: the rib-versus-plain
// decision is still open, so Automatic owns them without changing them and the review says so.
inline const std::vector<MixedNozzleTowerManagedKey> &mixed_nozzle_tower_managed_key_table()
{
    static const std::vector<MixedNozzleTowerManagedKey> table {
        {MIXED_NOZZLE_TOWER_ENABLE_KEY, MixedNozzleTowerKeyTarget::Process,
         MixedNozzleTowerRecommendation::Value, "1", 1, "Prime tower", nullptr},
        {MIXED_NOZZLE_TOWER_NO_SPARSE_KEY, MixedNozzleTowerKeyTarget::Process,
         MixedNozzleTowerRecommendation::Value, "1", 1, "Supports only where needed", nullptr},
        {MIXED_NOZZLE_TOWER_AUTO_PAD_KEY, MixedNozzleTowerKeyTarget::Process,
         MixedNozzleTowerRecommendation::Value, "1", 1, "Automatic footprint", nullptr},
        {MIXED_NOZZLE_TOWER_WIDTH_KEY, MixedNozzleTowerKeyTarget::Process,
         MixedNozzleTowerRecommendation::Cap, "35", 2, "Tower width cap", nullptr},
        {MIXED_NOZZLE_TOWER_WALL_TYPE_KEY, MixedNozzleTowerKeyTarget::Process,
         MixedNozzleTowerRecommendation::Value, "rib", 2, "Wall type", "Bambu default"},
        {MIXED_NOZZLE_TOWER_RIB_WIDTH_KEY, MixedNozzleTowerKeyTarget::Process,
         MixedNozzleTowerRecommendation::Value, "8", 2, "Rib width", "Bambu default"},
        {MIXED_NOZZLE_TOWER_INFILL_GAP_KEY, MixedNozzleTowerKeyTarget::Process,
         MixedNozzleTowerRecommendation::Value, "150%", 2, "Infill gap", "Bambu default"},
        {MIXED_NOZZLE_TOWER_FLAT_IRONING_KEY, MixedNozzleTowerKeyTarget::Process,
         MixedNozzleTowerRecommendation::Value, "0", 2, "Flat ironing", nullptr},
        // -1 is the option's own "Auto". The tower height is only known once sliced, so Automatic
        // leaves the brim to the engine (mixed_nozzle_tower_brim_width).
        {MIXED_NOZZLE_TOWER_BRIM_KEY, MixedNozzleTowerKeyTarget::Process,
         MixedNozzleTowerRecommendation::Value, "-1", 3, "Brim", nullptr},
        {MIXED_NOZZLE_TOWER_PRIME_MODE_KEY, MixedNozzleTowerKeyTarget::Process,
         MixedNozzleTowerRecommendation::Value, "Default", 2, "Prime volume mode", nullptr},
        {MIXED_NOZZLE_TOWER_CHANGE_PRIME_KEY, MixedNozzleTowerKeyTarget::Printer,
         MixedNozzleTowerRecommendation::NilPerElement, nullptr, 2, "Nozzle-switch conditioning", nullptr},
    };
    return table;
}

inline const MixedNozzleTowerManagedKey *mixed_nozzle_tower_managed_key(const std::string &key)
{
    const auto &table = mixed_nozzle_tower_managed_key_table();
    const auto it = std::find_if(table.begin(), table.end(),
                                 [&key](const auto &entry) { return key == entry.key; });
    return it == table.end() ? nullptr : &*it;
}

inline bool mixed_nozzle_tower_key_known(const std::string &key, int version)
{
    const auto *entry = mixed_nozzle_tower_managed_key(key);
    return entry != nullptr && entry->since_version <= version;
}

// A stored value is legal only in the option's own canonical serialization; a round trip through
// the option's default rejects junk, wrong types and non-canonical spellings.
inline bool mixed_nozzle_tower_value_canonical(const std::string &key, const std::string &value)
{
    const ConfigOptionDef *def = print_config_def.get(key);
    if (def == nullptr || def->default_value.get() == nullptr)
        return false;
    std::unique_ptr<ConfigOption> option(def->default_value->clone());
    try {
        if (!option->deserialize(value))
            return false;
    } catch (...) {
        return false;
    }
    return option->serialize() == value;
}

inline std::optional<std::string> mixed_nozzle_tower_read_value(const DynamicPrintConfig &config,
                                                                 const std::string &key)
{
    const ConfigOption *option = config.option(key);
    return option == nullptr ? std::nullopt : std::optional<std::string>(option->serialize());
}

inline bool mixed_nozzle_tower_write_value(DynamicPrintConfig &config, const std::string &key,
                                            const std::string &value)
{
    ConfigSubstitutionContext substitutions{ForwardCompatibilitySubstitutionRule::Disable};
    try {
        return config.set_deserialize_nothrow(key, value, substitutions);
    } catch (...) {
        return false;
    }
}

// The value Automatic writes for one key. nullopt when the key is absent and only preserved.
inline std::optional<std::string> mixed_nozzle_tower_recommended_value(
    const MixedNozzleTowerManagedKey &entry, const DynamicPrintConfig &source)
{
    const auto current = mixed_nozzle_tower_read_value(source, entry.key);
    switch (entry.recommendation) {
    case MixedNozzleTowerRecommendation::Value:
        return std::string(entry.value);
    case MixedNozzleTowerRecommendation::Cap: {
        if (!current) return std::string(entry.value);
        const auto *option = source.option(entry.key);
        const auto *cap = dynamic_cast<const ConfigOptionFloat *>(option);
        if (cap == nullptr) return std::string(entry.value);
        return cap->value <= MIXED_NOZZLE_TOWER_WIDTH_CAP ? *current : std::string(entry.value);
    }
    case MixedNozzleTowerRecommendation::Preserve:
        return current;
    case MixedNozzleTowerRecommendation::NilPerElement: {
        const auto *option = source.option(entry.key);
        const auto *vector = dynamic_cast<const ConfigOptionVectorBase *>(option);
        const std::size_t count = vector == nullptr ? 1 : std::max<std::size_t>(1, vector->size());
        std::string value = "nil";
        for (std::size_t i = 1; i < count; ++i) value += ";nil";
        return value;
    }
    }
    return std::nullopt;
}

enum class MixedNozzleTowerChoice { Unset, Automatic, Enabled, Disabled };
enum class MixedNozzleTowerProvenance { FreshSetup, ImportedProject, DeliberateAutomatic };

struct MixedNozzleTowerPolicyRequest {
    MixedNozzleSlicingMode mode {MixedNozzleSlicingMode::Off};
    MixedNozzleTowerChoice choice {MixedNozzleTowerChoice::Unset};
    MixedNozzleTowerProvenance provenance {MixedNozzleTowerProvenance::FreshSetup};
    // Final physical tools from the assignment/map resolver. Need is never inferred from mode
    // alone; an empty or one-tool proposal stays tower-off.
    std::vector<int> resolved_physical_tools;
    bool stale {false};
    std::vector<std::uint64_t> consuming_plate_ids;
    // Staging uses the selected preset name as the ownership scope.
    std::string process_scope;
    bool mapping_unresolved {false};
};

struct MixedNozzleTowerPolicyEvaluation {
    bool blocking_choice {false};
    bool manage_values {false};
    bool tower_required {false};
    std::vector<std::string> admission_diagnostics;
};

// Project-only ownership record, plain and versioned so an unknown payload fails closed. Values are
// serialized DynamicPrintConfig strings; this type does not apply them.
struct MixedNozzleTowerPolicyLedger {
    int version {MIXED_NOZZLE_TOWER_POLICY_VERSION};
    bool qualified {true};
    MixedNozzleTowerProvenance provenance {MixedNozzleTowerProvenance::FreshSetup};
    std::string process_scope;
    // Non-empty whenever a printer-preset member of the profile is managed.
    std::string printer_scope;
    std::set<std::string> managed_keys;
    std::set<std::string> manual_keys;
    std::vector<std::pair<std::string, std::string>> before_values;
    std::vector<std::pair<std::string, std::string>> applied_values;
    std::vector<std::uint64_t> consuming_plate_ids;

    std::optional<std::string> before(const std::string &key) const
    {
        const auto it = std::find_if(before_values.begin(), before_values.end(),
                                     [&key](const auto &entry) { return entry.first == key; });
        return it == before_values.end() ? std::nullopt : std::optional<std::string>(it->second);
    }

    std::optional<std::string> applied(const std::string &key) const
    {
        const auto it = std::find_if(applied_values.begin(), applied_values.end(),
                                     [&key](const auto &entry) { return entry.first == key; });
        return it == applied_values.end() ? std::nullopt : std::optional<std::string>(it->second);
    }

    const std::string &scope_for(const MixedNozzleTowerManagedKey &entry) const
    {
        return entry.target == MixedNozzleTowerKeyTarget::Printer ? printer_scope : process_scope;
    }

    std::string serialize() const
    {
        nlohmann::json out;
        out["v"] = version;
        out["qualified"] = qualified;
        out["provenance"] = static_cast<int>(provenance);
        out["scope"] = process_scope;
        out["managed"] = managed_keys;
        out["manual"] = manual_keys;
        out["before"] = nlohmann::json::array();
        for (const auto &entry : before_values)
            out["before"].push_back({{"key", entry.first}, {"value", entry.second}});
        out["applied"] = nlohmann::json::array();
        for (const auto &entry : applied_values)
            out["applied"].push_back({{"key", entry.first}, {"value", entry.second}});
        out["consumers"] = consuming_plate_ids;
        if (version >= 2)
            out["printer_scope"] = printer_scope;
        return out.dump();
    }

    // Strict parser: rejects unknown fields, malformed typed values and duplicate rows. Callers
    // treat rejection as preserve-only and never infer an automatic owner from a missing or corrupt
    // payload.
    static std::optional<MixedNozzleTowerPolicyLedger> parse(const std::string &payload)
    {
        try {
            bool duplicate_field = false;
            std::vector<std::set<std::string>> object_keys;
            const nlohmann::json input = nlohmann::json::parse(payload,
                [&](int, nlohmann::json::parse_event_t event, nlohmann::json& value) {
                    if (event == nlohmann::json::parse_event_t::object_start) object_keys.emplace_back();
                    else if (event == nlohmann::json::parse_event_t::key &&
                             !object_keys.back().insert(value.get<std::string>()).second) duplicate_field = true;
                    else if (event == nlohmann::json::parse_event_t::object_end) object_keys.pop_back();
                    return true;
                });
            if (duplicate_field || !input.is_object()) return std::nullopt;
            static const std::set<std::string> required {
                "v", "qualified", "provenance", "scope", "managed", "before", "applied", "consumers"};
            static const std::set<std::string> optional {"manual", "printer_scope"};
            for (const auto &field : required)
                if (!input.contains(field)) return std::nullopt;
            for (const auto &member : input.items())
                if (required.count(member.key()) == 0 && optional.count(member.key()) == 0) return std::nullopt;
            if (!input.at("v").is_number_integer() || !input.at("provenance").is_number_integer() ||
                !input.at("qualified").is_boolean() || !input.at("consumers").is_array()) return std::nullopt;
            // Read as 64-bit so an out-of-range ordinal cannot truncate into an accepted version.
            const std::int64_t raw_version = input.at("v").get<std::int64_t>();
            if (raw_version < MIXED_NOZZLE_TOWER_POLICY_MIN_VERSION ||
                raw_version > MIXED_NOZZLE_TOWER_POLICY_VERSION ||
                input.at("provenance") < 0 || input.at("provenance") > 2) return std::nullopt;
            const int version = int(raw_version);
            if (version < 2 && input.contains("printer_scope")) return std::nullopt;
            MixedNozzleTowerPolicyLedger result;
            result.version = version;
            result.qualified = input.at("qualified").get<bool>();
            const int p = input.at("provenance").get<int>();
            if (p < 0 || p > 2) return std::nullopt;
            result.provenance = static_cast<MixedNozzleTowerProvenance>(p);
            result.process_scope = input.at("scope").get<std::string>();
            if (input.contains("printer_scope")) {
                if (!input.at("printer_scope").is_string()) return std::nullopt;
                result.printer_scope = input.at("printer_scope").get<std::string>();
            }
            for (const std::string &key : input.at("managed").get<std::vector<std::string>>())
                if (!result.managed_keys.insert(key).second ||
                    !mixed_nozzle_tower_key_known(key, version)) return std::nullopt;
            if (input.contains("manual"))
                for (const auto& key : input.at("manual").get<std::vector<std::string>>())
                    if (!result.manual_keys.insert(key).second || result.managed_keys.count(key) != 0 ||
                        !mixed_nozzle_tower_key_known(key, version)) return std::nullopt;
            const auto value_legal = [version](const std::string &key, const std::string &value) {
                if (value == "absent") return true;
                // v1 recorded only "0"/"1" booleans; keep that grammar exact so a v1 payload cannot
                // carry a typed value.
                if (version < 2) return value == "0" || value == "1";
                return mixed_nozzle_tower_value_canonical(key, value);
            };
            const auto parse_pairs = [version, &value_legal](const nlohmann::json &rows,
                                        std::vector<std::pair<std::string, std::string>> &out) {
                if (!rows.is_array()) return false;
                for (const auto &row : rows) {
                    if (!row.is_object() || row.size() != 2 || !row.contains("key") || !row.contains("value")) return false;
                    if (!row.at("key").is_string() || !row.at("value").is_string()) return false;
                    const std::string key = row.at("key").get<std::string>();
                    const std::string value = row.at("value").get<std::string>();
                    if (!mixed_nozzle_tower_key_known(key, version) || !value_legal(key, value) ||
                        std::any_of(out.begin(), out.end(), [&key](const auto &entry) { return entry.first == key; })) return false;
                    out.emplace_back(key, value);
                }
                return true;
            };
            if (!parse_pairs(input.at("before"), result.before_values) ||
                !parse_pairs(input.at("applied"), result.applied_values)) return std::nullopt;
            for (const auto& id : input.at("consumers"))
                if (!id.is_number_unsigned()) return std::nullopt;
            for (const auto& row : result.before_values)
                if (result.managed_keys.count(row.first) == 0) return std::nullopt;
            for (const auto& row : result.applied_values)
                if (result.managed_keys.count(row.first) == 0 || row.second == "absent") return std::nullopt;
            // A managed printer-preset key is only interpretable with the printer scope that owns it.
            for (const auto& key : result.managed_keys) {
                const auto *entry = mixed_nozzle_tower_managed_key(key);
                if (entry != nullptr && entry->target == MixedNozzleTowerKeyTarget::Printer &&
                    result.printer_scope.empty()) return std::nullopt;
            }
            result.consuming_plate_ids = input.at("consumers").get<std::vector<std::uint64_t>>();
            std::sort(result.consuming_plate_ids.begin(), result.consuming_plate_ids.end());
            if (std::adjacent_find(result.consuming_plate_ids.begin(), result.consuming_plate_ids.end()) != result.consuming_plate_ids.end() ||
                result.process_scope.empty() ||
                result.managed_keys.size() != result.before_values.size() ||
                result.managed_keys.size() != result.applied_values.size()) return std::nullopt;
            return result;
        } catch (...) { return std::nullopt; }
    }
};

// ---------------------------------------------------------------------------------------------
// Typed apply and restore. Every write goes back through the option's own deserializer, so a value
// keeps exactly the form the ledger recorded.
// ---------------------------------------------------------------------------------------------

inline void mixed_nozzle_apply_tower_values(DynamicPrintConfig &process, DynamicPrintConfig *printer,
                                            const MixedNozzleTowerPolicyLedger &ledger)
{
    for (const auto &row : ledger.applied_values) {
        const auto *entry = mixed_nozzle_tower_managed_key(row.first);
        if (entry == nullptr) continue;
        DynamicPrintConfig *target = entry->target == MixedNozzleTowerKeyTarget::Printer ? printer : &process;
        if (target != nullptr)
            mixed_nozzle_tower_write_value(*target, row.first, row.second);
    }
}

// Restores every managed key still holding the value the ledger applied. Returns the keys the user
// changed since, which the caller records as manual.
inline std::set<std::string> mixed_nozzle_restore_tower_values(
    DynamicPrintConfig &process, DynamicPrintConfig *printer,
    const MixedNozzleTowerPolicyLedger &ledger)
{
    std::set<std::string> detached;
    for (const std::string &key : ledger.managed_keys) {
        const auto *entry = mixed_nozzle_tower_managed_key(key);
        DynamicPrintConfig *target = entry != nullptr && entry->target == MixedNozzleTowerKeyTarget::Printer
            ? printer : &process;
        const auto applied = ledger.applied(key);
        const auto before = ledger.before(key);
        const auto current = target == nullptr ? std::optional<std::string>{}
                                             : mixed_nozzle_tower_read_value(*target, key);
        if (target != nullptr && current && applied && before && *current == *applied) {
            if (*before == "absent") target->erase(key);
            else mixed_nozzle_tower_write_value(*target, key, *before);
        } else {
            detached.insert(key);
        }
    }
    return detached;
}

struct MixedNozzleTowerLedgerInputs {
    const DynamicPrintConfig *process {nullptr};
    const DynamicPrintConfig *printer {nullptr};
    std::string process_scope;
    std::string printer_scope;
    // Keys the user has deliberately edited in the selected preset (its dirty options).
    std::set<std::string> explicit_process_keys;
    std::set<std::string> explicit_printer_keys;
    const MixedNozzleTowerPolicyLedger *existing {nullptr};
    MixedNozzleTowerProvenance provenance {MixedNozzleTowerProvenance::FreshSetup};
};

// The single place that decides what Automatic owns and writes. The transaction and the dialogs'
// text both use it, so the preview is the plan that is applied.
inline MixedNozzleTowerPolicyLedger mixed_nozzle_tower_build_ledger(const MixedNozzleTowerLedgerInputs &inputs)
{
    MixedNozzleTowerPolicyLedger result;
    result.provenance = inputs.provenance;
    result.process_scope = inputs.process_scope;
    result.printer_scope = inputs.printer_scope;
    if (inputs.existing != nullptr)
        result.manual_keys = inputs.existing->manual_keys;
    for (const auto &entry : mixed_nozzle_tower_managed_key_table()) {
        const std::string key = entry.key;
        if (result.manual_keys.count(key) != 0) continue;
        const DynamicPrintConfig *source = entry.target == MixedNozzleTowerKeyTarget::Printer
            ? inputs.printer : inputs.process;
        if (source == nullptr) continue;
        // A printer key without a printer scope would not round-trip, so leave it unmanaged.
        if (entry.target == MixedNozzleTowerKeyTarget::Printer && inputs.printer_scope.empty()) continue;
        const auto applied = mixed_nozzle_tower_recommended_value(entry, *source);
        if (!applied) continue; // preserve-only key that this project does not carry
        const auto current = mixed_nozzle_tower_read_value(*source, key);
        const auto prior = inputs.existing != nullptr ? inputs.existing->applied(key)
                                                     : std::optional<std::string>{};
        const auto &explicit_keys = entry.target == MixedNozzleTowerKeyTarget::Printer
            ? inputs.explicit_printer_keys : inputs.explicit_process_keys;
        const bool still_owned = prior && current && *current == *prior;
        if (!still_owned && (prior || explicit_keys.count(key) != 0)) {
            result.manual_keys.insert(key);
            continue;
        }
        std::string before_value;
        if (inputs.existing != nullptr) {
            const std::string &scope = entry.target == MixedNozzleTowerKeyTarget::Printer
                ? inputs.printer_scope : inputs.process_scope;
            if (inputs.existing->scope_for(entry) == scope) {
                if (const auto captured = inputs.existing->before(key))
                    before_value = *captured;
            }
        }
        if (before_value.empty())
            before_value = current ? *current : "absent";
        result.managed_keys.insert(key);
        result.before_values.emplace_back(key, before_value);
        result.applied_values.emplace_back(key, *applied);
    }
    return result;
}

// ---------------------------------------------------------------------------------------------
// Presentation. The dialogs, the review page and the sidebar render these from the same ledger the
// transaction writes.
// ---------------------------------------------------------------------------------------------

inline constexpr const char *MIXED_NOZZLE_TOWER_PROVENANCE_AUTOMATIC = "Automatic";
inline constexpr const char *MIXED_NOZZLE_TOWER_PROVENANCE_MANUAL = "Manual";
inline constexpr const char *MIXED_NOZZLE_TOWER_PROVENANCE_PROJECT = "This project";

inline std::string mixed_nozzle_tower_display_value(const std::string &key, const std::string &value)
{
    if (value == "absent") return "not set";
    // The brim is resolved while slicing, so say what will happen instead of showing -1.
    if (key == MIXED_NOZZLE_TOWER_BRIM_KEY && value == "-1")
        return "automatic by tower height (3 / 5 / 8 mm)";
    const ConfigOptionDef *def = print_config_def.get(key);
    if (def != nullptr && def->type == coBool) return value == "1" ? "on" : "off";
    if (value == "nil" || value.rfind("nil;", 0) == 0) return "resolved purge";
    if (def != nullptr && def->type == coPercent && (value.empty() || value.back() != '%')) return value + "%";
    return value;
}

struct MixedNozzleTowerReviewLine {
    std::string key;
    std::string label;
    std::string value;        // display form
    std::string provenance;
    bool preserved {false};
    std::string note;

    std::string text() const
    {
        std::string body = preserved ? "unchanged (" + value + ")" : value;
        if (!note.empty()) body += ", " + note;
        return label + ": " + body + " (" + provenance + ")";
    }
};

// Every value the ledger owns, in profile order, with provenance. Manual keys are listed too.
inline std::vector<MixedNozzleTowerReviewLine> mixed_nozzle_tower_review_lines(
    const MixedNozzleTowerPolicyLedger &ledger)
{
    std::vector<MixedNozzleTowerReviewLine> lines;
    for (const auto &entry : mixed_nozzle_tower_managed_key_table()) {
        const std::string key = entry.key;
        if (ledger.managed_keys.count(key) != 0) {
            const auto applied = ledger.applied(key);
            const auto before = ledger.before(key);
            MixedNozzleTowerReviewLine line;
            line.key = key;
            line.label = entry.label;
            line.value = mixed_nozzle_tower_display_value(key, applied.value_or("absent"));
            line.provenance = MIXED_NOZZLE_TOWER_PROVENANCE_AUTOMATIC;
            line.preserved = entry.recommendation == MixedNozzleTowerRecommendation::Preserve ||
                             (applied && before && *applied == *before);
            if (entry.pending_note != nullptr) line.note = entry.pending_note;
            lines.push_back(std::move(line));
        } else if (ledger.manual_keys.count(key) != 0) {
            MixedNozzleTowerReviewLine line;
            line.key = key;
            line.label = entry.label;
            line.value = "your value";
            line.provenance = MIXED_NOZZLE_TOWER_PROVENANCE_MANUAL;
            line.preserved = true;
            lines.push_back(std::move(line));
        }
    }
    return lines;
}

// The same profile read from the presets, for the "Keep this project's current tower settings"
// explanation.
inline std::vector<MixedNozzleTowerReviewLine> mixed_nozzle_tower_current_lines(
    const DynamicPrintConfig &process, const DynamicPrintConfig *printer)
{
    std::vector<MixedNozzleTowerReviewLine> lines;
    for (const auto &entry : mixed_nozzle_tower_managed_key_table()) {
        const DynamicPrintConfig *source = entry.target == MixedNozzleTowerKeyTarget::Printer
            ? printer : &process;
        if (source == nullptr) continue;
        const auto current = mixed_nozzle_tower_read_value(*source, entry.key);
        if (!current) continue;
        MixedNozzleTowerReviewLine line;
        line.key = entry.key;
        line.label = entry.label;
        line.value = mixed_nozzle_tower_display_value(entry.key, *current);
        line.provenance = MIXED_NOZZLE_TOWER_PROVENANCE_PROJECT;
        lines.push_back(std::move(line));
    }
    return lines;
}

inline std::string mixed_nozzle_tower_join_lines(const std::vector<MixedNozzleTowerReviewLine> &lines,
                                                  const std::string &separator)
{
    std::string text;
    for (const auto &line : lines) {
        if (!text.empty()) text += separator;
        text += line.text();
    }
    return text;
}

inline std::optional<MixedNozzleTowerPolicyLedger> mixed_nozzle_tower_ledger(
    const DynamicPrintConfig &project, bool *unreadable = nullptr)
{
    if (unreadable != nullptr) *unreadable = false;
    const auto *serialized = project.option<ConfigOptionString>(MIXED_NOZZLE_TOWER_POLICY_OPTION);
    if (serialized == nullptr || serialized->value.empty()) return std::nullopt;
    auto parsed = MixedNozzleTowerPolicyLedger::parse(serialized->value);
    if (!parsed && unreadable != nullptr) *unreadable = true;
    return parsed;
}

inline bool mixed_nozzle_tower_ledger_manages(const DynamicPrintConfig &project, const std::string &key)
{
    const auto ledger = mixed_nozzle_tower_ledger(project);
    return ledger && ledger->managed_keys.count(key) != 0;
}

// Tower keys the user changed: the ledger's detached keys, plus any profile key the print preset's
// "differs from system" mask names that the ledger does not own (an owned key differs because
// Automatic wrote it).
inline std::set<std::string> mixed_nozzle_tower_user_changed_keys(
    const DynamicPrintConfig &project, const std::vector<std::string> &different_settings_to_system)
{
    std::set<std::string> changed;
    const auto ledger = mixed_nozzle_tower_ledger(project);
    if (ledger) changed = ledger->manual_keys;
    const std::size_t index = mixed_nozzle_mask_index_print();
    if (index < different_settings_to_system.size()) {
        std::vector<std::string> parsed;
        if (unescape_strings_cstyle(different_settings_to_system[index], parsed))
            for (std::string &key : parsed)
                if (mixed_nozzle_tower_managed_key(key) != nullptr &&
                    (!ledger || ledger->managed_keys.count(key) == 0))
                    changed.insert(std::move(key));
    }
    return changed;
}

// Automatic is the default when the project has no tower ownership and the user has not changed a
// profile setting. An automatic ledger stays automatic; an unreadable payload stays preserve-only.
// A project already set up without an automatic ledger chose its own tower settings, which is what
// the sidebar says, so it stays on them.
inline bool mixed_nozzle_tower_default_automatic(
    const DynamicPrintConfig &project, const std::vector<std::string> &different_settings_to_system,
    MixedNozzleSlicingMode configured_mode)
{
    bool unreadable = false;
    const auto ledger = mixed_nozzle_tower_ledger(project, &unreadable);
    if (unreadable) return false;
    if (ledger && !ledger->managed_keys.empty()) return true;
    if (configured_mode != MixedNozzleSlicingMode::Off) return false;
    return mixed_nozzle_tower_user_changed_keys(project, different_settings_to_system).empty();
}

// The text under the tower control, built from the same ledger the transaction writes so the
// preview cannot drift from the applied plan.
inline std::string mixed_nozzle_tower_choice_explanation(bool automatic,
                                                          const MixedNozzleTowerLedgerInputs &inputs)
{
    if (inputs.process == nullptr) return {};
    if (!automatic)
        return std::string("Keeps this project's current tower settings:\n") +
               mixed_nozzle_tower_join_lines(
                   mixed_nozzle_tower_current_lines(*inputs.process, inputs.printer), "\n");
    const auto ledger = mixed_nozzle_tower_build_ledger(inputs);
    return std::string("Automatic applies and keeps these tower settings:\n") +
           mixed_nozzle_tower_join_lines(mixed_nozzle_tower_review_lines(ledger), "\n");
}

inline std::vector<std::string> mixed_nozzle_tower_review_note_lines(
    const MixedNozzleTowerPolicyLedger &ledger)
{
    std::vector<std::string> notes;
    for (const auto &line : mixed_nozzle_tower_review_lines(ledger))
        notes.push_back(line.text());
    return notes;
}

// Editing a managed key in the Tab hands it back to the user: it moves to manual_keys and its
// before/applied rows are dropped, so Undo and consumer removal never restore it.
inline bool detach_mixed_nozzle_tower_key(DynamicPrintConfig &project, const std::string &key,
                                           const std::string& scope = {})
{
    const auto *entry = mixed_nozzle_tower_managed_key(key);
    if (entry == nullptr)
        return false;
    const auto *serialized = project.option<ConfigOptionString>(MIXED_NOZZLE_TOWER_POLICY_OPTION);
    const auto parsed = serialized ? MixedNozzleTowerPolicyLedger::parse(serialized->value) : std::nullopt;
    if (serialized && !serialized->value.empty() && !parsed)
        return false; // Future/corrupt payloads remain byte-for-byte intact.
    // With no ledger there is nothing to detach from. A process key still records manual intent
    // under its process scope; a printer key relies on the builder's dirty-key check instead.
    if (!parsed && (scope.empty() || entry->target == MixedNozzleTowerKeyTarget::Printer))
        return false;
    if (parsed && !scope.empty() && parsed->scope_for(*entry) != scope)
        return false;
    MixedNozzleTowerPolicyLedger remaining = parsed.value_or(MixedNozzleTowerPolicyLedger{});
    if (!parsed) remaining.process_scope = scope;
    remaining.managed_keys.erase(key);
    remaining.manual_keys.insert(key);
    remaining.before_values.erase(std::remove_if(remaining.before_values.begin(), remaining.before_values.end(),
        [&key](const auto &entry) { return entry.first == key; }), remaining.before_values.end());
    remaining.applied_values.erase(std::remove_if(remaining.applied_values.begin(), remaining.applied_values.end(),
        [&key](const auto &entry) { return entry.first == key; }), remaining.applied_values.end());
    project.set_key_value(MIXED_NOZZLE_TOWER_POLICY_OPTION, new ConfigOptionString(remaining.serialize()));
    return true;
}

inline void detach_mixed_nozzle_tower_preset(DynamicPrintConfig& project)
{
    const auto* payload = project.option<ConfigOptionString>(MIXED_NOZZLE_TOWER_POLICY_OPTION);
    if (payload && MixedNozzleTowerPolicyLedger::parse(payload->value))
        project.erase(MIXED_NOZZLE_TOWER_POLICY_OPTION);
    project.set_key_value(MIXED_NOZZLE_TOWER_POLICY_FRESH_OPTION, new ConfigOptionBool(false));
}

inline bool remove_mixed_nozzle_tower_consumer(DynamicPrintConfig& project,
    DynamicPrintConfig& process, const std::string& process_scope, std::uint64_t removed_slot,
    DynamicPrintConfig* printer = nullptr, const std::string& printer_scope = {})
{
    const auto* payload = project.option<ConfigOptionString>(MIXED_NOZZLE_TOWER_POLICY_OPTION);
    auto ledger = payload ? MixedNozzleTowerPolicyLedger::parse(payload->value) : std::nullopt;
    if (!ledger || ledger->process_scope != process_scope) return false;
    const auto old_slots = ledger->consuming_plate_ids;
    auto& slots = ledger->consuming_plate_ids;
    slots.erase(std::remove(slots.begin(), slots.end(), removed_slot), slots.end());
    for (auto& slot : slots) if (slot > removed_slot) --slot;
    if (slots == old_slots) return false;
    if (slots.empty()) {
        DynamicPrintConfig* printer_target =
            printer != nullptr && (printer_scope.empty() || printer_scope == ledger->printer_scope)
                ? printer : nullptr;
        for (const auto& key : mixed_nozzle_restore_tower_values(process, printer_target, *ledger))
            ledger->manual_keys.insert(key);
        ledger->managed_keys.clear();
        ledger->before_values.clear();
        ledger->applied_values.clear();
    }
    if (ledger->managed_keys.empty() && ledger->manual_keys.empty()) project.erase(MIXED_NOZZLE_TOWER_POLICY_OPTION);
    else project.set_key_value(MIXED_NOZZLE_TOWER_POLICY_OPTION, new ConfigOptionString(ledger->serialize()));
    return true;
}

// A fresh eligible setup owns only unset or default process values. An imported project stays
// preserve-only until the user chooses Automatic. An explicit conflict is a blocking diagnostic,
// and one-tool Body jobs stay tower-off.
inline MixedNozzleTowerPolicyEvaluation evaluate_tower_policy(const MixedNozzleTowerPolicyRequest &request)
{
    MixedNozzleTowerPolicyEvaluation result;
    if (request.stale) {
        result.admission_diagnostics.emplace_back("Mixed-Nozzle setup is stale; recompute before Apply.");
        return result;
    }
    if (request.mode == MixedNozzleSlicingMode::Off)
        return result;
    if (request.mapping_unresolved) {
        result.admission_diagnostics.emplace_back(
            "Mixed-Nozzle physical-tool mapping is unresolved; complete mapping before Apply.");
        return result;
    }
    if (std::any_of(request.resolved_physical_tools.begin(), request.resolved_physical_tools.end(),
                    [](int tool) { return tool < 0; })) {
        result.admission_diagnostics.emplace_back(
            "Mixed-Nozzle setup has an invalid physical-tool resolution; recompute before Apply.");
        return result;
    }
    std::set<int> resolved_tools(request.resolved_physical_tools.begin(), request.resolved_physical_tools.end());
    const bool actual_two_tool_need = resolved_tools.size() >= 2;
    // One-tool Body jobs and any other one-tool setup need no tower.
    if (!actual_two_tool_need)
        return result;

    result.tower_required = true;
    if (request.choice == MixedNozzleTowerChoice::Disabled) {
        result.blocking_choice = true;
        result.admission_diagnostics.emplace_back(
            "Tower is explicitly disabled for a setup that requires two tools.");
        return result;
    }
    // Automatic always applies the optimised purge route: prime_volume_mode is managed with Default
    // as its recommended value, the previous value goes in the ledger for Undo, and the review
    // lists it.

    result.manage_values = request.choice == MixedNozzleTowerChoice::Automatic &&
                           (request.provenance == MixedNozzleTowerProvenance::FreshSetup ||
                            request.provenance == MixedNozzleTowerProvenance::DeliberateAutomatic);
    if (!result.manage_values && request.provenance == MixedNozzleTowerProvenance::ImportedProject)
        result.admission_diagnostics.emplace_back(
            "Imported project has no automatic tower ownership; existing settings are preserved.");
    return result;
}

} // namespace Slic3r::GUI
