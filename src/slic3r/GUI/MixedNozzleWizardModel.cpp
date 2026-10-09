#include "MixedNozzleWizardModel.hpp"

#include "FeatureSplitEditorSupport.hpp"
#include "MixedNozzleNativeEntry.hpp"
#include "ValidationActionRouting.hpp"
#include "libslic3r/LocalesUtils.hpp"
#include "libslic3r/MixedNozzleConfig.hpp"
#include "libslic3r/MixedNozzleProcessConfig.hpp"
#include "libslic3r/RegionalLayerBands.hpp"
#include "libslic3r/Slicing.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <iomanip>
#include <limits>
#include <sstream>
#include <string_view>
#include <tuple>
#include <utility>

namespace Slic3r::GUI {

namespace {

constexpr double kWizardEpsilon = 1e-9;
// Caps how many cadence rows are materialized, not which cadences are legal. A pathological
// interval becomes an unavailable diagnostic row instead of a billion-step loop.
constexpr uint64_t kMaxMaterializedResolverCadenceChoices = 4096;

bool approximately_equal(double lhs, double rhs)
{
    const double scale = std::max({1., std::abs(lhs), std::abs(rhs)});
    return std::abs(lhs - rhs) <= kWizardEpsilon * scale;
}

bool valid_height(double value)
{
    return std::isfinite(value) && value > 0.;
}

std::string height_text(double value)
{
    std::ostringstream stream;
    stream << std::setprecision(12) << value;
    return stream.str();
}

// Two decimals, so 0.40 sits beside 0.48; a typed third decimal is kept.
std::string layer_row_text(double value)
{
    if (std::abs(std::round(value * 100.) / 100. - value) > 1e-9)
        return height_text(value);
    std::ostringstream stream;
    stream << std::fixed << std::setprecision(2) << value;
    return stream.str();
}

void add_reason(WizardCandidate &candidate, const std::string &reason)
{
    if (std::find(candidate.reason_codes.begin(), candidate.reason_codes.end(), reason) ==
        candidate.reason_codes.end())
        candidate.reason_codes.push_back(reason);
}

void add_error(WizardReview &review, const std::string &error)
{
    if (std::find(review.errors.begin(), review.errors.end(), error) == review.errors.end())
        review.errors.push_back(error);
}

const ConfigOptionFloats *option_floats(const PrintConfig &config, const char *key)
{
    return config.option<ConfigOptionFloats>(key);
}

const ConfigOptionFloat *option_float(const FullPrintConfig &config, const char *key)
{
    return config.option<ConfigOptionFloat>(key);
}

std::optional<std::vector<int>> physical_volume_types(const PrintConfig &config)
{
    if (config.nozzle_volume_type.values.size() == 2)
        return config.nozzle_volume_type.values;
    return std::nullopt;
}

std::optional<std::vector<std::string>> physical_variant_lists(const PrintConfig &config)
{
    const auto *ids = config.option<ConfigOptionInts>("printer_extruder_id");
    const auto *variants = config.option<ConfigOptionStrings>("printer_extruder_variant");
    if (ids == nullptr || variants == nullptr || ids->values.empty() ||
        ids->values.size() != variants->values.size())
        return std::nullopt;

    std::vector<std::string> result(2);
    for (size_t index = 0; index < ids->values.size(); ++index) {
        const int physical_one_based = ids->values[index];
        if (physical_one_based < 1 || physical_one_based > 2 || variants->values[index].empty())
            return std::nullopt;
        std::string &slot = result[size_t(physical_one_based - 1)];
        if (!slot.empty())
            slot += ",";
        slot += variants->values[index];
    }
    if (std::any_of(result.begin(), result.end(), [](const std::string &value) { return value.empty(); }))
        return std::nullopt;
    return result;
}

enum class SourceCompatibilityResult {
    NotProvided,
    Compatible,
    Unresolved,
    Mismatch,
};

SourceCompatibilityResult source_compatibility(
    const WizardNativeSourceCompatibility &source, const PrintConfig &effective)
{
    const auto *nozzles = option_floats(effective, "nozzle_diameter");
    const auto volumes = physical_volume_types(effective);
    const auto variants = physical_variant_lists(effective);
    if (effective.extruder_type.values.size() != 2 || !volumes || !variants)
        return SourceCompatibilityResult::Unresolved;
    if (nozzles == nullptr || nozzles->values.size() != 2 ||
        !std::all_of(nozzles->values.begin(), nozzles->values.end(), valid_height) ||
        source.role_nozzle_diameters.size() != 2 || source.process_extruder_ids.empty() ||
        source.process_extruder_ids.size() != source.process_extruder_variants.size() ||
        source.physical_variant_lists.size() != 2)
        return SourceCompatibilityResult::Unresolved;

    DynamicPrintConfig process;
    process.set_key_value("mixed_nozzle_process_nozzle_diameters",
        new ConfigOptionFloats(source.role_nozzle_diameters));
    process.set_key_value("print_extruder_id", new ConfigOptionInts(source.process_extruder_ids));
    process.set_key_value("print_extruder_variant",
        new ConfigOptionStrings(source.process_extruder_variants));

    DynamicPrintConfig printer;
    printer.set_key_value("nozzle_diameter", new ConfigOptionFloats(nozzles->values));
    auto *variant_lists = new ConfigOptionStrings;
    variant_lists->values = *variants;
    printer.set_key_value("extruder_variant_list", variant_lists);
    auto *types = new ConfigOptionEnumsGeneric;
    types->values = effective.extruder_type.values;
    printer.set_key_value("extruder_type", types);

    DynamicPrintConfig project;
    project.set_key_value("nozzle_diameter", new ConfigOptionFloats(nozzles->values));
    auto *project_volumes = new ConfigOptionEnumsGeneric;
    project_volumes->values = *volumes;
    project.set_key_value("nozzle_volume_type", project_volumes);
    const MixedNozzleProcessRoleResolution resolved =
        resolve_mixed_nozzle_process_roles(process, project, printer);
    return resolved.mapping.has_value() ? SourceCompatibilityResult::Compatible
                                        : SourceCompatibilityResult::Mismatch;
}

bool body_shared_base_is_safe(const PrintConfig &config, const MixedNozzleResolvedCadence &cadence)
{
    // The first layer is taken at the fine height because the wizard sets it later. The wizard lays
    // a prime tower, so a pair whose coarse nozzle cannot lay the base is legal only when the
    // lagging tower is available.
    constexpr bool wizard_lays_prime_tower = true;
    const bool lagging_tower_available = mixed_nozzle_tower_lagging_available(config);
    const BodySplitToolEnvelope fine = body_split_tool_envelope(
        config, cadence.fine_tool.physical_extruder, cadence.fine_height, cadence.fine_height, cadence.fine_height);
    const BodySplitToolEnvelope coarse = body_split_tool_envelope(
        config, cadence.coarse_tool.physical_extruder, cadence.fine_height, cadence.coarse_height, cadence.fine_height);
    return body_split_tool_envelope_admitted(fine, wizard_lays_prime_tower, lagging_tower_available) &&
           body_split_tool_envelope_admitted(coarse, wizard_lays_prime_tower, lagging_tower_available);
}

bool better_row(const WizardCatalogueRow *candidate, SourceCompatibilityResult candidate_source,
                const WizardCatalogueRow *current, SourceCompatibilityResult current_source)
{
    const auto source_rank = [](const WizardCatalogueRow *row, SourceCompatibilityResult source) {
        if (row == nullptr)
            return 1;
        if (row->source_preset_id && source == SourceCompatibilityResult::Compatible)
            return 4;
        // A row whose native metadata disagrees with the physical pair must not hide the resolver
        // seed for the same h/N. It is kept when no such seed exists and reported as NeedsInput.
        if (source == SourceCompatibilityResult::Unresolved || source == SourceCompatibilityResult::Mismatch)
            return 0;
        if (source == SourceCompatibilityResult::Compatible)
            return 3;
        if (row->stock_provenance)
            return 3;
        // A named catalogue row carries more provenance than a resolver seed.
        return 2;
    };
    const int candidate_rank = source_rank(candidate, candidate_source);
    const int current_rank = source_rank(current, current_source);
    if (candidate_rank != current_rank)
        return candidate_rank > current_rank;

    const std::string candidate_id = candidate == nullptr ? std::string() : candidate->stable_id;
    const std::string current_id = current == nullptr ? std::string() : current->stable_id;
    if (candidate_id != current_id)
        return candidate_id < current_id;
    const std::string candidate_source_id =
        candidate == nullptr || !candidate->source_preset_id ? std::string() : *candidate->source_preset_id;
    const std::string current_source_id =
        current == nullptr || !current->source_preset_id ? std::string() : *current->source_preset_id;
    return candidate_source_id < current_source_id;
}

struct CandidateSeed {
    double fine_height {0.};
    double coarse_height {0.};
    const WizardCatalogueRow *row {nullptr};
    SourceCompatibilityResult source {SourceCompatibilityResult::NotProvided};
    bool resolver_generated {false};
    bool current_selection {false};
    bool chosen_finish {false};
};

bool same_pair(const CandidateSeed &seed, double fine_height, double coarse_height)
{
    return approximately_equal(seed.fine_height, fine_height) &&
           approximately_equal(seed.coarse_height, coarse_height);
}

void add_seed(std::vector<CandidateSeed> &seeds, double fine_height, double coarse_height,
              const WizardCatalogueRow *row, SourceCompatibilityResult source,
              bool resolver_generated = false)
{
    if (!valid_height(fine_height) || !valid_height(coarse_height))
        return;
    const auto found = std::find_if(seeds.begin(), seeds.end(),
        [fine_height, coarse_height](const CandidateSeed &seed) {
            return same_pair(seed, fine_height, coarse_height);
        });
    if (found == seeds.end()) {
        seeds.push_back({fine_height, coarse_height, row, source, resolver_generated, false, false});
    } else if (better_row(row, source, found->row, found->source)) {
        found->row = row;
        found->source = source;
        found->resolver_generated = resolver_generated;
    }
}

std::optional<int> cadence_ratio(double fine_height, double coarse_height)
{
    if (!valid_height(fine_height) || !valid_height(coarse_height))
        return std::nullopt;
    const double raw = coarse_height / fine_height;
    if (!std::isfinite(raw) || raw > double(std::numeric_limits<int>::max()))
        return std::nullopt;
    const int ratio = int(std::lround(raw));
    if (ratio < 2 || !approximately_equal(raw, double(ratio)))
        return std::nullopt;
    return ratio;
}

bool within_resolved_height_envelope(const PrintConfig &config,
                                     const MixedNozzleResolvedTool &tool, double height)
{
    const double minimum = resolved_min_layer_height(config, tool.physical_extruder);
    const double maximum = std::min(resolved_max_layer_height(config, tool.physical_extruder),
                                    tool.nozzle_diameter);
    return std::isfinite(minimum) && std::isfinite(maximum) &&
           (height > minimum || approximately_equal(height, minimum)) &&
           (height < maximum || approximately_equal(height, maximum));
}

std::optional<bool> material_columns_match(const PrintConfig &config,
                                           size_t fine_logical_filament,
                                           size_t coarse_logical_filament)
{
    if (fine_logical_filament >= config.filament_type.size() ||
        coarse_logical_filament >= config.filament_type.size() ||
        fine_logical_filament >= config.filament_vendor.size() ||
        coarse_logical_filament >= config.filament_vendor.size() ||
        fine_logical_filament >= config.filament_colour.size() ||
        coarse_logical_filament >= config.filament_colour.size())
        return std::nullopt;
    return config.filament_type.get_at(fine_logical_filament) ==
               config.filament_type.get_at(coarse_logical_filament) &&
           config.filament_vendor.get_at(fine_logical_filament) ==
               config.filament_vendor.get_at(coarse_logical_filament) &&
           config.filament_colour.get_at(fine_logical_filament) ==
               config.filament_colour.get_at(coarse_logical_filament);
}

// Equal display columns do not prove two logical filaments can share one tool: preset IDs,
// provenance masks and the ledger must agree too. Missing IDs or an unproven sibling composition
// stay unresolved.

std::optional<size_t> native_variant_column(const PrintConfig &config,
                                             size_t logical_filament,
                                             size_t value_count,
                                             bool value_is_present)
{
    if (logical_filament >= config.filament_map.size())
        return std::nullopt;

    if (value_is_present && value_count == config.filament_map.size())
        return logical_filament; // Native composition may already have collapsed variant columns.
    const auto &variants = config.filament_extruder_variant.values;
    if (variants.empty()) {
        // An expanded value vector without the variant inventory is ambiguous and fails closed.
        if (value_is_present && value_count > config.filament_map.size())
            return std::nullopt;
        return logical_filament;
    }
    if (config.filament_self_index.values.size() != variants.size())
        return std::nullopt;

    const MixedNozzleToolResolution resolved = resolve_mixed_nozzle_tool(
        config, logical_filament, MixedNozzleResolveScope::PhysicalToolOnly);
    if (!resolved || !resolved.tool->variant_column)
        return std::nullopt;
    const int column = *resolved.tool->variant_column;
    if (column < 0 || size_t(column) >= variants.size() ||
        config.filament_self_index.values[size_t(column)] != int(logical_filament + 1))
        return std::nullopt;

    // get_config_index_base() returns column zero when the variant is absent, so recheck the
    // variant text. A Hybrid rack uses the filament's own flow type, as the resolver does.
    const size_t physical = resolved.tool->physical_extruder;
    const ExtruderType extruder_type = physical < config.extruder_type.size() ?
        ExtruderType(config.extruder_type.get_at(physical)) : etDirectDrive;
    const NozzleVolumeType volume_type = mixed_nozzle_filament_volume_type(config, physical, logical_filament);
    if (variants[size_t(column)] != get_extruder_variant_string(extruder_type, volume_type))
        return std::nullopt;
    if (value_is_present && size_t(column) >= value_count)
        return std::nullopt;
    return size_t(column);
}

std::optional<bool> native_material_settings_match(const PrintConfig &config,
                                                    size_t fine_logical_filament,
                                                    size_t coarse_logical_filament)
{
    if (fine_logical_filament >= config.filament_ids.size() ||
        coarse_logical_filament >= config.filament_ids.size() ||
        config.filament_ids.get_at(fine_logical_filament).empty() ||
        config.filament_ids.get_at(coarse_logical_filament).empty())
        return std::nullopt;
    if (config.filament_ids.get_at(fine_logical_filament) !=
        config.filament_ids.get_at(coarse_logical_filament))
        return std::nullopt;

    const size_t num_filaments = config.filament_map.size();
    if (fine_logical_filament >= num_filaments || coarse_logical_filament >= num_filaments)
        return std::nullopt;

    // Nozzle-specific siblings may carry different volumetric caps for one material; that is only a
    // conflict when provenance marks an explicit per-filament override. The dirty mask and the
    // ledger are merged so loaded and live-edited projects are checked the same way.
    const MixedNozzleLedger ledger = mixed_nozzle_read_ledger(
        config.mixed_nozzle_filament_explicit_keys.values,
        config.mixed_nozzle_filament_provenance.values, num_filaments);
    if (!ledger.qualified)
        return std::nullopt;
    const ConfigOptionStrings *difference_mask = config.option<ConfigOptionStrings>(
        "different_settings_to_system");
    if (difference_mask != nullptr && !difference_mask->values.empty() &&
        difference_mask->values.size() != mixed_nozzle_mask_length(num_filaments))
        return std::nullopt;
    const auto explicit_keys = [&ledger, &config, num_filaments](size_t logical_filament) {
        std::set<std::string> keys = mixed_nozzle_mask_keys_for_filament(
            config.different_settings_to_system.values, logical_filament, num_filaments);
        if (logical_filament < ledger.explicit_keys.size())
            keys.insert(ledger.explicit_keys[logical_filament].begin(),
                        ledger.explicit_keys[logical_filament].end());
        return keys;
    };
    // The fine and coarse variant columns of a filament vector `size` long.
    const auto paired_variant_columns = [&config, fine_logical_filament, coarse_logical_filament](size_t size) {
        return std::make_pair(native_variant_column(config, fine_logical_filament, size, true),
                              native_variant_column(config, coarse_logical_filament, size, true));
    };
    const std::set<std::string> fine_explicit_keys = explicit_keys(fine_logical_filament);
    const std::set<std::string> coarse_explicit_keys = explicit_keys(coarse_logical_filament);
    if (fine_explicit_keys != coarse_explicit_keys)
        return std::nullopt;

    // Every key must agree at the selected logical-filament column. Scalar options are process-wide
    // and cannot disagree per filament.
    for (const std::string &key : fine_explicit_keys) {
        const ConfigOption *option = config.option(key);
        if (option == nullptr)
            return std::nullopt;
        const auto *vector = dynamic_cast<const ConfigOptionVectorBase *>(option);
        if (vector == nullptr)
            continue;
        const bool variant_bound = filament_options_with_variant.count(key) != 0;
        const auto [fine_index, coarse_index] = variant_bound ? paired_variant_columns(vector->size()) :
            std::make_pair(std::optional<size_t>(fine_logical_filament), std::optional<size_t>(coarse_logical_filament));
        const std::vector<std::string> values = vector->vserialize();
        if (!fine_index || !coarse_index || *fine_index >= values.size() || *coarse_index >= values.size() ||
            values[*fine_index] != values[*coarse_index])
            return std::nullopt;
    }

    const ConfigOptionFloats *caps = config.option<ConfigOptionFloats>(
        "filament_max_volumetric_speed");
    if (caps != nullptr) {
        const auto [fine_column, coarse_column] = paired_variant_columns(caps->values.size());
        if (!fine_column || !coarse_column)
            return std::nullopt;
        const auto cap_present = [caps](size_t index) {
            return index < caps->values.size() && std::isfinite(caps->values[index]);
        };
        const bool fine_cap_present = cap_present(*fine_column);
        const bool coarse_cap_present = cap_present(*coarse_column);
        if (fine_cap_present != coarse_cap_present)
            return std::nullopt;
        if (fine_cap_present &&
            !approximately_equal(caps->values[*fine_column], caps->values[*coarse_column])) {
            // Unequal caps are admissible only when the composition carries the sibling witness for
            // both columns, on top of the checks above.
            const ConfigOptionStrings *binding = config.option<ConfigOptionStrings>(
                "mixed_nozzle_filament_binding");
            const auto native_binding = [binding](size_t index, const char *prefix) {
                return binding != nullptr && index < binding->values.size() &&
                       binding->values[index].rfind(prefix, 0) == 0;
            };
            if (!native_binding(fine_logical_filament, "base:") &&
                !native_binding(fine_logical_filament, "sibling:"))
                return std::nullopt;
            if (!native_binding(coarse_logical_filament, "base:") &&
                !native_binding(coarse_logical_filament, "sibling:"))
                return std::nullopt;
        }
    }
    return true;
}

bool selected_filaments_are_bound(const PrintConfig &config,
                                  size_t fine_logical_filament,
                                  size_t coarse_logical_filament)
{
    const MixedNozzleLedger ledger = mixed_nozzle_read_ledger(
        config.mixed_nozzle_filament_explicit_keys.values,
        config.mixed_nozzle_filament_provenance.values,
        config.filament_map.size());
    if (!ledger.qualified || fine_logical_filament >= ledger.provenance.size() ||
        coarse_logical_filament >= ledger.provenance.size())
        return false;
    return ledger.provenance[fine_logical_filament] == MixedNozzleProvenance::Bound &&
           ledger.provenance[coarse_logical_filament] == MixedNozzleProvenance::Bound;
}

std::optional<std::pair<int, int>> add_resolver_cadence_seeds(
    std::vector<CandidateSeed> &seeds, const WizardDraft &draft, const PrintConfig &config)
{
    if (!valid_height(draft.chosen_fine_height) || !draft.fine_logical_filament ||
        !draft.coarse_logical_filament || draft.mode == MixedNozzleSlicingMode::Off)
        return std::nullopt;

    const MixedNozzleToolResolution fine = resolve_mixed_nozzle_tool(
        config, *draft.fine_logical_filament, MixedNozzleResolveScope::PhysicalToolOnly);
    const MixedNozzleToolResolution coarse = resolve_mixed_nozzle_tool(
        config, *draft.coarse_logical_filament, MixedNozzleResolveScope::PhysicalToolOnly);
    if (!fine || !coarse || fine.tool->physical_extruder == coarse.tool->physical_extruder ||
        coarse.tool->nozzle_diameter <= fine.tool->nozzle_diameter ||
        !within_resolved_height_envelope(config, *fine.tool, draft.chosen_fine_height))
        return std::nullopt;

    const double coarse_minimum = resolved_min_layer_height(config, coarse.tool->physical_extruder);
    const double coarse_maximum = std::min(
        resolved_max_layer_height(config, coarse.tool->physical_extruder),
        coarse.tool->nozzle_diameter);
    if (!std::isfinite(coarse_minimum) || !std::isfinite(coarse_maximum) ||
        coarse_maximum < coarse_minimum)
        return std::nullopt;

    const long double first_ratio_value = std::ceil(
        (static_cast<long double>(coarse_minimum) - kWizardEpsilon) /
        static_cast<long double>(draft.chosen_fine_height));
    const long double last_ratio_value = std::floor(
        (static_cast<long double>(coarse_maximum) + kWizardEpsilon) /
        static_cast<long double>(draft.chosen_fine_height));
    if (!std::isfinite(first_ratio_value) || !std::isfinite(last_ratio_value) ||
        last_ratio_value < 2. || first_ratio_value > last_ratio_value ||
        first_ratio_value > static_cast<long double>(std::numeric_limits<int>::max()))
        return std::nullopt;

    // Clamp in long double before converting: a tiny fine height can make first_ratio_value very
    // negative, and converting that to int is undefined.
    const long double first_ratio_clamped = std::max(2.L, first_ratio_value);
    const long double last_ratio_clamped = std::min(
        last_ratio_value, static_cast<long double>(std::numeric_limits<int>::max()));
    if (first_ratio_clamped > last_ratio_clamped)
        return std::nullopt;
    const int first_ratio = static_cast<int>(first_ratio_clamped);
    const int last_ratio = static_cast<int>(last_ratio_clamped);
    const uint64_t ratio_count = static_cast<uint64_t>(last_ratio) -
                                 static_cast<uint64_t>(first_ratio) + 1;
    if (ratio_count > kMaxMaterializedResolverCadenceChoices)
        return std::pair<int, int>{first_ratio, last_ratio};

    for (int ratio = first_ratio;; ++ratio) {
        const double coarse_height = draft.chosen_fine_height * double(ratio);
        const MixedNozzleCadenceResolution resolved = resolve_mixed_nozzle_cadence(
            config, draft.chosen_fine_height, coarse_height,
            *draft.fine_logical_filament, *draft.coarse_logical_filament);
        if (resolved.cadence)
            add_seed(seeds, draft.chosen_fine_height, coarse_height, nullptr,
                     SourceCompatibilityResult::NotProvided, true);
        if (ratio == last_ratio)
            break;
    }
    return std::nullopt;
}

void add_height_delta_if_changed(WizardCandidate &candidate, const FullPrintConfig &config,
                                 const char *key, double value)
{
    const ConfigOptionFloat *current = option_float(config, key);
    if (current != nullptr && approximately_equal(current->value, value))
        return;
    candidate.proposed_deltas.push_back({
        key, current == nullptr ? std::string{} : height_text(current->value),
        height_text(value), "Resolver", "Shared process", "mm"});
}

void add_height_deltas(WizardCandidate &candidate, const FullPrintConfig &config)
{
    add_height_delta_if_changed(candidate, config, "layer_height", candidate.fine_height);
    if (candidate.coarse_height)
        add_height_delta_if_changed(candidate, config, "mixed_nozzle_coarse_layer_height",
                                    *candidate.coarse_height);
}

WizardCandidate build_single_nozzle_candidate(const WizardDraft &draft,
                                              const FullPrintConfig &config)
{
    WizardCandidate candidate;
    candidate.stable_id = "single-nozzle";
    candidate.kind = WizardCandidateKind::SingleNozzle;
    candidate.mode = draft.mode;
    candidate.fine_height = valid_height(draft.chosen_fine_height)
        ? draft.chosen_fine_height : config.layer_height.value;
    candidate.tier = "Single";
    add_height_delta_if_changed(candidate, config, "layer_height", candidate.fine_height);

    if (!valid_height(candidate.fine_height)) {
        candidate.eligibility = WizardCandidateEligibility::NeedsInput;
        add_reason(candidate, "FineHeightMissing");
        return candidate;
    }
    if (!draft.fine_logical_filament || !draft.coarse_logical_filament) {
        candidate.eligibility = WizardCandidateEligibility::NeedsInput;
        add_reason(candidate, "UnresolvedLogicalFilamentBinding");
        return candidate;
    }

    const MixedNozzleToolResolution fine = resolve_mixed_nozzle_tool(
        config, *draft.fine_logical_filament, MixedNozzleResolveScope::PhysicalToolOnly);
    const MixedNozzleToolResolution coarse = resolve_mixed_nozzle_tool(
        config, *draft.coarse_logical_filament, MixedNozzleResolveScope::PhysicalToolOnly);
    if (!fine || !coarse) {
        candidate.eligibility = WizardCandidateEligibility::NeedsInput;
        const MixedNozzleToolResolution *failed = fine ? &coarse : &fine;
        if (failed->diagnostic) {
            add_reason(candidate, failed->diagnostic->stable_code());
            if (failed->diagnostic->code == MixedNozzleDiagnosticCode::StaticManualMapRequired ||
                failed->diagnostic->code == MixedNozzleDiagnosticCode::LogicalFilamentMissing ||
                failed->diagnostic->code == MixedNozzleDiagnosticCode::PhysicalExtruderInvalid)
                add_reason(candidate, "UnresolvedLogicalFilamentBinding");
        } else {
            add_reason(candidate, "UnresolvedLogicalFilamentBinding");
        }
        return candidate;
    }
    // Check the envelope before any cadence arithmetic so a tiny fine height cannot start a huge
    // loop.
    if (!within_resolved_height_envelope(config, *fine.tool, candidate.fine_height)) {
        candidate.eligibility = WizardCandidateEligibility::Unsupported;
        add_reason(candidate, "MNS-CAD-A04");
        return candidate;
    }

    const std::optional<bool> material_match = material_columns_match(
        config, *draft.fine_logical_filament, *draft.coarse_logical_filament);
    if (!material_match) {
        candidate.eligibility = WizardCandidateEligibility::NeedsInput;
        add_reason(candidate, "SingleNozzleMaterialCompatibilityUnresolved");
        return candidate;
    }
    if (!*material_match) {
        candidate.eligibility = WizardCandidateEligibility::Unsupported;
        add_reason(candidate, "SingleNozzleMaterialMismatch");
        return candidate;
    }
    const std::optional<bool> native_settings_match = native_material_settings_match(
        config, *draft.fine_logical_filament, *draft.coarse_logical_filament);
    if (!native_settings_match) {
        candidate.eligibility = WizardCandidateEligibility::NeedsInput;
        add_reason(candidate, "SingleNozzleMaterialCompatibilityUnresolved");
        return candidate;
    }
    // Also rejects ambiguous many-to-one maps and missing tool coverage.
    if (!mixed_nozzle_same_material_pair(config) ||
        !selected_filaments_are_bound(config, *draft.fine_logical_filament,
                                       *draft.coarse_logical_filament)) {
        candidate.eligibility = WizardCandidateEligibility::NeedsInput;
        add_reason(candidate, "SingleNozzleMaterialCompatibilityUnresolved");
        return candidate;
    }

    candidate.eligibility = WizardCandidateEligibility::Eligible;
    return candidate;
}

} // namespace

std::vector<WizardCandidate> build_wizard_candidates(
    const WizardDraft &draft,
    const FullPrintConfig &copied_effective_config,
    const std::vector<WizardCatalogueRow> &catalogue_rows)
{
    std::vector<CandidateSeed> seeds;
    for (const WizardCatalogueRow &row : catalogue_rows) {
        const SourceCompatibilityResult source = row.source_compatibility
            ? source_compatibility(*row.source_compatibility, copied_effective_config)
            : SourceCompatibilityResult::NotProvided;
        add_seed(seeds, row.fine_height, row.coarse_height, &row, source);
    }
    // A stale chosen_coarse_height is only seeded when it is a whole cadence at the current fine
    // height; otherwise the resolver rows cover it.
    if (valid_height(draft.chosen_fine_height) && draft.chosen_coarse_height &&
        valid_height(*draft.chosen_coarse_height) &&
        cadence_ratio(draft.chosen_fine_height, *draft.chosen_coarse_height))
        add_seed(seeds, draft.chosen_fine_height, *draft.chosen_coarse_height, nullptr,
                 SourceCompatibilityResult::NotProvided);
    // Profile ratio hints do not cap this enumeration.
    const std::optional<std::pair<int, int>> unavailable_ratio_range =
        add_resolver_cadence_seeds(seeds, draft, copied_effective_config);

    for (CandidateSeed &seed : seeds) {
        seed.current_selection = valid_height(draft.chosen_fine_height) && draft.chosen_coarse_height &&
                                 same_pair(seed, draft.chosen_fine_height, *draft.chosen_coarse_height);
        seed.chosen_finish = valid_height(draft.chosen_fine_height) &&
                             approximately_equal(seed.fine_height, draft.chosen_fine_height);
    }
    std::sort(seeds.begin(), seeds.end(), [](const CandidateSeed &lhs, const CandidateSeed &rhs) {
        const bool lhs_current = lhs.current_selection;
        const bool rhs_current = rhs.current_selection;
        if (lhs_current != rhs_current)
            return lhs_current;
        if (lhs.chosen_finish != rhs.chosen_finish)
            return lhs.chosen_finish;
        // Pairs are already deduplicated by add_seed; exact comparisons keep the ordering strict
        // weak.
        if (lhs.fine_height != rhs.fine_height)
            return lhs.fine_height < rhs.fine_height;
        if (lhs.coarse_height != rhs.coarse_height)
            return lhs.coarse_height < rhs.coarse_height;
        const std::string lhs_id = lhs.row == nullptr ? std::string{} : lhs.row->stable_id;
        const std::string rhs_id = rhs.row == nullptr ? std::string{} : rhs.row->stable_id;
        return lhs_id < rhs_id;
    });

    std::vector<WizardCandidate> candidates;
    candidates.reserve(seeds.size() + 1);
    const WizardCandidate single_nozzle = build_single_nozzle_candidate(
        draft, copied_effective_config);
    bool single_nozzle_inserted = false;
    for (const CandidateSeed &seed : seeds) {
        // Single-nozzle is a finish-level choice, kept with the selected fine height. The current
        // selection still ranks above it.
        if (!single_nozzle_inserted && !seed.chosen_finish) {
            candidates.emplace_back(single_nozzle);
            single_nozzle_inserted = true;
        }
        WizardCandidate candidate;
        candidate.mode = draft.mode;
        candidate.fine_height = seed.fine_height;
        candidate.coarse_height = seed.coarse_height;
        candidate.ratio = cadence_ratio(seed.fine_height, seed.coarse_height);
        if (seed.row != nullptr)
            candidate.stable_id = seed.row->stable_id;
        else if (seed.resolver_generated && candidate.ratio)
            candidate.stable_id = "resolver-n" + std::to_string(*candidate.ratio);
        else
            candidate.stable_id = "custom-current";
        candidate.tier = seed.row == nullptr
            ? (seed.resolver_generated ? "Resolver" : "Custom")
            : (seed.row->tier.empty() ? "Custom" : seed.row->tier);
        add_height_deltas(candidate, copied_effective_config);

        if (draft.mode == MixedNozzleSlicingMode::Off) {
            candidate.eligibility = WizardCandidateEligibility::Unsupported;
            add_reason(candidate, "ModeOff");
            candidates.emplace_back(std::move(candidate));
            continue;
        }
        if (!candidate.ratio) {
            candidate.eligibility = WizardCandidateEligibility::Unsupported;
            add_reason(candidate, "CadenceRatioInvalid");
            candidates.emplace_back(std::move(candidate));
            continue;
        }
        if (!draft.fine_logical_filament || !draft.coarse_logical_filament) {
            candidate.eligibility = WizardCandidateEligibility::NeedsInput;
            add_reason(candidate, "UnresolvedLogicalFilamentBinding");
            candidates.emplace_back(std::move(candidate));
            continue;
        }

        const MixedNozzleCadenceResolution cadence = resolve_mixed_nozzle_cadence(
            copied_effective_config, seed.fine_height, seed.coarse_height,
            *draft.fine_logical_filament, *draft.coarse_logical_filament);
        if (!cadence.cadence) {
            const auto unresolved_binding = [](MixedNozzleDiagnosticCode code) {
                return code == MixedNozzleDiagnosticCode::StaticManualMapRequired ||
                       code == MixedNozzleDiagnosticCode::LogicalFilamentMissing ||
                       code == MixedNozzleDiagnosticCode::PhysicalExtruderInvalid;
            };
            const auto needs_input = [](MixedNozzleDiagnosticCode code) {
                switch (code) {
                case MixedNozzleDiagnosticCode::StaticManualMapRequired:
                case MixedNozzleDiagnosticCode::LogicalFilamentMissing:
                case MixedNozzleDiagnosticCode::PhysicalExtruderInvalid:
                case MixedNozzleDiagnosticCode::NozzleDiameterInvalid:
                case MixedNozzleDiagnosticCode::NozzleIdentityMissing:
                case MixedNozzleDiagnosticCode::NozzleIdentityInvalid:
                case MixedNozzleDiagnosticCode::NozzleVolumeMissing:
                case MixedNozzleDiagnosticCode::NozzleVolumeInvalid:
                    return true;
                default:
                    return false;
                }
            };
            candidate.eligibility = cadence.diagnostic &&
                    needs_input(cadence.diagnostic->code)
                ? WizardCandidateEligibility::NeedsInput
                : WizardCandidateEligibility::Unsupported;
            if (cadence.diagnostic) {
                add_reason(candidate, cadence.diagnostic->stable_code());
                if (unresolved_binding(cadence.diagnostic->code))
                    add_reason(candidate, "UnresolvedLogicalFilamentBinding");
            } else {
                candidate.eligibility = WizardCandidateEligibility::NeedsInput;
                add_reason(candidate, "UnresolvedLogicalFilamentBinding");
            }
        } else if (draft.mode == MixedNozzleSlicingMode::BodySplit &&
                   !body_shared_base_is_safe(copied_effective_config, *cadence.cadence)) {
            candidate.eligibility = WizardCandidateEligibility::Unsupported;
            add_reason(candidate, "BodySharedBaseHeightBelowEnvelope");
        } else {
            candidate.eligibility = WizardCandidateEligibility::Eligible;
        }

        if (seed.source == SourceCompatibilityResult::Unresolved ||
            seed.source == SourceCompatibilityResult::Mismatch) {
            candidate.source_preset_id.reset();
            add_reason(candidate, seed.source == SourceCompatibilityResult::Unresolved
                ? "SourceCompatibilityUnresolved" : "SourceCompatibilityMismatch");
            if (seed.source == SourceCompatibilityResult::Unresolved &&
                candidate.eligibility == WizardCandidateEligibility::Eligible)
                candidate.eligibility = WizardCandidateEligibility::NeedsInput;
        }

        if (!draft.resolved_physical_roles.empty() && cadence.cadence) {
            for (const WizardPhysicalRole &role : draft.resolved_physical_roles) {
                std::optional<MixedNozzleResolvedTool> resolved;
                if (draft.fine_logical_filament && role.logical_filament == *draft.fine_logical_filament)
                    resolved = cadence.cadence->fine_tool;
                else if (draft.coarse_logical_filament && role.logical_filament == *draft.coarse_logical_filament)
                    resolved = cadence.cadence->coarse_tool;
                if (!resolved || resolved->physical_extruder != role.physical_extruder ||
                    !approximately_equal(resolved->nozzle_diameter, role.nozzle_diameter)) {
                    candidate.eligibility = WizardCandidateEligibility::Unsupported;
                    add_reason(candidate, "ResolvedPhysicalRoleMismatch");
                }
            }
        }
        if (seed.row != nullptr && seed.source == SourceCompatibilityResult::Compatible &&
            candidate.eligibility == WizardCandidateEligibility::Eligible)
            candidate.source_preset_id = seed.row->source_preset_id;
        candidates.emplace_back(std::move(candidate));
    }
    if (!single_nozzle_inserted)
        candidates.emplace_back(single_nozzle);
    if (unavailable_ratio_range) {
        WizardCandidate diagnostic;
        diagnostic.stable_id = "resolver-cadence-range-unavailable";
        diagnostic.kind = WizardCandidateKind::MixedCadence;
        diagnostic.mode = draft.mode;
        diagnostic.fine_height = draft.chosen_fine_height;
        diagnostic.tier = "Resolver";
        diagnostic.eligibility = WizardCandidateEligibility::NeedsInput;
        diagnostic.unavailable_ratio_range = unavailable_ratio_range;
        add_reason(diagnostic, "CadenceEnumerationUnavailable");
        candidates.emplace_back(std::move(diagnostic));
    }
    return candidates;
}

WizardReview build_wizard_review(const WizardDraft &draft, const WizardCandidate &candidate)
{
    WizardReview review;
    review.selected_candidate_kind = draft.selected_candidate_kind.value_or(candidate.kind);
    review.preserved_locks = draft.locked_keys;
    review.affected_plate_ids.push_back(draft.scope.current_plate_id);

    const auto shared_process_delta = [&draft](const WizardKeyDelta &delta) {
        return delta.scope == "Shared process" || delta.key == "layer_height" ||
               delta.key == "mixed_nozzle_coarse_layer_height";
    };
    const auto append_delta = [&](const WizardKeyDelta &delta) {
        review.entries.push_back(delta);
        if (shared_process_delta(delta) && draft.scope.kind == WizardScopeKind::CurrentPlate &&
            !draft.scope.allow_shared_process_changes)
            add_error(review, "SharedProcessChangeOutsideScope");
        if (draft.locked_keys.count(delta.key) != 0 && delta.old_value != delta.new_value)
            add_error(review, "LockedKeyChange:" + delta.key);
    };

    for (const WizardKeyDelta &delta : candidate.proposed_deltas)
        append_delta(delta);
    for (const WizardKeyDelta &delta : draft.approved_key_deltas)
        append_delta(delta);

    for (const WizardBodyEdit &edit : draft.staged_body_edits)
        for (const auto &[key, value] : edit.staged_values)
            append_delta({key, {}, value, "Body editor", "Current plate", {}});

    if (draft.selected_candidate_kind && *draft.selected_candidate_kind != candidate.kind)
        add_error(review, "SelectedCandidateKindMismatch");

    if (review.selected_candidate_kind == WizardCandidateKind::SingleNozzle) {
        const auto is_conversion_key = [](const std::string &key) {
            return key == "mixed_nozzle_mode" || key == "mixed_nozzle_slicing_mode" ||
                   key == "filament_map" || key == "filament_map_mode" ||
                   key == "filament_nozzle_map" || key == "filament_volume_map" ||
                   key == "print_extruder_id" || key == "print_extruder_variant";
        };
        const bool conversion_reviewed = std::any_of(
            review.entries.begin(), review.entries.end(),
            [&is_conversion_key](const WizardKeyDelta &delta) {
                return delta.old_value != delta.new_value && is_conversion_key(delta.key);
            });
        if (!conversion_reviewed)
            add_error(review, "SingleNozzleConversionReviewRequired");
    }

    if (candidate.eligibility != WizardCandidateEligibility::Eligible)
        for (const std::string &reason : candidate.reason_codes)
            add_error(review, "Candidate:" + reason);

    const bool shared_process_change = std::any_of(review.entries.begin(), review.entries.end(), shared_process_delta);
    if (shared_process_change && draft.scope.kind == WizardScopeKind::CurrentPlate) {
        review.needs_shared_consent = true;
        review.shared_plate_count = std::max<std::size_t>(1, draft.scope.process_affected_plate_ids.size());
    }
    if (shared_process_change) {
        if (draft.scope.process_affected_plate_ids.empty())
            add_error(review, "SharedProcessAffectedPlatesMissing");
        else
            review.affected_plate_ids = draft.scope.process_affected_plate_ids;
    }
    std::sort(review.affected_plate_ids.begin(), review.affected_plate_ids.end());
    review.affected_plate_ids.erase(std::unique(review.affected_plate_ids.begin(), review.affected_plate_ids.end()),
                                    review.affected_plate_ids.end());
    review.can_apply = review.errors.empty() && candidate.eligibility == WizardCandidateEligibility::Eligible;
    return review;
}

std::optional<size_t> preferred_wizard_candidate(
    const std::vector<WizardCandidate> &candidates,
    std::optional<int> previous_ratio,
    double fine_height)
{
    // A row with no resolved ratio, or one that needs input, is never preselected.
    if (previous_ratio) {
        for (size_t i = 0; i < candidates.size(); ++i) {
            const WizardCandidate &candidate = candidates[i];
            if (candidate.kind == WizardCandidateKind::MixedCadence &&
                candidate.eligibility == WizardCandidateEligibility::Eligible &&
                candidate.ratio && *candidate.ratio == *previous_ratio &&
                std::abs(candidate.fine_height - fine_height) <= kWizardEpsilon * std::max(1., std::abs(fine_height)))
                return i;
        }
    }
    for (size_t i = 0; i < candidates.size(); ++i)
        if (candidates[i].eligibility == WizardCandidateEligibility::Eligible)
            return i;
    return std::nullopt;
}

std::vector<double> legal_fine_heights(const FullPrintConfig &copied_effective_config,
                                       const std::vector<WizardCatalogueRow> &catalogue_rows,
                                       const WizardHeightEnvelope &fine_envelope)
{
    // An unresolved envelope offers nothing.
    if (!std::isfinite(fine_envelope.min_height) || !std::isfinite(fine_envelope.max_height) ||
        fine_envelope.max_height <= 0. || fine_envelope.max_height < fine_envelope.min_height)
        return {};

    // Used only to reject a catalogue row that declares a different fine nozzle.
    std::optional<double> fine_diameter;
    const auto *nozzles = option_floats(copied_effective_config, "nozzle_diameter");
    if (nozzles != nullptr && nozzles->values.size() == 2 &&
        std::all_of(nozzles->values.begin(), nozzles->values.end(), valid_height))
        fine_diameter = std::min(nozzles->values[0], nozzles->values[1]);

    std::vector<double> heights;
    const auto offer = [&heights, &fine_envelope](double height) {
        if (!valid_height(height) ||
            (height < fine_envelope.min_height && !approximately_equal(height, fine_envelope.min_height)) ||
            (height > fine_envelope.max_height && !approximately_equal(height, fine_envelope.max_height)))
            return;
        if (std::none_of(heights.begin(), heights.end(),
                [height](double offered) { return approximately_equal(offered, height); }))
            heights.push_back(height);
    };

    for (const WizardCatalogueRow &row : catalogue_rows) {
        // A metadata-free custom row has no pair to contradict and is offered on its height.
        if (row.source_compatibility && row.source_compatibility->role_nozzle_diameters.size() == 2) {
            if (!fine_diameter ||
                !approximately_equal(row.source_compatibility->role_nozzle_diameters.front(), *fine_diameter))
                continue;
        }
        offer(row.fine_height);
    }
    // The height in effect is always offered.
    if (const auto *current = option_float(copied_effective_config, "layer_height"))
        offer(current->value);

    std::sort(heights.begin(), heights.end());
    return heights;
}

std::string wizard_material_label(const std::string &preset_name)
{
    // Vendor tails read " @BBL H2D 0.2 nozzle". Only the part up to "nozzle" is the variant; a user
    // suffix after it names the material and stays.
    static const std::string vendor_marker = " @";
    static const std::string nozzle_marker = " nozzle";
    const std::size_t vendor = preset_name.find(vendor_marker);
    if (vendor == std::string::npos)
        return preset_name;
    const std::size_t nozzle = preset_name.find(nozzle_marker, vendor);
    if (nozzle == std::string::npos)
        return preset_name;
    std::string name = preset_name;
    name.erase(vendor, nozzle + nozzle_marker.size() - vendor);
    return name;
}

// Whether this material has a profile column for one physical nozzle. Profiles with no variant
// columns answer true.
static bool wizard_material_has_variant(const FullPrintConfig &config, std::size_t logical_filament,
                                        std::size_t physical_extruder)
{
    const auto &variants = config.filament_extruder_variant.values;
    const auto &self_index = config.filament_self_index.values;
    // Profiles that carry no variant columns at all use the selected preset for every nozzle.
    if (variants.empty() || variants.size() != self_index.size())
        return true;
    if (physical_extruder >= config.extruder_type.size() ||
        physical_extruder >= config.nozzle_volume_type.size())
        return true;
    // A Hybrid rack prints this material through the column for its own flow type.
    const std::string wanted = get_extruder_variant_string(
        ExtruderType(config.extruder_type.get_at(physical_extruder)),
        mixed_nozzle_filament_volume_type(config, physical_extruder, logical_filament));
    if (wanted.empty())
        return true;
    for (std::size_t index = 0; index < variants.size(); ++index)
        if (variants[index] == wanted && self_index[index] == int(logical_filament) + 1)
            return true;
    return false;
}

WizardMaterialResolution wizard_material_resolution(const FullPrintConfig &config,
                                                    std::size_t logical_filament, bool coarse_role)
{
    WizardMaterialResolution result;
    const MixedNozzleToolResolution resolved = resolve_mixed_nozzle_tool(
        config, logical_filament, MixedNozzleResolveScope::PhysicalToolOnly);
    if (!resolved) {
        // The wizard sets the plate map itself, so this is a real gap in the project.
        result.needs_input = true;
        result.text = "This material has no nozzle of its own yet. Pick another one.";
        return result;
    }
    const std::size_t physical = resolved.tool->physical_extruder;
    const std::string diameter = height_text(resolved.tool->nozzle_diameter);
    if (coarse_role && !wizard_material_has_variant(config, logical_filament, physical)) {
        result.needs_input = true;
        result.text = "No " + diameter +
            " mm profile for this material yet. Pick another material for the coarse layers.";
        return result;
    }
    std::string flow;
    if (physical < config.nozzle_volume_type.size())
        flow = get_nozzle_volume_type_string(
            NozzleVolumeType(config.nozzle_volume_type.get_at(physical)));
    result.text = std::string("Prints on the ") + (physical == 0 ? "left" : "right") +
        " nozzle, " + diameter + " mm";
    if (!flow.empty())
        result.text += " " + flow;
    result.text += ".";
    return result;
}

std::vector<int> wizard_derived_filament_map(const std::vector<double> &nozzle_diameters,
                                             const std::vector<int> &current_map,
                                             std::size_t filament_count,
                                             std::optional<std::size_t> fine_logical_filament,
                                             std::optional<std::size_t> coarse_logical_filament)
{
    // Slots the wizard does not own keep the mapping they already carry. A slot with no mapping,
    // or one pointing at a tool this printer does not have, falls back to the first extruder.
    std::vector<int> map(filament_count, 1);
    for (std::size_t slot = 0; slot < filament_count && slot < current_map.size(); ++slot) {
        const int physical = current_map[slot];
        if (physical >= 1 && std::size_t(physical) <= nozzle_diameters.size())
            map[slot] = physical;
    }
    if (nozzle_diameters.size() < 2)
        return map;
    if (nozzle_diameters.size() > 2) {
        // Printers with more than two tools are not Bambu printers and print filament i on tool i,
        // so the fine and coarse slots are the toolheads the user picked.
        for (std::size_t slot = 0; slot < filament_count && slot < nozzle_diameters.size(); ++slot)
            map[slot] = int(slot) + 1;
        return map;
    }
    // Fine is the smaller installed nozzle whichever side it sits on; coarse is the other one.
    const std::size_t fine_physical = nozzle_diameters[0] <= nozzle_diameters[1] ? 0 : 1;
    const std::size_t coarse_physical = fine_physical == 0 ? 1 : 0;
    if (fine_logical_filament && *fine_logical_filament < filament_count)
        map[*fine_logical_filament] = int(fine_physical) + 1;
    if (coarse_logical_filament && *coarse_logical_filament < filament_count &&
        (!fine_logical_filament || *coarse_logical_filament != *fine_logical_filament))
        map[*coarse_logical_filament] = int(coarse_physical) + 1;
    return map;
}

std::string wizard_filament_map_review_line(const std::vector<int> &derived_map,
                                            std::optional<std::size_t> fine_logical_filament,
                                            std::optional<std::size_t> coarse_logical_filament)
{
    if (derived_map.empty())
        return {};
    const auto side = [&derived_map](std::size_t slot) {
        return derived_map[slot] <= 1 ? "left" : "right";
    };
    std::string line = "Filament map:";
    if (fine_logical_filament && *fine_logical_filament < derived_map.size())
        line += " slot " + std::to_string(*fine_logical_filament + 1) + " on the " +
                side(*fine_logical_filament) + " nozzle (fine),";
    if (coarse_logical_filament && *coarse_logical_filament < derived_map.size())
        line += " slot " + std::to_string(*coarse_logical_filament + 1) + " on the " +
                side(*coarse_logical_filament) + " nozzle (coarse),";
    line += " other slots unchanged.";
    return line;
}

bool wizard_page_can_block_next(int page)
{
    // The parts, the materials and a coarse layer while its time is unknown are the only choices
    // the wizard cannot make for the user.
    return page == 0 || page == 1 || page == 2;
}

std::string wizard_materials_block_reason(const FullPrintConfig &config,
                                          std::optional<std::size_t> fine_logical_filament,
                                          std::optional<std::size_t> coarse_logical_filament,
                                          bool body_split,
                                          const std::vector<int> &body_logical_filaments)
{
    if (!fine_logical_filament || !coarse_logical_filament)
        return "Choose a material for the fine layers and one for the coarse layers.";
    if (*fine_logical_filament == *coarse_logical_filament)
        return "Fine and coarse layers need two different material slots.";
    // An automatic plate map is not a reason to stop: the check uses the map the wizard will set.
    FullPrintConfig resolved_config = config;
    const std::size_t count = std::max({config.filament_map.values.size(),
                                        *fine_logical_filament + 1, *coarse_logical_filament + 1});
    resolved_config.filament_map.values = wizard_derived_filament_map(
        config.nozzle_diameter.values, config.filament_map.values, count,
        fine_logical_filament, coarse_logical_filament);
    resolved_config.filament_map_mode.value = fmmManual;
    const WizardMaterialResolution coarse = wizard_material_resolution(
        resolved_config, *coarse_logical_filament, true);
    if (coarse.needs_input)
        return coarse.text;
    const WizardMaterialResolution fine = wizard_material_resolution(
        resolved_config, *fine_logical_filament, false);
    if (fine.needs_input)
        return fine.text;
    if (body_split)
        for (const int slot : body_logical_filaments)
            if (slot < 0)
                return "Give every part a material before going on.";
    return {};
}

WizardModeSelection wizard_select_mode(MixedNozzleSlicingMode mode)
{
    const bool off = mode == MixedNozzleSlicingMode::Off;
    const bool body = mode == MixedNozzleSlicingMode::BodySplit;
    return {!body && !off, body, off};
}

MixedNozzleSlicingMode wizard_selected_mode(const WizardModeSelection &selection)
{
    // Both Feature and Body on is not a Body Split choice, so it reads as Feature Split. Off only
    // when Off is the card that is on.
    if (selection.off && !selection.feature && !selection.body)
        return MixedNozzleSlicingMode::Off;
    return selection.body && !selection.feature ? MixedNozzleSlicingMode::BodySplit
                                                : MixedNozzleSlicingMode::FeatureSplit;
}

std::string wizard_mode_switch_line(MixedNozzleSlicingMode before, MixedNozzleSlicingMode after)
{
    if (before == after)
        return {};
    return std::string("Mixed-Nozzle Slicing: ") + mixed_nozzle_mode_name(before) + " to " +
           mixed_nozzle_mode_name(after);
}

bool wizard_off_skips_page(int page)
{
    // The mode and Review pages are always shown; More options is never on the Next/Back path.
    return page == 1 || page == 2;
}

std::vector<WizardMaterialRow> wizard_material_rows(const std::vector<std::string> &labels,
                                                    const std::vector<std::string> &filament_colours)
{
    std::vector<WizardMaterialRow> rows;
    rows.reserve(labels.size());
    for (std::size_t slot = 0; slot < labels.size(); ++slot) {
        WizardMaterialRow row;
        row.slot = slot;
        row.label = labels[slot];
        // A project mid-edit can hold fewer colours than materials. The row index is the slot the
        // picker reports, so no row is dropped.
        row.colour = WIZARD_MATERIAL_FALLBACK_COLOUR;
        if (slot < filament_colours.size()) {
            std::string colour = filament_colours[slot];
            // A multi-colour pack separates colours with semicolons; show the first, as the sidebar
            // does.
            const std::size_t separator = colour.find(';');
            if (separator != std::string::npos)
                colour.erase(separator);
            const std::size_t first = colour.find_first_not_of(" \t");
            const std::size_t last = colour.find_last_not_of(" \t");
            colour = first == std::string::npos ? std::string{} : colour.substr(first, last - first + 1);
            if (!colour.empty() && colour.front() == '#')
                row.colour = colour;
        }
        rows.push_back(std::move(row));
    }
    return rows;
}

WizardEstimate wizard_estimate(double seconds, unsigned int switches, double tower_mm3)
{
    WizardEstimate estimate;
    estimate.status = WizardEstimateStatus::Estimated;
    estimate.seconds = seconds;
    estimate.low = seconds * (1. - kWizardEstimateMargin);
    estimate.high = seconds * (1. + kWizardEstimateMargin);
    estimate.switches = switches;
    estimate.tower_mm3 = tower_mm3;
    return estimate;
}

namespace {

// "about 6 h 26 min", to the nearest minute and never less than one.
std::string estimate_duration_text(double seconds)
{
    const long minutes = std::max(1L, std::lround(std::max(0., seconds) / 60.));
    std::string text = "about ";
    if (minutes >= 60) {
        text += std::to_string(minutes / 60) + " h";
        // Two-digit minutes after hours: "2 h 01 min".
        if (minutes % 60 != 0)
            text += std::string(minutes % 60 < 10 ? " 0" : " ") + std::to_string(minutes % 60) + " min";
    } else {
        text += std::to_string(minutes) + " min";
    }
    return text;
}

} // namespace

// "about 6 h 30 min, 448 switches", "estimating", "could not be sliced", or "no estimate" with its
// reason.
static std::string wizard_estimate_text(const WizardEstimate &estimate)
{
    switch (estimate.status) {
    case WizardEstimateStatus::Pending:
        return "estimating";
    case WizardEstimateStatus::Failed:
        return "could not be sliced";
    case WizardEstimateStatus::Unavailable:
        return estimate.note.empty() ? std::string("no estimate") : "no estimate: " + estimate.note;
    case WizardEstimateStatus::Estimated:
        break;
    }
    std::string text = estimate_duration_text(estimate.seconds);
    if (estimate.switches == 0)
        text += ", no switches";
    else if (estimate.switches == 1)
        text += ", 1 switch";
    else
        text += ", " + std::to_string(estimate.switches) + " switches";
    return text;
}

// Plain facts about the selected row: switching saves no time, a hand pick is slower than the
// fastest, or the row is slower than all-fine. Empty when there is nothing to say; never blocks
// Apply.
static std::string wizard_cadence_selection_note(const WizardCadencePage &page,
                                                 std::optional<std::size_t> selected_row,
                                                 bool explicit_pick)
{
    const auto estimated = [](const std::optional<WizardEstimate> &estimate) -> const WizardEstimate * {
        return estimate && estimate->status == WizardEstimateStatus::Estimated ? &*estimate : nullptr;
    };
    const WizardEstimate *baseline = estimated(page.baseline_estimate);
    std::vector<std::string> lines;
    // Slower means more than 2% slower, the same test "No clear fastest" uses.
    if (page.ranked && baseline != nullptr && !page.rows.empty() &&
        std::all_of(page.rows.begin(), page.rows.end(), [&estimated, baseline](const WizardCadenceRow &row) {
            const WizardEstimate *estimate = estimated(row.estimate);
            return estimate != nullptr && wizard_seconds_slower(baseline->seconds, estimate->seconds);
        }))
        lines.push_back("On this model, switching nozzles does not save time.");

    const WizardEstimate *chosen = selected_row && *selected_row < page.rows.size()
        ? estimated(page.rows[*selected_row].estimate) : nullptr;
    if (chosen != nullptr) {
        const WizardEstimate *fastest = page.ranked ? estimated(page.rows.front().estimate) : nullptr;
        // Only the user's own pick can sit outside the fastest group; say what it costs.
        const bool slower_than_fastest = explicit_pick && fastest != nullptr && page.fastest_group > 0 &&
                                         *selected_row >= page.fastest_group;
        const bool slower_than_baseline = baseline != nullptr &&
                                          wizard_seconds_slower(baseline->seconds, chosen->seconds);
        const std::string fastest_part = slower_than_fastest
            ? estimate_duration_text(chosen->seconds - fastest->seconds) + " slower than the fastest row" : std::string();
        const std::string baseline_part = slower_than_baseline
            ? estimate_duration_text(chosen->seconds - baseline->seconds) +
              " slower than printing everything on the fine nozzle" : std::string();
        if (slower_than_fastest && slower_than_baseline)
            lines.push_back("This choice is " + fastest_part + " and " + baseline_part + ".");
        else if (slower_than_fastest)
            lines.push_back("This choice is " + fastest_part + ".");
        else if (slower_than_baseline)
            lines.push_back("This choice is " + baseline_part + ".");
    }
    std::string note;
    for (const std::string &line : lines)
        note += (note.empty() ? "" : " ") + line;
    return note;
}

std::optional<std::size_t> wizard_ranked_selection(const WizardCadencePage &page,
                                                   std::optional<std::size_t> kept_candidate,
                                                   bool explicit_pick)
{
    const auto row_of_kept = [&page, &kept_candidate]() -> std::optional<std::size_t> {
        if (kept_candidate)
            for (std::size_t index = 0; index < page.rows.size(); ++index)
                if (page.rows[index].candidate_index == *kept_candidate)
                    return index;
        return std::nullopt;
    };
    // A row that could not be sliced is never kept selected, not even the project's own row.
    const auto unsliceable = [&page](std::size_t index) {
        return page.rows[index].estimate && wizard_estimate_unsliceable(*page.rows[index].estimate);
    };
    // Only a row picked in this session outranks the estimates; a static default is not a choice.
    if (explicit_pick)
        if (const std::optional<std::size_t> kept = row_of_kept(); kept && !unsliceable(*kept))
            return kept;
    // On a tie, prefer the tied row with the fewest nozzle changes (fewer restart blobs, less
    // tower), then the first of those.
    if (page.ranked && !page.rows.empty()) {
        std::size_t best = 0;
        for (std::size_t index = 1; index < page.fastest_group && index < page.rows.size(); ++index) {
            const std::optional<WizardEstimate> &candidate = page.rows[index].estimate;
            const std::optional<WizardEstimate> &current = page.rows[best].estimate;
            if (candidate && current && candidate->switches < current->switches)
                best = index;
        }
        return best;
    }
    // Unranked rows are timed rows first, fastest first, so the first row that slices is the best
    // place to move to. With nothing kept the dialog shows the first row.
    const std::optional<std::size_t> kept = row_of_kept();
    if (kept.value_or(0) < page.rows.size() && unsliceable(kept.value_or(0)))
        for (std::size_t index = 0; index < page.rows.size(); ++index)
            if (!unsliceable(index))
                return index;
    return kept;
}

static std::string wizard_speed_time_text(const WizardEstimate &estimate)
{
    switch (estimate.status) {
    case WizardEstimateStatus::Pending:
        // A row not started yet. The one being sliced says so instead.
        return "waiting";
    case WizardEstimateStatus::Failed:
        return "could not be sliced";
    case WizardEstimateStatus::Unavailable:
        return estimate.note.empty() ? std::string("no time") : "no time: " + estimate.note;
    case WizardEstimateStatus::Estimated:
        break;
    }
    std::string text = estimate_duration_text(estimate.seconds);
    if (estimate.switches == 0)
        text += ", no nozzle changes";
    else if (estimate.switches == 1)
        text += ", 1 nozzle change";
    else
        text += ", " + std::to_string(estimate.switches) + " nozzle changes";
    return text;
}

std::string wizard_speed_row_label(const WizardCadenceRow &row, MixedNozzleSlicingMode mode)
{
    std::string label = layer_row_text(row.coarse_height) + " mm";
    if (mode == MixedNozzleSlicingMode::BodySplit)
        label += " layers";
    if (row.current)
        label += " (now)";
    if (row.ratio > 0)
        label += "  N=" + std::to_string(row.ratio);
    if (row.estimate)
        label += "  " + (row.slicing && row.estimate->status == WizardEstimateStatus::Pending
                             ? std::string("slicing now...") : wizard_speed_time_text(*row.estimate));
    return label;
}

std::string wizard_coarse_layer_help(MixedNozzleSlicingMode mode, const FullPrintConfig &copied_effective_config)
{
    std::string text = std::string("N is how many fine layers make one coarse layer. A larger N gives thicker "
                                   "coarse layers, so ") +
                       (mode == MixedNozzleSlicingMode::BodySplit ? "the coarse parts print" : "the inside prints") +
                       " faster with fewer nozzle changes.";
    const MixedNozzleCoarseEnvelope envelope = mixed_nozzle_coarse_envelope(copied_effective_config);
    if (envelope.resolved)
        text += " The " + height_text(envelope.nozzle_diameter) + " mm nozzle prints at most " +
                layer_row_text(envelope.maximum) + " mm.";
    return text;
}

std::string wizard_speed_row_tag(const WizardCadencePage &page, std::size_t row)
{
    if (!page.ranked || row >= page.rows.size())
        return {};
    if (page.fastest_group >= 2)
        return row < page.fastest_group ? "About as fast" : std::string();
    return row == 0 ? "Fastest" : std::string();
}

std::string wizard_speed_progress_line(const WizardCadencePage &page, bool ranking_available)
{
    if (!ranking_available)
        return "Print times are not available here, so the rows are in layer order.";
    if (page.rows.empty())
        return {};
    std::size_t done = 0;
    bool pending = false;
    bool every_row = true;
    for (const WizardCadenceRow &row : page.rows) {
        const bool waiting = !row.estimate || row.estimate->status == WizardEstimateStatus::Pending;
        pending = pending || waiting;
        done += waiting ? 0 : 1;
        every_row = every_row && row.estimate && row.estimate->status == WizardEstimateStatus::Estimated;
    }
    if (pending)
        return "Working out print times: " + std::to_string(done) + " of " + std::to_string(page.rows.size()) +
               " done. Each time comes from a full slice, so large models can take a few minutes. "
               "Pick a row yourself to go on before they finish.";
    if (every_row)
        return {};
    // Rows with a time go on top, fastest first; each other row says why it has none.
    const bool any_timed = std::any_of(page.rows.begin(), page.rows.end(), [](const WizardCadenceRow &row) {
        return row.estimate && row.estimate->status == WizardEstimateStatus::Estimated;
    });
    return any_timed ? "Rows with a time are listed fastest first."
                     : "No row has a time, so the rows are in layer order.";
}

std::string wizard_speed_block_reason(const WizardCadencePage &page, std::optional<std::size_t> selected_row,
                                      bool explicit_pick, bool ranking_available)
{
    if (!ranking_available || explicit_pick || !selected_row || *selected_row >= page.rows.size())
        return {};
    const std::optional<WizardEstimate> &estimate = page.rows[*selected_row].estimate;
    if (estimate && estimate->status != WizardEstimateStatus::Pending)
        return {};
    return "The print times are still coming in. Wait for them, or pick a coarse layer yourself to go on now.";
}

void wizard_mark_current(WizardCadencePage &page, double fine_height,
                         const std::optional<std::pair<double, double>> &existing)
{
    const std::optional<std::size_t> current = wizard_existing_row(page, fine_height, existing);
    for (std::size_t index = 0; index < page.rows.size(); ++index)
        page.rows[index].current = current && *current == index;
}

void wizard_mark_slicing(WizardCadencePage &page, const std::vector<WizardCandidate> &candidates,
                         const std::optional<std::string> &slicing_row, bool baseline_slicing)
{
    for (WizardCadenceRow &row : page.rows)
        row.slicing = slicing_row && row.candidate_index < candidates.size() &&
                      candidates[row.candidate_index].stable_id == *slicing_row;
    page.baseline_slicing = baseline_slicing;
}

std::optional<std::size_t> wizard_existing_row(const WizardCadencePage &page, double fine_height,
                                               const std::optional<std::pair<double, double>> &existing)
{
    if (!existing || !approximately_equal(existing->first, fine_height))
        return std::nullopt;
    for (std::size_t index = 0; index < page.rows.size(); ++index)
        if (approximately_equal(page.rows[index].coarse_height, existing->second))
            return index;
    return std::nullopt;
}

std::string wizard_speed_times_note(const WizardCadencePage &page)
{
    // The process basis keeps the current process when it is already an MN preset for this mode and
    // pair, so reopening the step shows times for the user's own settings.
    const bool any_time = std::any_of(page.rows.begin(), page.rows.end(), [](const WizardCadenceRow &row) {
        return row.estimate && row.estimate->status == WizardEstimateStatus::Estimated;
    });
    if (!any_time)
        return {};
    return "These times use the preset's current settings. Changing walls, infill or other settings later "
           "will change them, and can change which row is fastest. To see them again with your settings, "
           "open Change... in the Mixed Nozzle panel.";
}

WizardSpeedProgress wizard_speed_progress(const WizardCadencePage &page)
{
    WizardSpeedProgress progress;
    progress.total = page.rows.size();
    for (const WizardCadenceRow &row : page.rows) {
        const bool waiting = !row.estimate || row.estimate->status == WizardEstimateStatus::Pending;
        progress.done += waiting ? 0 : 1;
        progress.working = progress.working || waiting;
    }
    // Without ranking no row has an estimate.
    const bool ranking = std::any_of(page.rows.begin(), page.rows.end(),
                                     [](const WizardCadenceRow &row) { return row.estimate.has_value(); });
    progress.working = progress.working && ranking;
    return progress;
}

bool wizard_switching_saves_no_time(const WizardCadencePage &page)
{
    const std::optional<WizardEstimate> &baseline = page.baseline_estimate;
    if (!page.ranked || page.rows.empty() || !baseline || baseline->status != WizardEstimateStatus::Estimated)
        return false;
    return std::all_of(page.rows.begin(), page.rows.end(), [&baseline](const WizardCadenceRow &row) {
        return row.estimate && row.estimate->status == WizardEstimateStatus::Estimated &&
               wizard_seconds_slower(baseline->seconds, row.estimate->seconds);
    });
}

std::string wizard_speed_summary_line(const WizardCadencePage &page, double fine_height,
                                      std::optional<std::size_t> selected_row, bool explicit_pick)
{
    const std::optional<WizardEstimate> &baseline = page.baseline_estimate;
    // Whenever ranking runs, say where the one-nozzle baseline is, even before it has a time.
    if (!baseline)
        return {};
    if (baseline->status != WizardEstimateStatus::Estimated) {
        const std::string state =
            baseline->status == WizardEstimateStatus::Unavailable && !baseline->note.empty() ? "no time: " + baseline->note :
            baseline->status == WizardEstimateStatus::Pending && page.baseline_slicing ? std::string("slicing now...") :
                                                                                          wizard_speed_time_text(*baseline);
        // "slicing now..." and a note that is a whole sentence ("Add a model ... times.") already
        // end it.
        const bool ends_sentence = !state.empty() && state.back() == '.';
        return "One nozzle only, " + height_text(fine_height) + " mm everywhere: " + state + (ends_sentence ? "" : ".");
    }
    std::string line = "One nozzle only, " + height_text(fine_height) + " mm everywhere: " +
                       estimate_duration_text(baseline->seconds) + ".";
    if (wizard_switching_saves_no_time(page)) {
        const double faster = page.rows.front().estimate->seconds - baseline->seconds;
        line += " On this model, switching nozzles does not save time. One nozzle only is " +
                estimate_duration_text(faster) + " faster.";
        return line;
    }
    if (page.ranked && !page.rows.empty() && page.rows.front().estimate &&
        wizard_seconds_slower(page.rows.front().estimate->seconds, baseline->seconds))
        line += " The fastest choice saves " +
                estimate_duration_text(baseline->seconds - page.rows.front().estimate->seconds) + ".";
    const std::string note = wizard_cadence_selection_note(page, selected_row, explicit_pick);
    if (!note.empty() && note.rfind("On this model", 0) != 0)
        line += " " + note;
    return line;
}

std::string wizard_no_coarse_layer_line(double fine_height, const FullPrintConfig &copied_effective_config)
{
    // Two nozzles of one size have no coarse layer at any fine layer, so say that rather than ask
    // for a smaller fine layer.
    const std::vector<double> &pair = copied_effective_config.nozzle_diameter.values;
    if (pair.size() == 2 && pair[0] > 0. && std::abs(pair[0] - pair[1]) < 1e-6)
        return "Both nozzles are " + height_text(pair[0]) + " mm, so there is no coarse layer to pick. Set the "
               "left and right Nozzle boxes in the sidebar to two different sizes, then open setup again.";
    const MixedNozzleCoarseEnvelope envelope = mixed_nozzle_coarse_envelope(copied_effective_config);
    std::string line = "No coarse layer works with a " + height_text(fine_height) + " mm fine layer";
    if (envelope.resolved)
        line += " on the " + height_text(envelope.nozzle_diameter) + " mm nozzle";
    return line + ". Pick a smaller fine layer above.";
}

WizardDraft wizard_candidate_draft(const WizardDraft &draft, const WizardCandidate &candidate,
                                   const std::vector<WizardBodyAssignment> &body)
{
    WizardDraft staged = draft;
    staged.selected_candidate_kind = candidate.kind;
    // The process is the step's one basis (wizard_process_basis), shared by every row, so the draft
    // keeps whatever it already carries.
    staged.chosen_fine_height = candidate.fine_height;
    staged.chosen_coarse_height = candidate.coarse_height;
    staged.chosen_cadence_ratio = candidate.ratio;
    staged.staged_body_edits.clear();
    if (staged.mode == MixedNozzleSlicingMode::BodySplit)
        for (const WizardBodyAssignment &assignment : body) {
            const double height = assignment.coarse ? candidate.coarse_height.value_or(0.)
                                                    : candidate.fine_height;
            staged.staged_body_edits.push_back({assignment.object_id, assignment.volume_id,
                {{"extruder", std::to_string(assignment.logical_filament + 1)},
                 {"regional_layer_height", ConfigOptionFloat(height).serialize()}}});
        }
    return staged;
}

namespace {

WizardCadencePage unranked_cadence_page(const std::vector<WizardCandidate> &candidates,
                                        double fine_height,
                                        const FullPrintConfig &copied_effective_config)
{
    WizardCadencePage page;
    if (!valid_height(fine_height))
        return page;
    // The one admissible list, so every row here is one Apply accepts.
    const std::vector<double> admissible = mixed_nozzle_admissible_coarse_heights(
        fine_height, copied_effective_config);
    for (std::size_t index = 0; index < candidates.size(); ++index) {
        const WizardCandidate &candidate = candidates[index];
        if (candidate.kind == WizardCandidateKind::SingleNozzle) {
            // The baseline is a different configuration, not a cadence, so it is not a pickable
            // row.
            if (page.single_nozzle.empty())
                page.single_nozzle = candidate.eligibility == WizardCandidateEligibility::Eligible
                    ? "One nozzle only: print the whole job on one nozzle at the fine layer."
                    : "One nozzle only: needs a separate material check first.";
            continue;
        }
        if (candidate.eligibility != WizardCandidateEligibility::Eligible || !candidate.ratio ||
            !candidate.coarse_height || !approximately_equal(candidate.fine_height, fine_height))
            continue;
        if (std::none_of(admissible.begin(), admissible.end(), [&candidate](double height) {
                return approximately_equal(height, *candidate.coarse_height); }))
            continue;
        if (std::any_of(page.rows.begin(), page.rows.end(), [&candidate](const WizardCadenceRow &row) {
                return row.ratio == *candidate.ratio; }))
            continue;
        WizardCadenceRow row;
        row.candidate_index = index;
        row.coarse_height = *candidate.coarse_height;
        row.ratio = *candidate.ratio;
        // A row that would need a stable code or a review is not offered at all.
        row.label = "coarse " + height_text(row.coarse_height) + " mm, every " +
                    std::to_string(row.ratio) + " layers";
        page.rows.push_back(std::move(row));
    }
    std::sort(page.rows.begin(), page.rows.end(),
              [](const WizardCadenceRow &lhs, const WizardCadenceRow &rhs) {
                  return lhs.ratio < rhs.ratio;
              });

    // Say which coarse layer is not offered and why, not just the limit.
    const MixedNozzleCoarseEnvelope envelope = mixed_nozzle_coarse_envelope(copied_effective_config);
    if (!envelope.resolved)
        return page;
    const auto excluded = [&envelope, fine_height](int ratio) -> std::string {
        const double height = fine_height * double(ratio);
        const std::string layer = "A " + height_text(height) + " mm coarse layer (" + std::to_string(ratio) +
                                  " fine layers) is not offered: the " + height_text(envelope.nozzle_diameter) +
                                  " mm nozzle's ";
        if (height > envelope.maximum + 1e-9)
            return layer + "limit is " + height_text(envelope.maximum) + " mm.";
        if (height + 1e-9 < envelope.minimum)
            return layer + "minimum is " + height_text(envelope.minimum) + " mm.";
        return {};
    };
    std::string sentence;
    if (!page.rows.empty() && page.rows.front().ratio > 2)
        sentence = excluded(2);
    const int next_ratio = (page.rows.empty() ? 1 : page.rows.back().ratio) + 1;
    const std::string above = excluded(next_ratio);
    if (!above.empty())
        sentence = sentence.empty() ? above : sentence + " " + above;
    page.exclusions = sentence;
    return page;
}

// "coarse 0.56 mm (about 1 h 58 min)".
std::string named_row_time(const WizardCadenceRow &row, const WizardEstimate &estimate)
{
    return "coarse " + height_text(row.coarse_height) + " mm (" + estimate_duration_text(estimate.seconds) + ")";
}

// Put each row's time on its label and, once every row has one, order fastest first. Runs after the
// exclusions sentence, which reads the rows in ratio order.
void rank_cadence_page(WizardCadencePage &page, const std::vector<WizardCandidate> &candidates,
                       double fine_height, const std::optional<WizardEstimate> &baseline)
{
    const auto estimate_of = [&candidates](const WizardCadenceRow &row) -> const WizardEstimate * {
        if (row.candidate_index >= candidates.size() || !candidates[row.candidate_index].estimate)
            return nullptr;
        return &*candidates[row.candidate_index].estimate;
    };
    const bool ranking = baseline.has_value() ||
        std::any_of(page.rows.begin(), page.rows.end(),
                    [&estimate_of](const WizardCadenceRow &row) { return estimate_of(row) != nullptr; });
    if (!ranking)
        return;

    page.baseline_estimate = baseline;
    if (baseline)
        page.baseline = height_text(fine_height) + " mm everywhere on the fine nozzle, no switching: " +
            (baseline->status == WizardEstimateStatus::Estimated ? estimate_duration_text(baseline->seconds)
                                                                  : wizard_estimate_text(*baseline));
    if (page.rows.empty())
        return;

    bool pending = false;
    bool every_row = true;
    for (WizardCadenceRow &row : page.rows) {
        const WizardEstimate *estimate = estimate_of(row);
        const WizardEstimate shown = estimate == nullptr ? WizardEstimate{} : *estimate;
        row.estimate = shown;
        row.label += ": " + wizard_estimate_text(shown);
        pending = pending || shown.status == WizardEstimateStatus::Pending;
        every_row = every_row && shown.status == WizardEstimateStatus::Estimated;
    }

    if (every_row) {
        std::stable_sort(page.rows.begin(), page.rows.end(),
            [&estimate_of](const WizardCadenceRow &lhs, const WizardCadenceRow &rhs) {
                return estimate_of(lhs)->seconds < estimate_of(rhs)->seconds;
            });
        page.ranked = true;
        // Rows within 2% of the fastest are tied with it. Rows are sorted, so the group is the
        // first few.
        page.fastest_group = 1;
        while (page.fastest_group < page.rows.size() &&
               wizard_seconds_tie(estimate_of(page.rows[0])->seconds,
                                  estimate_of(page.rows[page.fastest_group])->seconds))
            ++page.fastest_group;
        if (page.fastest_group >= 2) {
            page.no_clear_fastest = true;
            // Each tied row with its own time.
            std::string tied;
            for (std::size_t index = 0; index < page.fastest_group; ++index) {
                if (index > 0)
                    tied += index + 1 == page.fastest_group ? " and " : ", ";
                tied += named_row_time(page.rows[index], *estimate_of(page.rows[index]));
            }
            page.status = "No clear fastest: " + tied + " are within 2% of each other; pick either.";
        } else {
            page.fastest_row = 0;
            page.status = "Fastest first: " + named_row_time(page.rows[0], *estimate_of(page.rows[0])) +
                          ". Each time comes from a full slice of that row.";
        }
    } else {
        // While not every row has a time, timed rows go on top fastest first and the rest stay
        // below in layer order.
        std::stable_sort(page.rows.begin(), page.rows.end(),
            [&estimate_of](const WizardCadenceRow &lhs, const WizardCadenceRow &rhs) {
                const WizardEstimate *left = estimate_of(lhs);
                const WizardEstimate *right = estimate_of(rhs);
                const bool left_timed = left != nullptr && left->status == WizardEstimateStatus::Estimated;
                const bool right_timed = right != nullptr && right->status == WizardEstimateStatus::Estimated;
                if (left_timed != right_timed)
                    return left_timed;
                return left_timed && left->seconds < right->seconds;
            });
        const bool any_timed = std::any_of(page.rows.begin(), page.rows.end(), [](const WizardCadenceRow &row) {
            return row.estimate && row.estimate->status == WizardEstimateStatus::Estimated;
        });
        if (pending)
            page.status = "Estimating the print time of each row from a full slice. You can pick a row "
                          "and go on at any time.";
        else
            page.status = any_timed ? "Rows with a time are listed fastest first."
                                    : "No row has a time, so the rows are in layer order.";
    }
}

} // namespace

WizardCadencePage wizard_cadence_page(const std::vector<WizardCandidate> &candidates,
                                      double fine_height,
                                      const FullPrintConfig &copied_effective_config,
                                      const std::optional<WizardEstimate> &baseline)
{
    WizardCadencePage page = unranked_cadence_page(candidates, fine_height, copied_effective_config);
    rank_cadence_page(page, candidates, fine_height, baseline);
    return page;
}

namespace {

// The resolved fine tool's own printable range: its machine limits, capped by its diameter.
struct WizardFineNozzle {
    bool resolved {false};
    std::size_t physical_extruder {0};
    double diameter {0.};
    double minimum {0.};
    double maximum {0.};
};

WizardFineNozzle resolve_wizard_fine_nozzle(const FullPrintConfig &config,
                                            std::optional<std::size_t> fine_logical_filament)
{
    WizardFineNozzle nozzle;
    if (!fine_logical_filament)
        return nozzle;
    const MixedNozzleToolResolution fine = resolve_mixed_nozzle_tool(
        config, *fine_logical_filament, MixedNozzleResolveScope::PhysicalToolOnly);
    if (!fine)
        return nozzle;
    nozzle.physical_extruder = fine.tool->physical_extruder;
    nozzle.diameter = fine.tool->nozzle_diameter;
    nozzle.minimum = resolved_min_layer_height(config, nozzle.physical_extruder);
    nozzle.maximum = std::min(resolved_max_layer_height(config, nozzle.physical_extruder),
                              nozzle.diameter);
    nozzle.resolved = std::isfinite(nozzle.minimum) && std::isfinite(nozzle.maximum) &&
                      nozzle.maximum >= nozzle.minimum;
    return nozzle;
}

} // namespace

WizardFinishPage wizard_finish_page(const FullPrintConfig &copied_effective_config,
                                    const std::vector<double> &offered_heights,
                                    std::optional<std::size_t> fine_logical_filament,
                                    double current_fine_height)
{
    WizardFinishPage page;
    const WizardFineNozzle nozzle = resolve_wizard_fine_nozzle(
        copied_effective_config, fine_logical_filament);
    // Name the nozzle these heights belong to, with the range it can print.
    page.nozzle_line = nozzle.resolved
        ? "Fine nozzle: " + height_text(nozzle.diameter) + " mm (" +
          (nozzle.physical_extruder == 0 ? "left" : "right") + "), prints " +
          height_text(nozzle.minimum) + " to " + height_text(nozzle.maximum) + " mm"
        : "Fine nozzle: not resolved yet. Choose the fine material in step 2.";
    if (offered_heights.empty())
        return page;

    std::vector<std::size_t> cadence_counts;
    for (std::size_t i = 0; i < offered_heights.size(); ++i) {
        const double height = offered_heights[i];
        const std::vector<double> coarse = mixed_nozzle_admissible_coarse_heights(
            height, copied_effective_config);
        WizardFineHeightRow row;
        row.height = height;
        // Smallest is the finest, largest the fastest at this nozzle; the coarse layers each opens
        // are listed under it.
        std::vector<std::string> tags;
        if (offered_heights.size() > 1 && i == 0)
            tags.emplace_back("finest");
        if (offered_heights.size() > 1 && i + 1 == offered_heights.size())
            tags.emplace_back("fastest");
        if (approximately_equal(height, current_fine_height))
            tags.emplace_back("now");
        row.label = layer_row_text(height) + " mm";
        for (std::size_t tag = 0; tag < tags.size(); ++tag)
            row.label += (tag == 0 ? " (" : ", ") + tags[tag] + (tag + 1 == tags.size() ? ")" : "");
        page.rows.push_back(std::move(row));
        cadence_counts.push_back(coarse.size());
    }

    // Keep the height in effect. Otherwise prefer the one that opens the most cadences, the finer
    // on a tie.
    std::optional<std::size_t> recommended;
    for (std::size_t i = 0; i < offered_heights.size(); ++i)
        if (approximately_equal(offered_heights[i], current_fine_height))
            recommended = i;
    if (!recommended)
        for (std::size_t i = 0; i < cadence_counts.size(); ++i)
            if (!recommended || cadence_counts[i] > cadence_counts[*recommended])
                recommended = i;
    if (recommended && *recommended < page.rows.size())
        page.rows[*recommended].recommended = true;
    return page;
}

std::string wizard_custom_fine_height_rejection(const FullPrintConfig &copied_effective_config,
                                                std::optional<std::size_t> fine_logical_filament,
                                                double height)
{
    const WizardFineNozzle nozzle = resolve_wizard_fine_nozzle(
        copied_effective_config, fine_logical_filament);
    if (!nozzle.resolved)
        return "Choose the fine material in step 2, so this height can be checked against "
               "its nozzle.";
    return mixed_nozzle_height_limit_sentence(height, nozzle.diameter, nozzle.minimum,
                                              nozzle.maximum);
}

namespace {

// The floor Slicing.cpp applies to a machine limit left at zero.
constexpr double kWizardDefaultMinLayerHeight = 0.07;

} // namespace

WizardPairAlignment wizard_align_to_project_pair(
    const std::vector<double> &config_nozzle_diameters,
    const std::vector<double> &config_min_layer_heights,
    const std::vector<double> &config_max_layer_heights,
    const std::vector<double> &project_pair,
    const std::vector<WizardNozzleLimitRow> &known_limits)
{
    WizardPairAlignment alignment;
    // Only a pair of two different, printable diameters is aligned to.
    if (project_pair.size() != 2 ||
        !std::all_of(project_pair.begin(), project_pair.end(), valid_height) ||
        approximately_equal(project_pair[0], project_pair[1]))
        return alignment;

    alignment.aligned = true;
    alignment.nozzle_diameters = project_pair;
    alignment.min_layer_heights.reserve(project_pair.size());
    alignment.max_layer_heights.reserve(project_pair.size());
    for (std::size_t tool = 0; tool < project_pair.size(); ++tool) {
        const double diameter = project_pair[tool];
        // A preset variant naming one diameter for both tools is describing the other nozzle here.
        const bool config_describes_this_nozzle = tool < config_nozzle_diameters.size() &&
            approximately_equal(config_nozzle_diameters[tool], diameter);
        double minimum = tool < config_min_layer_heights.size() ? config_min_layer_heights[tool] : 0.;
        double maximum = tool < config_max_layer_heights.size() ? config_max_layer_heights[tool] : 0.;
        if (!config_describes_this_nozzle) {
            const auto known = std::find_if(known_limits.begin(), known_limits.end(),
                [diameter](const WizardNozzleLimitRow &row) {
                    return approximately_equal(row.diameter, diameter);
                });
            if (known != known_limits.end()) {
                minimum = known->min_height;
                maximum = known->max_height;
            } else {
                // Nothing installed states a limit for this nozzle; use the slicing rule for a zero
                // limit.
                minimum = kWizardDefaultMinLayerHeight;
                maximum = 0.75 * diameter;
            }
            alignment.assumed_extruders.push_back(tool);
        }
        alignment.min_layer_heights.push_back(minimum);
        alignment.max_layer_heights.push_back(maximum);
    }
    alignment.changed = alignment.nozzle_diameters != config_nozzle_diameters ||
                        alignment.min_layer_heights != config_min_layer_heights ||
                        alignment.max_layer_heights != config_max_layer_heights;
    return alignment;
}

std::string wizard_assumed_limits_line(const WizardPairAlignment &alignment)
{
    std::string text;
    for (std::size_t tool : alignment.assumed_extruders) {
        if (tool >= alignment.nozzle_diameters.size() ||
            tool >= alignment.min_layer_heights.size() ||
            tool >= alignment.max_layer_heights.size())
            continue;
        if (!text.empty())
            text += " ";
        text += "The selected printer preset has no limits for the " +
                height_text(alignment.nozzle_diameters[tool]) +
                " mm nozzle, so the wizard uses the machine profile for it: " +
                height_text(alignment.min_layer_heights[tool]) + " to " +
                height_text(alignment.max_layer_heights[tool]) + " mm.";
    }
    return text;
}

std::string wizard_printer_variant_for_pair(const std::vector<WizardPrinterVariantRow> &rows,
                                            const std::string &selected_name,
                                            const std::string &selected_model,
                                            const std::string &selected_variant,
                                            double fine_diameter)
{
    if (!valid_height(fine_diameter) || selected_model.empty())
        return {};
    const std::string wanted = height_text(fine_diameter);
    if (selected_variant == wanted)
        return {};
    // An installed variant first. Otherwise the shipped system variant, as stock sync selects
    // (and shows) the variant for a nozzle size the user never installed.
    std::string system_variant;
    for (const WizardPrinterVariantRow &row : rows) {
        if (row.name == selected_name || row.name.empty() || row.printer_model != selected_model ||
            row.printer_variant != wanted)
            continue;
        if (row.visible)
            return row.name;
        if (row.system && system_variant.empty())
            system_variant = row.name;
    }
    return system_variant;
}

std::string wizard_preset_switch_line(const std::string &kind, const std::string &before,
                                      const std::string &after)
{
    if (kind.empty() || before.empty() || after.empty() || before == after)
        return {};
    return kind + ": " + before + " to " + after;
}

bool wizard_catalogue_row_fits_pair(const WizardCatalogueRow &row,
                                    const std::vector<double> &project_pair)
{
    // A row with no declared pair is a plain height pair and carries nothing to contradict.
    if (!row.source_compatibility || row.source_compatibility->role_nozzle_diameters.size() != 2)
        return true;
    if (project_pair.size() != 2)
        return false;
    const std::vector<double> &roles = row.source_compatibility->role_nozzle_diameters;
    return (approximately_equal(roles[0], project_pair[0]) &&
            approximately_equal(roles[1], project_pair[1])) ||
           (approximately_equal(roles[0], project_pair[1]) &&
            approximately_equal(roles[1], project_pair[0]));
}

WizardFirstLayer wizard_first_layer(double effective_height, double effective_speed,
                                    double preset_height, double preset_speed)
{
    WizardFirstLayer first;
    if (!valid_height(effective_height))
        return first;
    first.resolved = true;
    first.height = effective_height;
    first.speed = effective_speed;
    first.preset_height = preset_height;
    first.preset_speed = preset_speed;
    const std::string current = height_text(effective_height) + " mm at " +
                                height_text(effective_speed) + " mm/s";
    first.differs = valid_height(preset_height) &&
        (!approximately_equal(preset_height, effective_height) ||
         !approximately_equal(preset_speed, effective_speed));
    if (!first.differs) {
        first.review_line = "First layer: " + current;
        return first;
    }
    const std::string preset = height_text(preset_height) + " mm at " +
                               height_text(preset_speed) + " mm/s";
    // Setup always applies the preset's first layer and says so.
    first.review_line = "First layer: " + preset + ", from the preset (was " + current + ")";
    return first;
}

std::vector<WizardProcessFix> wizard_process_fixes(const ConfigBase &copied_effective_config,
                                                   MixedNozzleSlicingMode mode)
{
    std::vector<WizardProcessFix> fixes;
    if (mode == MixedNozzleSlicingMode::Off)
        return fixes;
    const auto boolean = [&copied_effective_config](const char *key) {
        const auto *option = copied_effective_config.option<ConfigOptionBool>(key);
        return option != nullptr && option->value;
    };
    const auto switch_off = [&fixes, &boolean](const char *key, const char *name) {
        if (boolean(key))
            fixes.push_back({key, std::string(name) + ": on to off", true});
    };
    // Plain switches, turned off by Apply. The order is the order Print::validate checks them in.
    switch_off("prime_tower_flat_ironing", "Prime tower flat ironing");
    switch_off("enable_tower_interface_features", "Prime tower interface features");
    switch_off("ooze_prevention", "Ooze prevention");
    if (const auto *angle = copied_effective_config.option<ConfigOptionFloat>("wipe_tower_rotation_angle");
        angle != nullptr && !approximately_equal(angle->value, 0.))
        fixes.push_back({"wipe_tower_rotation_angle",
                         "Prime tower rotation: " + height_text(angle->value) + " to 0 degrees", true});
    if (const auto *filament = copied_effective_config.option<ConfigOptionInt>("wipe_tower_filament");
        filament != nullptr && filament->value != 0)
        fixes.push_back({"wipe_tower_filament",
                         "Prime tower filament: slot " + std::to_string(filament->value) +
                         " to the printing filaments", true});
    // A Bambu printer always runs the Type 1 tower; other printers run the one their profile names,
    // and Orca defaults that to Type 2. The mixed-nozzle rules are built on Type 1, so Apply
    // switches it. Only a named non-Bambu vendor profile counts; a config with no printer model is
    // left alone.
    const auto *printer_model = copied_effective_config.option<ConfigOptionString>("printer_model");
    const bool other_vendor_printer = printer_model != nullptr && !printer_model->value.empty() &&
                                      printer_model->value.rfind("Bambu Lab", 0) != 0;
    if (const auto *tower_type = copied_effective_config.option<ConfigOptionEnum<WipeTowerType>>("wipe_tower_type");
        other_vendor_printer && tower_type != nullptr && tower_type->value != WipeTowerType::Type1)
        fixes.push_back({"wipe_tower_type", "Prime tower type: Type 2 to Type 1, the tower mixed-nozzle slicing is built on", true});
    // Those printers usually keep Type 2 tower settings, which the Type 1 tower rules refuse. The
    // first two are printer settings, so Apply writes them there.
    if (other_vendor_printer) {
        switch_off("purge_in_prime_tower", "Purge into the prime tower");
        switch_off("enable_filament_ramming", "Configurable filament ramming");
        switch_off("single_extruder_multi_material_priming", "Priming all extruders at the start");
    }
    // Feature Split sizes its own infill combination from the cadence, and Body Split refuses
    // native combination, so both switches go off.
    switch_off("infill_combination", "Combine infill");
    // 0 and 100 percent both mean automatic; 100 percent is Orca's default, so only a real cap is
    // listed.
    if (const auto *cap = copied_effective_config.option<ConfigOptionFloatOrPercent>("infill_combination_max_layer_height");
        cap != nullptr && !approximately_equal(cap->value, 0.) && !(cap->percent && approximately_equal(cap->value, 100.)))
        fixes.push_back({"infill_combination_max_layer_height", "Infill combination height limit: to 0 (automatic)", true});
    // Body Split admission asks these of the process. The MN presets already have them; a
    // printer with no MN preset keeps its own process, so Apply sets them.
    if (mode == MixedNozzleSlicingMode::BodySplit) {
        if (const auto *pattern = copied_effective_config.option<ConfigOptionEnum<InfillPattern>>("sparse_infill_pattern");
            pattern != nullptr && !regional_band_admits_sparse_infill_pattern(pattern->value))
            fixes.push_back({"sparse_infill_pattern", "Sparse infill pattern: to Grid, which a coarse part can lay in bands", true});
        const auto nonzero = [&copied_effective_config](const char *key) {
            const auto *option = copied_effective_config.option<ConfigOptionFloat>(key);
            return option != nullptr && !approximately_equal(option->value, 0.);
        };
        if (nonzero("minimum_sparse_infill_area"))
            fixes.push_back({"minimum_sparse_infill_area", "Minimum sparse infill area: to 0 mm2", true});
        if (nonzero("top_shell_thickness"))
            fixes.push_back({"top_shell_thickness", "Top shell thickness: to 0, so the top shell layers count decides", true});
        if (nonzero("bottom_shell_thickness"))
            fixes.push_back({"bottom_shell_thickness", "Bottom shell thickness: to 0, so the bottom shell layers count decides", true});
        if (const auto *walls = copied_effective_config.option<ConfigOptionEnum<PerimeterGeneratorType>>("wall_generator");
            walls != nullptr && walls->value != PerimeterGeneratorType::Classic)
            fixes.push_back({"wall_generator", "Wall generator: to Classic", true});
        // The engine refuses it only when it acts on at least one layer.
        const auto *foot_layers = copied_effective_config.option<ConfigOptionInt>("elefant_foot_compensation_layers");
        if (nonzero("elefant_foot_compensation") && (foot_layers == nullptr || foot_layers->value > 0))
            fixes.push_back({"elefant_foot_compensation", "Elephant foot compensation: to 0", true});
        for (const char *key : {"flush_into_infill", "flush_into_objects", "flush_into_support"})
            if (boolean(key))
                fixes.push_back({key, std::string("Flushing into the part: ") + key + " on to off", true});
    }
    // The rest are the user's choices, so they are named and left alone.
    if (const auto *relative = copied_effective_config.option<ConfigOptionBool>("use_relative_e_distances");
        relative != nullptr && !relative->value)
        fixes.push_back({"use_relative_e_distances",
                         "Relative E distances are off. A prime tower needs them, and they live in "
                         "the printer settings.", false});
    if (boolean("spiral_mode"))
        fixes.push_back({"spiral_mode",
                         "Spiral vase mode cannot be used with mixed-nozzle slicing. Turn it off "
                         "before slicing.", false});
    if (const auto *pattern = copied_effective_config.option<ConfigOptionEnum<InfillPattern>>("sparse_infill_pattern");
        pattern != nullptr && mode == MixedNozzleSlicingMode::FeatureSplit &&
        !feature_combine_admits_sparse_infill_pattern(pattern->value))
        fixes.push_back({"sparse_infill_pattern",
                         "This sparse infill pattern cannot be split into fine and coarse layers. Choose "
                         "another pattern before slicing.", false});
    return fixes;
}

std::string wizard_intro_lead_line()
{
    return "Mixed-nozzle slicing uses both nozzles in one print, each at its own layer height. Choose how "
           "the print is split between them.";
}

std::string wizard_fine_layer_help(MixedNozzleSlicingMode mode)
{
    return std::string("Thinner fine layers give smoother walls and top surfaces, sharper small details and "
                       "text, and cleaner overhangs and shallow slopes. They add time to what the fine nozzle "
                       "prints, but the coarse layer can stay about as thick with a larger N, so ") +
           (mode == MixedNozzleSlicingMode::BodySplit ? "the coarse parts print" : "the inside prints") +
           " about as fast.";
}

WizardIntroOutcome wizard_intro_outcome(bool intro_line_shown, bool applied)
{
    WizardIntroOutcome outcome;
    if (!intro_line_shown)
        return outcome;
    outcome.persist_seen = applied;
    outcome.suppress_this_session = !applied;
    return outcome;
}

bool mixed_nozzle_should_show_intro(bool intro_seen, MixedNozzleSlicingMode effective_mode,
                                    bool assignments_present)
{
    if (effective_mode != MixedNozzleSlicingMode::Off && assignments_present)
        return false;
    return !intro_seen;
}

namespace {

void replace_all(std::string &text, const std::string &from, const std::string &to)
{
    if (from.empty())
        return;
    for (std::size_t at = text.find(from); at != std::string::npos; at = text.find(from, at + to.size()))
        text.replace(at, from.size(), to);
}

std::string joined_names(const std::vector<std::string> &names)
{
    std::string text;
    for (std::size_t i = 0; i < names.size(); ++i) {
        if (i > 0)
            text += i + 1 == names.size() ? " and " : ", ";
        text += names[i];
    }
    return text;
}

constexpr const char *kBodyFamilyPrefix = "MN Body ";
constexpr const char *kFeatureFamilyPrefix = "MN Feature ";

WizardProcessFamily family_of(const std::string &name)
{
    if (name.rfind(kBodyFamilyPrefix, 0) == 0)
        return WizardProcessFamily::BodySplit;
    if (name.rfind(kFeatureFamilyPrefix, 0) == 0)
        return WizardProcessFamily::FeatureSplit;
    return WizardProcessFamily::Other;
}

std::vector<double> pair_of(const std::string &name)
{
    for (const std::string prefix : {std::string(kBodyFamilyPrefix), std::string(kFeatureFamilyPrefix)}) {
        if (name.rfind(prefix, 0) != 0)
            continue;
        // "0.2-0.6 ..." read with a dot whatever the app language. sscanf follows the C locale, so
        // a comma-decimal language read no pair and setup found no MN preset for the nozzles.
        const std::string_view rest = std::string_view(name).substr(prefix.size());
        size_t first_end = 0;
        const double first = string_to_double_decimal_point(rest, &first_end);
        if (first_end == 0 || first_end >= rest.size() || rest[first_end] != '-')
            continue;
        size_t second_end = 0;
        const double second = string_to_double_decimal_point(rest.substr(first_end + 1), &second_end);
        if (second_end == 0)
            continue;
        if (valid_height(first) && valid_height(second) && !approximately_equal(first, second))
            return {std::min(first, second), std::max(first, second)};
    }
    return {};
}

std::vector<double> sorted_pair(const std::vector<double> &pair)
{
    if (pair.size() != 2 || !valid_height(pair[0]) || !valid_height(pair[1]) ||
        approximately_equal(pair[0], pair[1]))
        return {};
    return {std::min(pair[0], pair[1]), std::max(pair[0], pair[1])};
}

bool same_nozzle_pair(const std::vector<double> &lhs, const std::vector<double> &rhs)
{
    return lhs.size() == 2 && rhs.size() == 2 && approximately_equal(lhs[0], rhs[0]) &&
           approximately_equal(lhs[1], rhs[1]);
}

std::string nozzle_pair_text(const std::vector<double> &project_pair)
{
    const std::vector<double> pair = sorted_pair(project_pair);
    if (pair.empty())
        return "nozzle pair";
    return height_text(pair[0]) + "/" + height_text(pair[1]) + " mm nozzle pair";
}

} // namespace

std::string wizard_admission_refusal_line(const std::string &engine_message, const std::string &opt_key,
                                          const std::string &object_name)
{
    // Engine codes in brackets are not shown; the caller logs the full message.
    std::string sentence = split_engine_message(engine_message).text;
    // The engine's vocabulary, in the words the wizard uses. Longest phrases first.
    for (const auto &[from, to] : std::vector<std::pair<std::string, std::string>>{
             {"Synchronized multi-nozzle layering", "Body Split"},
             {"synchronized multi-nozzle layering", "Body Split"},
             {"synchronized multi-nozzle", "Body Split"},
             {"synchronized regional layering", "Body Split"},
             {"native regional grids", "Body Split"},
             {"Synchronized ", ""},
             {"synchronized ", ""},
             {"MODEL_PART", "model part"},
             {"static physical-tool route", "nozzle assignment"},
             {"physical tools", "nozzles"},
             {"physical tool", "nozzle"}})
        replace_all(sentence, from, to);
    if (!sentence.empty() && sentence.back() != '.')
        sentence += ".";

    std::string setting;
    if (!opt_key.empty() && opt_key != "mixed_nozzle_slicing_mode")
        if (const ConfigOptionDef *definition = print_config_def.get(opt_key);
            definition != nullptr && !definition->label.empty())
            setting = definition->label;

    std::string line = object_name.empty() ? std::string() : object_name + ": ";
    line += "Body Split cannot slice this as set up. " + sentence;
    // The setting by its name only; the key is the Review link target.
    if (!setting.empty())
        line += " Setting to change: " + setting + ".";
    return line;
}

std::string wizard_single_part_hint(const std::vector<std::string> &object_names)
{
    if (object_names.empty())
        return {};
    const std::string names = joined_names(object_names);
    return names + (object_names.size() == 1 ? " is one part. " : " are single parts. ") +
        "To print some of it on the fine nozzle, paint those regions with the Color Painting tool, "
        "or use Split to parts, then open setup again.";
}

WizardProcessFamily wizard_process_family(const std::string &name, const std::string &inherits)
{
    const WizardProcessFamily own = family_of(name);
    return own != WizardProcessFamily::Other ? own : family_of(inherits);
}

// The nozzle pair a family preset's name states, smaller first. Empty when neither name states one.
static std::vector<double> wizard_process_pair(const std::string &name, const std::string &inherits)
{
    std::vector<double> pair = pair_of(name);
    return pair.empty() ? pair_of(inherits) : pair;
}

namespace {

struct FamilyBasis {
    const WizardProcessPresetRow *pick {nullptr};
    bool current_in_family {false};
    bool none_for_pair {false};
};

// The process pick rule for one family. The coarse height never takes part.
FamilyBasis family_basis(const std::vector<WizardProcessPresetRow> &rows, WizardProcessFamily family,
                         const std::string &current_name, const std::string &current_inherits,
                         const std::vector<double> &wanted, double fine_height)
{
    FamilyBasis basis;
    if (wizard_process_family(current_name, current_inherits) == family &&
        same_nozzle_pair(wizard_process_pair(current_name, current_inherits), wanted)) {
        basis.current_in_family = true;
        return basis;
    }
    std::vector<const WizardProcessPresetRow *> candidates;
    for (const WizardProcessPresetRow &row : rows)
        if (row.selectable && wizard_process_family(row.name, row.inherits) == family &&
            same_nozzle_pair(wizard_process_pair(row.name, row.inherits), wanted))
            candidates.push_back(&row);
    std::sort(candidates.begin(), candidates.end(),
              [](const WizardProcessPresetRow *lhs, const WizardProcessPresetRow *rhs) { return lhs->name < rhs->name; });
    if (candidates.empty()) {
        basis.none_for_pair = true;
        return basis;
    }
    const auto standard = [](const WizardProcessPresetRow &row) {
        return row.name.find(" Standard ") != std::string::npos;
    };
    const auto first = [&candidates](const auto &predicate) -> const WizardProcessPresetRow * {
        for (const WizardProcessPresetRow *row : candidates)
            if (predicate(*row))
                return row;
        return nullptr;
    };
    basis.pick = first([&](const WizardProcessPresetRow &row) {
        return approximately_equal(row.fine_height, fine_height) && standard(row);
    });
    if (basis.pick == nullptr)
        basis.pick = first([&](const WizardProcessPresetRow &row) {
            return approximately_equal(row.fine_height, fine_height);
        });
    if (basis.pick == nullptr)
        basis.pick = first(standard);
    if (basis.pick == nullptr)
        basis.pick = candidates.front();
    return basis;
}

} // namespace

WizardBodyProcessChoice wizard_body_process_choice(const std::vector<WizardProcessPresetRow> &rows,
                                                   const std::string &current_name,
                                                   const std::string &current_inherits,
                                                   const std::vector<double> &project_pair,
                                                   double fine_height,
                                                   std::optional<double> coarse_height)
{
    WizardBodyProcessChoice choice;
    const WizardProcessFamily current_family = wizard_process_family(current_name, current_inherits);
    choice.current_is_feature = current_family == WizardProcessFamily::FeatureSplit;
    const std::vector<double> wanted = sorted_pair(project_pair);
    if (wanted.empty())
        return choice;
    // A Body preset for this pair, stock or the user's own, is kept; the review overrides its
    // heights.
    const FamilyBasis basis = family_basis(rows, WizardProcessFamily::BodySplit, current_name,
                                           current_inherits, wanted, fine_height);
    if (basis.current_in_family) {
        choice.current_is_body = true;
        return choice;
    }
    if (basis.none_for_pair) {
        choice.no_body_preset_for_pair = true;
        return choice;
    }
    choice.exact_heights = approximately_equal(basis.pick->fine_height, fine_height) && coarse_height &&
                           approximately_equal(basis.pick->coarse_height, *coarse_height);
    if (basis.pick->name != current_name)
        choice.preset = basis.pick->name;
    return choice;
}

WizardProcessBasis wizard_process_basis(const std::vector<WizardProcessPresetRow> &rows,
                                        MixedNozzleSlicingMode mode,
                                        const std::string &current_name,
                                        const std::string &current_inherits,
                                        const std::vector<double> &project_pair,
                                        double fine_height,
                                        bool keep_current)
{
    WizardProcessBasis basis;
    if (keep_current) {
        basis.kept_by_choice = true;
        return basis;
    }
    const std::vector<double> wanted = sorted_pair(project_pair);
    if (mode == MixedNozzleSlicingMode::Off || wanted.empty()) {
        basis.no_preset_for_pair = true;
        return basis;
    }
    const WizardProcessFamily own = mode == MixedNozzleSlicingMode::BodySplit ? WizardProcessFamily::BodySplit
                                                                              : WizardProcessFamily::FeatureSplit;
    FamilyBasis chosen = family_basis(rows, own, current_name, current_inherits, wanted, fine_height);
    // The MN Body presets also serve Feature Split, used only when the pair has no MN Feature
    // preset.
    if (chosen.none_for_pair && own == WizardProcessFamily::FeatureSplit) {
        const FamilyBasis body = family_basis(rows, WizardProcessFamily::BodySplit, current_name,
                                              current_inherits, wanted, fine_height);
        if (!body.none_for_pair)
            chosen = body;
    }
    if (chosen.current_in_family) {
        basis.keeps_current = true;
        return basis;
    }
    if (chosen.none_for_pair) {
        basis.no_preset_for_pair = true;
        return basis;
    }
    if (chosen.pick->name == current_name)
        basis.keeps_current = true;
    else
        basis.preset = chosen.pick->name;
    return basis;
}

std::string wizard_process_basis_line(const WizardProcessBasis &basis, const std::string &current_name,
                                      const std::vector<double> &project_pair)
{
    if (!basis.preset.empty()) {
        std::string line = "Process preset: " + basis.preset + ", made for this nozzle pair.";
        if (!current_name.empty())
            line += " It replaces " + current_name + ".";
        return line + " All times below use it.";
    }
    const std::string current = current_name.empty() ? std::string("the current one") : current_name;
    if (basis.kept_by_choice)
        return "Process preset: your current " + current + ", kept as you chose. Setup changes only the "
               "layer heights. All times below use it.";
    if (basis.keeps_current)
        return "Process preset: your current " + current + ", made for this nozzle pair. All times below use it.";
    const std::vector<double> pair = sorted_pair(project_pair);
    const std::string nozzles = pair.empty() ? std::string("these nozzles")
                                             : "the " + height_text(pair[0]) + " and " + height_text(pair[1]) + " mm nozzles";
    return "Process preset: your current " + current + ". There is no mixed-nozzle preset for " + nozzles +
           " yet, so setup keeps yours and changes only the layer heights. All times below use it.";
}

std::vector<std::string> wizard_process_basis_lines(const WizardProcessBasis &basis,
                                                    const std::string &current_name,
                                                    const std::vector<double> &project_pair,
                                                    double preset_fine_height,
                                                    std::optional<double> preset_coarse_height,
                                                    double fine_height,
                                                    std::optional<double> coarse_height,
                                                    bool can_switch)
{
    std::vector<std::string> lines;
    if (basis.preset.empty())
        return lines;
    if (!can_switch) {
        lines.push_back("Setup would switch the process preset to " + basis.preset + ", which every plate "
                        "shares. Tick the box on this page to allow it.");
        return lines;
    }
    lines.push_back("Process preset: " + current_name + " to " + basis.preset + ", made for the " +
                    nozzle_pair_text(project_pair) + ".");
    const bool fine_differs = valid_height(fine_height) && !approximately_equal(preset_fine_height, fine_height);
    const bool coarse_differs = coarse_height && valid_height(*coarse_height) &&
                                (!preset_coarse_height || !approximately_equal(*preset_coarse_height, *coarse_height));
    if (fine_differs || coarse_differs)
        lines.push_back("Your layer heights are set on top: fine " + height_text(fine_height) + " mm" +
                        (coarse_height && valid_height(*coarse_height)
                             ? " and coarse " + height_text(*coarse_height) + " mm." : std::string(".")));
    return lines;
}

std::vector<std::string> wizard_body_process_lines(const WizardBodyProcessChoice &choice,
                                                   const std::string &current_name,
                                                   const std::vector<double> &project_pair,
                                                   double fine_height,
                                                   std::optional<double> coarse_height,
                                                   bool can_switch)
{
    std::vector<std::string> lines;
    const std::string pair = nozzle_pair_text(project_pair);
    if (!choice.preset.empty()) {
        if (!can_switch) {
            lines.push_back(current_name + (choice.current_is_feature
                ? " is a Feature Split process preset. " : " is not a Body Split process preset. ") +
                "Switching it to " + choice.preset + " changes the process every plate shares, so tick "
                "the box on this page to allow it.");
            return lines;
        }
        if (choice.current_is_feature)
            lines.push_back("This project is set to Body Split but uses " + current_name +
                ", a Feature Split process preset. Apply switches it to " + choice.preset +
                ", the Body Split preset for the " + pair + ".");
        else
            lines.push_back("Process preset: " + current_name + " to " + choice.preset +
                ", the Body Split preset for the " + pair + ".");
        if (!choice.exact_heights && valid_height(fine_height))
            lines.push_back("Your heights, fine " + height_text(fine_height) + " mm" +
                (coarse_height && valid_height(*coarse_height)
                    ? " and coarse " + height_text(*coarse_height) + " mm" : std::string()) +
                ", are kept on top of " + choice.preset + ".");
        return lines;
    }
    if (choice.no_body_preset_for_pair)
        lines.push_back("There is no Body Split process preset for the " + pair + " yet, so the process "
            "stays on " + current_name + (choice.current_is_feature ? ", a Feature Split preset," : "") +
            " with the Body Split heights applied on top.");
    return lines;
}

std::string wizard_body_process_guard_text(const WizardBodyProcessChoice &choice,
                                           const std::string &current_name,
                                           const std::vector<double> &project_pair)
{
    if (!choice.current_is_feature)
        return {};
    std::string line = "This Body Split project uses " + current_name + ", a Feature Split process preset.";
    if (!choice.preset.empty())
        line += " Open Set up... to switch to " + choice.preset + ", the Body Split preset for the " +
                nozzle_pair_text(project_pair) + ".";
    else if (choice.no_body_preset_for_pair)
        line += " There is no Body Split preset for the " + nozzle_pair_text(project_pair) + " yet.";
    return line;
}

std::string wizard_review_reason_text(const std::string &raw_reason)
{
    std::string reason = raw_reason;
    if (reason.rfind("Candidate:", 0) == 0)
        reason.erase(0, 10);
    if (reason == "SharedProcessChangeOutsideScope")
        return "The new layer heights would change the other plates that share this process. Tick the box "
               "above, or set this up for the whole project.";
    if (reason == "SharedProcessAffectedPlatesMissing")
        return "Setup could not tell which plates share this process. Open setup again.";
    if (reason == "SingleNozzleConversionReviewRequired")
        return "Printing this project with one nozzle needs its own material check first.";
    if (reason == "SingleNozzleMaterialCompatibilityUnresolved" || reason == "SingleNozzleMaterialMismatch")
        return "These two materials cannot be merged onto one nozzle.";
    if (reason == "UnresolvedLogicalFilamentBinding" || reason == "ResolvedPhysicalRoleMismatch")
        return "Check which nozzle each chosen material prints on, in Materials.";
    if (reason == "BodySharedBaseHeightBelowEnvelope")
        return "The first layers of the parts are outside what one of the nozzles can print. Pick another "
               "fine layer.";
    if (reason == "CadenceEnumerationUnavailable")
        return "This fine layer gives too many coarse layer choices. Pick a more usual fine layer.";
    if (reason == "SourceCompatibilityMismatch" || reason == "SourceCompatibilityUnresolved")
        return "This choice comes from a preset made for another nozzle pair.";
    if (reason == "CadenceRatioInvalid")
        return "The coarse layer is not a whole number of fine layers.";
    if (reason == "FineHeightMissing")
        return "Pick a fine layer first.";
    if (reason.rfind("LockedKeyChange:", 0) == 0)
        return "A change here would touch a setting you locked.";
    // The wizard writes the plate map itself, so if the engine's static-map code still reaches a
    // row, the materials are what to look at.
    if (reason == "MNS-MAP-A01")
        return "Choose the fine and coarse materials. Setup writes the plate's filament map itself.";
    for (int code = 0; code <= int(MixedNozzleDiagnosticCode::FilamentBindingUnqualified); ++code) {
        const MixedNozzleDiagnostic diagnostic{MixedNozzleDiagnosticCode(code), 0, {}};
        if (reason == diagnostic.stable_code())
            return diagnostic.message();
    }
    return "Some of these choices need another look before setup can apply them.";
}

std::string wizard_setup_diagnostic_message(const std::string &diagnostic)
{
    static const std::string internal = "Mixed-Nozzle wizard ";
    if (diagnostic.rfind(internal, 0) != 0)
        return diagnostic;
    // These mean the project moved under the open wizard.
    for (const char *word : {"stale", "changed", "outside", "unavailable", "no longer", "snapshot",
                             "incomplete owner", "owners are", "invalid or duplicated"})
        if (diagnostic.find(word) != std::string::npos)
            return "Setup could not finish because the project changed while it was open. Open setup again.";
    return "Setup could not finish with these choices. Go back over the materials and layer heights, or "
           "open setup again.";
}

std::string wizard_whole_project_label(std::size_t plate_count)
{
    return "The whole project (" + std::to_string(plate_count) + (plate_count == 1 ? " plate)" : " plates)");
}

std::string wizard_material_settings_note(std::size_t plate_count)
{
    return std::string("Turning this off can leave a material with the other nozzle's speed limit. ") +
           (plate_count > 1 ? "Material settings are shared by all " + std::to_string(plate_count) +
                                  " plates in this project."
                            : std::string("Material settings are shared by the whole project."));
}

std::string wizard_shared_consent_label(std::size_t plate_count)
{
    if (plate_count <= 1)
        return "Change the layer heights in the process this plate uses.";
    const std::size_t others = plate_count - 1;
    if (others == 1)
        return "Also change the layer heights on the other plate. They share one process.";
    return "Also change the layer heights on the other " + std::to_string(others) +
           " plates. They share one process.";
}

// The fine and coarse physical nozzles the material defaults start from. Two nozzles: the smaller is
// fine, the left on a tie. Two of N: the smallest and the largest nozzle, the lowest toolhead
// number on a tie.
static std::pair<std::size_t, std::size_t> wizard_default_fine_coarse_physical(const std::vector<double> &nozzle_diameters)
{
    const std::size_t fine = nozzle_diameters.size() == 2 && nozzle_diameters[1] < nozzle_diameters[0] ? 1 : 0;
    if (nozzle_diameters.size() > 2)
        if (const auto tools = mixed_nozzle_default_tool_pair(nozzle_diameters))
            return *tools;
    return {fine, fine == 0 ? std::size_t(1) : std::size_t(0)};
}

WizardMaterialDefaults wizard_default_materials(const std::vector<double> &nozzle_diameters,
                                                const std::vector<int> &filament_map,
                                                const std::vector<std::string> &filament_types,
                                                const std::vector<std::size_t> &object_slots)
{
    WizardMaterialDefaults defaults;
    const std::size_t count = std::max(filament_map.size(), filament_types.size());
    if (count == 0)
        return defaults;
    std::size_t fine_physical, coarse_physical;
    std::tie(fine_physical, coarse_physical) = wizard_default_fine_coarse_physical(nozzle_diameters);
    const auto on = [&filament_map](std::size_t slot, std::size_t physical) {
        return slot < filament_map.size() && filament_map[slot] == int(physical) + 1;
    };
    const auto type_of = [&filament_types](std::size_t slot) {
        return slot < filament_types.size() ? filament_types[slot] : std::string();
    };
    for (const std::size_t slot : object_slots)
        if (slot < count && on(slot, fine_physical)) {
            defaults.fine = slot;
            break;
        }
    for (std::size_t slot = 0; !defaults.fine && slot < count; ++slot)
        if (on(slot, fine_physical))
            defaults.fine = slot;
    for (std::size_t index = 0; !defaults.fine && index < object_slots.size(); ++index)
        if (object_slots[index] < count)
            defaults.fine = object_slots[index];
    if (!defaults.fine)
        defaults.fine = 0;
    const std::size_t fine = *defaults.fine;
    const std::string fine_type = type_of(fine);
    for (std::size_t slot = 0; !defaults.coarse && slot < count; ++slot)
        if (slot != fine && on(slot, coarse_physical) && !fine_type.empty() && type_of(slot) == fine_type)
            defaults.coarse = slot;
    for (std::size_t slot = 0; !defaults.coarse && slot < count; ++slot)
        if (slot != fine && on(slot, coarse_physical))
            defaults.coarse = slot;
    if (!defaults.coarse) {
        for (std::size_t slot = 0; !defaults.coarse && slot < count; ++slot)
            if (slot != fine)
                defaults.coarse = slot;
        if (defaults.coarse) {
            defaults.empty_nozzle = nozzle_diameters.size() == 2
                ? "the " + height_text(nozzle_diameters[coarse_physical]) + " mm nozzle" : std::string("the coarse nozzle");
            defaults.note = wizard_empty_nozzle_note(defaults.empty_nozzle, *defaults.coarse);
        }
    }
    return defaults;
}

std::string wizard_empty_nozzle_note(const std::string &empty_nozzle, std::size_t coarse_slot)
{
    return "No material sits on " + empty_nozzle + " yet, so setup puts slot " + std::to_string(coarse_slot + 1) +
           " there. Pick another if you like.";
}

std::vector<WizardBodyRole> wizard_default_body_roles(const std::vector<WizardBodyRoleRow> &rows,
                                                      std::size_t fine_physical)
{
    std::vector<WizardBodyRole> roles;
    roles.reserve(rows.size());
    bool painted = false;
    for (const WizardBodyRoleRow &row : rows) {
        roles.push_back(row.current_physical && *row.current_physical == fine_physical ? WizardBodyRole::Fine
                                                                                       : WizardBodyRole::Coarse);
        painted = painted || row.painted;
    }
    const bool one_nozzle = rows.size() >= 2 && !painted &&
        std::all_of(roles.begin(), roles.end(), [&roles](WizardBodyRole role) { return role == roles.front(); });
    if (one_nozzle) {
        std::size_t smallest = 0;
        for (std::size_t index = 1; index < rows.size(); ++index)
            if (rows[index].volume < rows[smallest].volume)
                smallest = index;
        for (std::size_t index = 0; index < rows.size(); ++index)
            roles[index] = index == smallest ? WizardBodyRole::Fine : WizardBodyRole::Coarse;
    }
    return roles;
}

std::vector<WizardBodyAssignment> wizard_body_roles_to_slots(
    const std::vector<WizardBodyRoleRow> &rows, const std::vector<WizardBodyRole> &roles,
    std::size_t fine_slot, std::size_t coarse_slot, std::size_t fine_physical, std::size_t coarse_physical,
    const std::vector<std::optional<WizardBodySlotOverride>> &overrides)
{
    // The slot most of a role's parts print with now, the lower on a tie. Step 2's material replaces
    // it; a part on any other slot has a third material of its own.
    const auto role_slot = [&rows, &roles](WizardBodyRole role) {
        std::map<int, std::size_t> counts;
        for (std::size_t index = 0; index < rows.size() && index < roles.size(); ++index)
            if (roles[index] == role && rows[index].current_slot >= 0)
                ++counts[rows[index].current_slot];
        int best = -1;
        for (const auto &[slot, count] : counts)
            if (best < 0 || count > counts[best])
                best = slot;
        return best;
    };
    const int fine_role_slot = role_slot(WizardBodyRole::Fine);
    const int coarse_role_slot = role_slot(WizardBodyRole::Coarse);
    std::vector<WizardBodyAssignment> body;
    for (std::size_t index = 0; index < rows.size() && index < roles.size(); ++index) {
        const WizardBodyRoleRow &row = rows[index];
        if (index < overrides.size() && overrides[index]) {
            body.push_back({row.object_id, row.volume_id, overrides[index]->slot,
                            overrides[index]->physical == coarse_physical});
            continue;
        }
        const bool coarse = roles[index] == WizardBodyRole::Coarse;
        const std::size_t physical = coarse ? coarse_physical : fine_physical;
        // A third material stays when it already prints on this nozzle. A part on its role's
        // common slot (a new import's slot 1, say) takes the material picked in step 2.
        const bool keeps_own = row.current_slot >= 0 && row.current_physical && *row.current_physical == physical &&
            row.current_slot != (coarse ? coarse_role_slot : fine_role_slot);
        body.push_back({row.object_id, row.volume_id,
                        keeps_own ? std::size_t(row.current_slot) : (coarse ? coarse_slot : fine_slot), coarse});
    }
    return body;
}

int wizard_body_slot_choice(int part_slot, std::optional<std::size_t> fine_slot,
                            std::optional<std::size_t> coarse_slot, std::size_t slot_count)
{
    if (part_slot < 1 || std::size_t(part_slot) > slot_count)
        return 0;
    const std::size_t slot = std::size_t(part_slot - 1);
    return (fine_slot && *fine_slot == slot) || (coarse_slot && *coarse_slot == slot) ? 0 : part_slot;
}

static std::string wizard_part_size_text(double x, double y, double z)
{
    // Whole millimetres from 10 mm up, one decimal below, so a 3.2 mm plate does not read 3.
    const auto dimension = [](double value) {
        if (!(value > 0.))
            return std::string("0");
        std::ostringstream stream;
        if (value >= 10.)
            stream << std::lround(value);
        else {
            stream << std::fixed << std::setprecision(1) << value;
            std::string text = stream.str();
            if (text.size() > 2 && text.compare(text.size() - 2, 2, ".0") == 0)
                text.erase(text.size() - 2);
            return text;
        }
        return stream.str();
    };
    return dimension(x) + " x " + dimension(y) + " x " + dimension(z) + " mm";
}

std::string wizard_part_row_text(const std::string &material_label, const std::string &object_name,
                                 const std::string &part_name, bool several_objects,
                                 double size_x, double size_y, double size_z)
{
    std::string text;
    if (!material_label.empty())
        text = material_label + "   ";
    text += several_objects && !object_name.empty() ? object_name + " / " + part_name : part_name;
    if (size_x > 0. || size_y > 0. || size_z > 0.)
        text += "   " + wizard_part_size_text(size_x, size_y, size_z);
    return text;
}

std::string wizard_one_nozzle_parts_line(const std::vector<WizardPartOnNozzle> &parts,
                                         const std::vector<double> &nozzle_diameters)
{
    if (parts.size() < 2 || nozzle_diameters.size() != 2)
        return {};
    const std::size_t physical = parts.front().physical;
    if (physical > 1 || std::any_of(parts.begin(), parts.end(),
                                    [physical](const WizardPartOnNozzle &part) { return part.physical != physical; }))
        return {};
    const auto side = [](std::size_t tool) { return std::string(tool == 0 ? "left" : "right"); };
    std::string names;
    for (std::size_t index = 0; index < parts.size(); ++index) {
        if (index > 0)
            names += index + 1 == parts.size() ? " and " : ", ";
        names += parts[index].name + " (slot " + std::to_string(parts[index].slot + 1) + ")";
    }
    std::string line = names + (parts.size() == 2 ? " both" : " all") + " print on the " + side(physical) + " " +
                       height_text(nozzle_diameters[physical]) + " mm nozzle.";
    // The part to move: the finest of its own, else the last one.
    std::size_t move = parts.size() - 1;
    for (std::size_t index = 0; index < parts.size(); ++index)
        if (parts[index].layer_height > 0. &&
            (parts[move].layer_height <= 0. || parts[index].layer_height < parts[move].layer_height))
            move = index;
    const std::size_t other = physical == 0 ? 1 : 0;
    const bool shared = std::any_of(parts.begin(), parts.end(), [&parts, move](const WizardPartOnNozzle &part) {
        return &part != &parts[move] && part.slot == parts[move].slot;
    });
    if (shared)
        line += " Give " + parts[move].name + " a slot on the " + side(other) + " nozzle";
    else
        line += " Put slot " + std::to_string(parts[move].slot + 1) + " on the " + side(other) +
                " nozzle in Filament grouping";
    return line + ", or run Change... to set it up.";
}

static WizardRoleSlots wizard_body_role_slots(const std::vector<WizardBodyRoleRow> &rows,
                                              const std::vector<WizardBodyRole> &roles)
{
    const auto most_common = [&rows, &roles](WizardBodyRole role) -> std::optional<std::size_t> {
        std::map<std::size_t, std::size_t> counts;
        for (std::size_t index = 0; index < rows.size() && index < roles.size(); ++index)
            if (roles[index] == role && rows[index].current_slot >= 0)
                ++counts[std::size_t(rows[index].current_slot)];
        std::optional<std::size_t> best;
        for (const auto &[slot, count] : counts)
            if (!best || count > counts[*best])
                best = slot;
        return best;
    };
    WizardRoleSlots slots;
    slots.fine = most_common(WizardBodyRole::Fine);
    slots.coarse = most_common(WizardBodyRole::Coarse);
    if (slots.fine && slots.coarse && *slots.fine == *slots.coarse)
        slots.coarse.reset();
    return slots;
}

WizardPickerDefaults wizard_body_picker_defaults(const std::vector<WizardBodyRoleRow> &rows,
                                                 const std::vector<WizardBodyRole> &roles,
                                                 const std::vector<double> &nozzle_diameters,
                                                 const std::vector<int> &filament_map,
                                                 const std::vector<std::string> &filament_types,
                                                 const std::vector<std::string> &filament_labels,
                                                 std::optional<std::size_t> fine_pick,
                                                 std::optional<std::size_t> coarse_pick)
{
    const WizardRoleSlots slots = wizard_body_role_slots(rows, roles);
    WizardPickerDefaults defaults;
    defaults.fine = slots.fine ? slots.fine : fine_pick;
    defaults.coarse = slots.coarse ? slots.coarse : coarse_pick;
    // wizard_body_role_slots() leaves the coarse slot empty only when it would be the fine one.
    bool coarse_part_on_slot = false;
    for (std::size_t index = 0; index < rows.size() && index < roles.size(); ++index)
        coarse_part_on_slot = coarse_part_on_slot ||
            (roles[index] == WizardBodyRole::Coarse && rows[index].current_slot >= 0);
    if (!slots.fine || slots.coarse || !coarse_part_on_slot) {
        // A kept picker value never lands on the slot the other role's parts use.
        if (defaults.fine && defaults.coarse && *defaults.fine == *defaults.coarse) {
            if (!slots.fine)
                defaults.fine.reset();
            else if (!slots.coarse)
                defaults.coarse.reset();
        }
        return defaults;
    }

    const std::size_t shared = *slots.fine;
    std::size_t fine_physical, coarse_physical;
    std::tie(fine_physical, coarse_physical) = wizard_default_fine_coarse_physical(nozzle_diameters);
    const auto on = [&filament_map](std::size_t slot, std::size_t physical) {
        return slot < filament_map.size() && filament_map[slot] == int(physical) + 1;
    };
    const auto label = [&filament_labels](std::size_t slot) {
        return slot < filament_labels.size() ? filament_labels[slot] : "slot " + std::to_string(slot + 1);
    };
    const auto nozzle = [&nozzle_diameters, coarse_physical](std::size_t physical) {
        return nozzle_diameters.size() == 2 ? "the " + height_text(nozzle_diameters[physical]) + " mm nozzle"
                                            : std::string(physical == coarse_physical ? "the coarse nozzle" : "the fine nozzle");
    };
    const std::string type = shared < filament_types.size() ? filament_types[shared] : std::string();
    // The closest other slot of the shared slot's material type, one already on `physical` first,
    // the lower slot on a tie.
    const auto closest = [&](std::size_t physical) -> std::optional<std::size_t> {
        if (type.empty())
            return std::nullopt;
        for (const bool on_nozzle : {true, false}) {
            std::optional<std::size_t> best;
            for (std::size_t slot = 0; slot < filament_types.size(); ++slot) {
                if (slot == shared || filament_types[slot] != type || (on_nozzle && !on(slot, physical)))
                    continue;
                const auto distance = [shared](std::size_t other) {
                    return other > shared ? other - shared : shared - other;
                };
                if (!best || distance(slot) < distance(*best))
                    best = slot;
            }
            if (best)
                return best;
        }
        return std::nullopt;
    };

    // The shared slot stays with the role whose nozzle it is on; fine when the map says neither.
    const bool shared_is_coarse = on(shared, coarse_physical);
    const std::size_t keeps_physical = shared_is_coarse ? coarse_physical : fine_physical;
    const std::size_t other_physical = shared_is_coarse ? fine_physical : coarse_physical;
    const std::string keeps_role = shared_is_coarse ? "coarse" : "fine";
    const std::string other_role = shared_is_coarse ? "fine" : "coarse";
    const std::optional<std::size_t> other = closest(other_physical);
    defaults.fine = shared_is_coarse ? other : std::optional<std::size_t>(shared);
    defaults.coarse = shared_is_coarse ? std::optional<std::size_t>(shared) : other;
    const std::string opening = "The fine and coarse parts share " + label(shared);
    const std::string material = type.empty() ? std::string("same material") : type;
    if (other)
        defaults.note = opening + ". The " + keeps_role + " parts keep it on " + nozzle(keeps_physical) +
                        ", and the " + other_role + " parts print with " + label(*other) + ", the closest " +
                        material + " slot on " + nozzle(other_physical) + ".";
    else
        defaults.note = opening + ", and no other " + material + " slot is loaded. Pick the material for the " +
                        other_role + " layers.";
    return defaults;
}

std::vector<std::size_t> wizard_object_material_slots(const std::vector<WizardObjectSlots> &objects)
{
    std::vector<std::size_t> order;
    std::map<std::size_t, std::size_t> uses;
    const auto use = [&order, &uses](int one_based) {
        if (one_based < 1)
            return;
        const std::size_t slot = std::size_t(one_based - 1);
        if (uses[slot]++ == 0)
            order.push_back(slot);
    };
    std::vector<std::size_t> fallbacks;
    for (const WizardObjectSlots &object : objects) {
        const bool object_used = object.part_slots.empty() ||
            std::any_of(object.part_slots.begin(), object.part_slots.end(), [](int slot) { return slot < 1; });
        for (const int part : object.part_slots)
            use(part < 1 ? object.object_slot : part);
        if (object.part_slots.empty())
            use(object.object_slot);
        if (!object_used && object.object_slot >= 1)
            fallbacks.push_back(std::size_t(object.object_slot - 1));
    }
    // Most used first; the first seen among equals.
    std::stable_sort(order.begin(), order.end(), [&uses](std::size_t lhs, std::size_t rhs) {
        return uses[lhs] > uses[rhs];
    });
    for (const std::size_t slot : fallbacks)
        if (std::find(order.begin(), order.end(), slot) == order.end())
            order.push_back(slot);
    return order;
}

std::vector<std::string> wizard_body_material_changes(const std::vector<std::string> &part_names,
                                                      const std::vector<WizardBodyRoleRow> &rows,
                                                      const std::vector<WizardBodyAssignment> &body,
                                                      const std::vector<std::string> &filament_labels)
{
    const auto label = [&filament_labels](std::size_t slot) {
        return slot < filament_labels.size() ? filament_labels[slot] : "slot " + std::to_string(slot + 1);
    };
    std::vector<std::string> lines;
    for (const WizardBodyAssignment &assignment : body)
        for (std::size_t index = 0; index < rows.size(); ++index) {
            const WizardBodyRoleRow &row = rows[index];
            if (row.object_id != assignment.object_id || row.volume_id != assignment.volume_id)
                continue;
            const std::string name = index < part_names.size() ? part_names[index] : std::string("Part");
            if (row.current_slot < 0)
                lines.push_back(name + ": set to " + label(assignment.logical_filament));
            else if (std::size_t(row.current_slot) != assignment.logical_filament)
                lines.push_back(name + ": " + label(std::size_t(row.current_slot)) + " to " +
                                label(assignment.logical_filament));
            break;
        }
    return lines;
}

std::string wizard_body_roles_block_reason(const std::vector<WizardBodyRoleRow> &rows,
                                           const std::vector<WizardBodyRole> &roles)
{
    if (rows.empty() || std::any_of(rows.begin(), rows.end(), [](const WizardBodyRoleRow &row) { return row.painted; }))
        return {};
    const bool fine = std::find(roles.begin(), roles.end(), WizardBodyRole::Fine) != roles.end();
    const bool coarse = std::find(roles.begin(), roles.end(), WizardBodyRole::Coarse) != roles.end();
    return fine && coarse ? std::string() : std::string("Put at least one part on each nozzle.");
}

std::string wizard_joining_summary(std::size_t joined_assemblies)
{
    if (joined_assemblies == 0)
        return {};
    return "Parts that touch across the two nozzles are joined with interlocking beams.";
}

MixedNozzleSlicingMode wizard_default_mode(MixedNozzleSlicingMode configured, bool split_by_part)
{
    if (configured == MixedNozzleSlicingMode::BodySplit || split_by_part)
        return MixedNozzleSlicingMode::BodySplit;
    return MixedNozzleSlicingMode::FeatureSplit;
}

WizardMoreOptions wizard_more_options(const WizardDraft &draft, bool rebind_offered, bool scope_locked)
{
    WizardMoreOptions options;
    options.scope_locked = scope_locked;
    options.scope = scope_locked ? WizardScopeKind::ProjectDefault : draft.scope.kind;
    options.tower = draft.tower_intent;
    options.update_material_settings = rebind_offered;
    options.keep_current_process = draft.keep_current_process;
    return options;
}

void wizard_apply_more_options(WizardDraft &draft, const WizardMoreOptions &options)
{
    draft.scope.kind = options.scope_locked ? WizardScopeKind::ProjectDefault : options.scope;
    draft.tower_intent = options.tower;
    draft.keep_current_process = options.keep_current_process;
    // allow_project_binding_changes is set when the materials are captured.
}

std::string wizard_nozzle_flow_line(const std::vector<double> &nozzle_diameters,
                                    const std::vector<std::string> &flow_types)
{
    if (nozzle_diameters.size() != 2)
        return {};
    std::string line;
    for (std::size_t tool = 0; tool < 2; ++tool) {
        line += std::string(tool == 0 ? "Left " : ", right ") + height_text(nozzle_diameters[tool]) + " mm";
        if (tool < flow_types.size() && !flow_types[tool].empty())
            line += " " + flow_types[tool];
    }
    return line + ". Setup keeps these. Change them in the Printer panel.";
}

std::string wizard_material_flow_line(const std::string &material, std::optional<std::size_t> physical_extruder,
                                      double nozzle_diameter, const std::string &flow_type,
                                      std::optional<double> speed_limit)
{
    std::string line = material;
    if (physical_extruder && valid_height(nozzle_diameter)) {
        line += std::string(" on the ") + (*physical_extruder == 0 ? "left" : "right") + " " +
                height_text(nozzle_diameter) + " mm";
        if (!flow_type.empty())
            line += " " + flow_type;
    }
    if (!speed_limit || !std::isfinite(*speed_limit))
        line += ", speed limit not known";
    else if (*speed_limit > 0.)
        line += ", speed limit " + height_text(*speed_limit) + " mm\xc2\xb3/s";
    else
        line += ", no speed limit";
    return line;
}

// One process value the setup changes: "Layer height: 0.08 mm to 0.1 mm".
static std::string wizard_change_line(const WizardKeyDelta &delta)
{
    std::string name = delta.key;
    if (const ConfigOptionDef *definition = print_config_def.get(delta.key);
        definition != nullptr && !definition->label.empty())
        name = definition->label;
    const std::string unit = delta.unit == "mm" ? " mm" : std::string();
    const auto value = [&unit](const std::string &text) {
        return text.empty() || text == "absent" ? std::string("not set") : text + unit;
    };
    return name + ": " + value(delta.old_value) + " to " + value(delta.new_value);
}

namespace {

std::string nozzle_side_text(const WizardReviewSummaryInput &input, std::optional<std::size_t> physical)
{
    if (!physical || *physical >= input.nozzle_diameters.size())
        return {};
    return std::string(*physical == 0 ? "left" : "right") + " " +
           height_text(input.nozzle_diameters[*physical]) + " mm";
}

// The reason a Review blocker names, and where it is fixed.
WizardFixTarget fix_target_for(const std::string &raw_reason)
{
    std::string reason = raw_reason;
    if (reason.rfind("Candidate:", 0) == 0)
        reason.erase(0, 10);
    if (reason == "SharedProcessChangeOutsideScope")
        return WizardFixTarget::Consent;
    if (reason == "UnresolvedLogicalFilamentBinding" || reason == "ResolvedPhysicalRoleMismatch" ||
        reason.rfind("SingleNozzle", 0) == 0 || reason.rfind("MNS-MAP", 0) == 0)
        return WizardFixTarget::Materials;
    if (reason == "BodySharedBaseHeightBelowEnvelope" || reason == "CadenceEnumerationUnavailable" ||
        reason == "CadenceRatioInvalid" || reason == "FineHeightMissing" ||
        reason.rfind("SourceCompatibility", 0) == 0 || reason.rfind("MNS-CAD", 0) == 0)
        return WizardFixTarget::Speed;
    return WizardFixTarget::None;
}

std::string fix_target_label(WizardFixTarget target)
{
    switch (target) {
    case WizardFixTarget::Materials: return "Go to Materials";
    case WizardFixTarget::Speed: return "Go to Detail and speed";
    case WizardFixTarget::ProcessSetting: return "Open the setting";
    case WizardFixTarget::Consent:
    case WizardFixTarget::None: break;
    }
    return {};
}

} // namespace

WizardReviewSummary wizard_review_summary(const WizardReviewSummaryInput &input)
{
    WizardReviewSummary summary;
    const auto add_detail = [&summary](const std::string &line) {
        if (!line.empty() && std::find(summary.details.begin(), summary.details.end(), line) == summary.details.end())
            summary.details.push_back(line);
    };

    if (input.mode == MixedNozzleSlicingMode::Off) {
        // Off is a mode change and nothing else.
        summary.header = "Mixed-nozzle slicing will be turned off. Everything prints with the nozzle each "
                         "material is on now.";
    } else {
        const std::vector<double> pair = sorted_pair(input.nozzle_diameters);
        summary.header = std::string(input.mode == MixedNozzleSlicingMode::BodySplit ? "Body Split" : "Feature Split") +
            (pair.empty() ? std::string(" with both nozzles")
                          : " with the " + height_text(pair[0]) + " mm and " + height_text(pair[1]) + " mm nozzles");
        const std::string fine_where = nozzle_side_text(input, input.fine_physical);
        const std::string coarse_where = nozzle_side_text(input, input.coarse_physical);
        const auto layer_value = [](double height, const std::string &where, const std::string &material) {
            std::string value = height_text(height) + " mm";
            if (!where.empty())
                value += " on the " + where;
            if (!material.empty())
                value += ", " + material;
            return value;
        };
        if (input.mode == MixedNozzleSlicingMode::BodySplit) {
            if (!input.fine_parts.empty())
                summary.rows.push_back({"Fine parts", joined_names(input.fine_parts) + ", " +
                                        layer_value(input.fine_height, fine_where, {})});
            if (!input.coarse_parts.empty() && input.coarse_height)
                summary.rows.push_back({"Coarse parts", joined_names(input.coarse_parts) + ", " +
                                        layer_value(*input.coarse_height, coarse_where, {})});
            if (!input.joined_assemblies.empty())
                summary.rows.push_back({"Joining", "interlocking beams on " + joined_names(input.joined_assemblies)});
        } else {
            summary.rows.push_back({"Fine layers", layer_value(input.fine_height, fine_where, input.fine_material)});
            if (input.coarse_height)
                summary.rows.push_back({"Coarse layers",
                                        layer_value(*input.coarse_height, coarse_where, input.coarse_material)});
        }
        // Which nozzle prints the supports and which the interface, said before Apply.
        if (input.supports) {
            const auto side_value = [&input](const WizardSupportSide &side, const char *default_text) {
                if (side.slot <= 0)
                    return std::string(default_text);
                const std::string where = nozzle_side_text(input, side.physical);
                if (where.empty())
                    return side.material;
                return side.material.empty() ? where : where + ", " + side.material;
            };
            summary.rows.push_back({"Supports", side_value(input.support_base, "the nozzle already printing each layer (Default)")});
            summary.rows.push_back({"Support interface", side_value(input.support_interface, "same as the supports (Default)")});
            // An interface in another material peels off cleanly only with enough layers: 2 printed one.
            if (input.support_interface.slot > 0 && !input.support_interface.type.empty() &&
                !input.fine_material_type.empty() && input.support_interface.type != input.fine_material_type &&
                input.support_interface_top_layers < 3)
                summary.rows.push_back({"Interface layers", "Set Top interface layers to 3 (now " +
                    std::to_string(input.support_interface_top_layers) + "). With fewer, a " + input.support_interface.type +
                    " interface can be hard to peel off. It is in the Support settings."});
        }
        if (input.estimate && input.estimate->status == WizardEstimateStatus::Estimated) {
            std::string time = estimate_duration_text(input.estimate->seconds);
            if (input.baseline && input.baseline->status == WizardEstimateStatus::Estimated)
                time += " (one nozzle only: " + estimate_duration_text(input.baseline->seconds) + ")";
            summary.rows.push_back({"Print time", time});
        }
        if (!input.process_after.empty()) {
            std::string process = input.process_after;
            if (!input.process_before.empty() && input.process_before != input.process_after)
                process += " (was " + input.process_before + ")";
            summary.rows.push_back({"Process preset", process});
        }
        const char *tower = "set up automatically";
        switch (input.tower_intent) {
        case WizardTowerIntent::Automatic: tower = "set up automatically"; break;
        case WizardTowerIntent::Preserve: tower = "your own settings"; break;
        case WizardTowerIntent::Enabled: tower = "your own settings, always on"; break;
        case WizardTowerIntent::Disabled: tower = "off, though two nozzles need it"; break;
        }
        summary.rows.push_back({"Prime tower", tower});
        // Listed when it changes, with what the project had.
        if (input.first_layer.resolved && input.first_layer.differs)
            summary.rows.push_back({"First layer", height_text(input.first_layer.preset_height) + " mm at " +
                height_text(input.first_layer.preset_speed) + " mm/s, from the preset (was " +
                height_text(input.first_layer.height) + " mm at " + height_text(input.first_layer.speed) + " mm/s)"});
        summary.also_changed = input.also_changed;
    }

    // The ready line: the first thing in the way, in plain words, and where it is fixed.
    if (!input.setup_diagnostic.empty()) {
        summary.ready_line = "Can't apply yet. " + wizard_setup_diagnostic_message(input.setup_diagnostic);
    } else if (!input.engine_refusal.empty()) {
        summary.ready_line = "Can't apply yet. " + input.engine_refusal;
        // A refusal that names a Process setting links to it.
        summary.option_key = input.engine_refusal_key;
        summary.target = summary.option_key.empty() ? WizardFixTarget::None : WizardFixTarget::ProcessSetting;
    } else if (!input.blockers.empty()) {
        summary.ready_line = "Can't apply yet. " + wizard_review_reason_text(input.blockers.front());
        summary.target = fix_target_for(input.blockers.front());
    } else if (input.estimate && wizard_estimate_unsliceable(*input.estimate)) {
        // The ranking slice of this exact choice failed, so the real slice would fail too.
        summary.ready_line = input.estimate->refused && !input.estimate->refusal.empty()
            ? "Can't apply yet. Slicing refuses this coarse layer: " + input.estimate->refusal
            : std::string("Can't apply yet. This coarse layer could not be sliced. Pick another one.");
        summary.target = WizardFixTarget::Speed;
    } else {
        summary.ready = true;
        summary.ready_line = "Ready to apply.";
    }
    summary.target_label = fix_target_label(summary.target);

    for (const WizardKeyDelta &delta : input.changes)
        if (delta.source != "Native binding" && delta.old_value != delta.new_value)
            add_detail(wizard_change_line(delta));
    for (const std::string &line : input.material_lines)
        add_detail(line);
    add_detail(input.map_line);
    if (input.first_layer.resolved)
        add_detail(input.first_layer.review_line);
    for (const std::string &note : input.notes)
        add_detail(note);
    // The tower block once, from the resulting ledger.
    for (const std::string &line : input.tower_lines)
        add_detail(line);
    return summary;
}

WizardModeCardText wizard_mode_card_text(const std::vector<double> &nozzle_diameters)
{
    const std::vector<double> pair = sorted_pair(nozzle_diameters);
    const std::string fine = pair.empty() ? std::string("fine") : height_text(pair[0]) + " mm";
    const std::string coarse = pair.empty() ? std::string("coarse") : height_text(pair[1]) + " mm";
    WizardModeCardText text;
    // The fine nozzle gets walls, top and bottom surfaces and solid infill; the coarse nozzle gets
    // sparse infill. Supports follow the material slots the preset names, so they are described as
    // movable rather than as a default.
    text.feature = "Split by feature. By default the " + fine + " nozzle prints walls, top and bottom "
                   "surfaces and solid infill, and the " + coarse + " nozzle prints sparse infill. You "
                   "can change which nozzle prints each feature, supports included. Works on any model.";
    text.body = "Split by part. Each part prints entirely on the nozzle you pick for it, outside "
                "included. Selected for you when the plate has a multi-part or painted object.";
    return text;
}

} // namespace Slic3r::GUI
