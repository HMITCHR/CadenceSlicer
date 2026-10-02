#include "BodySplitEditorModel.hpp"
#include "FeatureSplitEditorSupport.hpp"
#include "libslic3r/libslic3r.h"
#include "libslic3r/MixedNozzleConfig.hpp"
#include "libslic3r/Slicing.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <map>
#include <set>

namespace Slic3r::GUI {
namespace {

// Byte offset of each code-point boundary, plus the end offset. Malformed bytes advance by
// one so a bound over untrusted user text always terminates.
std::vector<size_t> utf8_boundaries(const std::string& text)
{
    std::vector<size_t> boundaries{0};
    for (size_t offset = 0; offset < text.size(); ) {
        const unsigned char lead = static_cast<unsigned char>(text[offset]);
        size_t length = 1;
        if ((lead & 0xE0) == 0xC0) length = 2;
        else if ((lead & 0xF0) == 0xE0) length = 3;
        else if ((lead & 0xF8) == 0xF0) length = 4;
        if (offset + length > text.size())
            length = 1;
        for (size_t k = 1; k < length; ++k)
            if ((static_cast<unsigned char>(text[offset + k]) & 0xC0) != 0x80) {
                length = 1;
                break;
            }
        offset += length;
        boundaries.push_back(offset);
    }
    return boundaries;
}

std::string body_split_row_ordinal(size_t row_index) { return "Body " + std::to_string(row_index + 1); }

const BodySplitFilamentPresentation* find_presentation(const BodySplitPresentation& presentation, int logical_filament)
{
    const auto it = std::find_if(presentation.filaments.begin(), presentation.filaments.end(),
        [logical_filament](const BodySplitFilamentPresentation& item) {
            return item.logical_filament == logical_filament;
        });
    return it == presentation.filaments.end() ? nullptr : &*it;
}

// The slicer admits a body list, while the two physical nozzles remain the role
// boundary.  Keep the role checks per object: a multi-object plate needs one
// fine/coarse assignment in every object, and a coarse cadence must line up
// across objects.  This mirrors Print::validate() without treating a volume's
// position in the UI list as its identity.
BodySplitEditorDiagnostic qualify_object_rows(const std::vector<const BodySplitEditorRow *> &rows,
                                              const PrintConfig &resolver_config,
                                              int *coarse_ratio_out)
{
    if (rows.size() < 2)
        return BodySplitEditorDiagnostic::ExactlyTwoModelPartsRequired;

    std::set<int> logical_filaments;
    std::set<size_t> physical_tools;
    for (const BodySplitEditorRow *row : rows) {
        if (!std::isfinite(row->base_layer_height) || row->base_layer_height <= 0. ||
            !std::isfinite(row->effective_layer_height) || row->effective_layer_height <= 0.)
            return BodySplitEditorDiagnostic::LayerHeightInvalid;
        if (row->logical_filament <= 0)
            return BodySplitEditorDiagnostic::LogicalFilamentInvalid;
        if (!row->resolution.tool.has_value())
            return BodySplitEditorDiagnostic::PhysicalToolUnresolved;
        if (row->painted_tool_mismatch)
            return BodySplitEditorDiagnostic::PaintedFilamentPhysicalToolMismatch;
        logical_filaments.emplace(row->logical_filament);
        physical_tools.emplace(row->resolution.tool->physical_extruder);
    }

    // A logical filament resolves to one physical nozzle. Keep the specific diagnostic for that
    // impossible shape, and let a filament be reused by any number of bodies on its own tool.
    if (physical_tools.size() < 2) {
        return logical_filaments.size() < 2 ?
            BodySplitEditorDiagnostic::DistinctLogicalFilamentsRequired :
            BodySplitEditorDiagnostic::DistinctPhysicalToolsRequired;
    }
    // Body Split still targets the engine's two-nozzle mode.  The number of
    // model bodies is independent from this check.
    if (physical_tools.size() != 2 || resolver_config.nozzle_diameter.size() < 2)
        return BodySplitEditorDiagnostic::DistinctPhysicalToolsRequired;

    bool has_base = false;
    size_t base_tool = size_t(-1);
    int coarse_ratio = 0;
    double coarse_height = 0.;
    size_t coarse_tool = size_t(-1);
    for (const BodySplitEditorRow *row : rows) {
        const double raw_ratio = row->effective_layer_height / row->base_layer_height;
        const int integer_ratio = int(std::lround(raw_ratio));
        if (!std::isfinite(raw_ratio) || !is_approx(raw_ratio, double(integer_ratio)))
            return BodySplitEditorDiagnostic::CadenceNotQualified;
        if (integer_ratio == 1) {
            if (!has_base) {
                has_base = true;
                base_tool = row->resolution.tool->physical_extruder;
            } else if (base_tool != row->resolution.tool->physical_extruder) {
                // Print::validate() has one base cadence owner per object.
                return BodySplitEditorDiagnostic::CadenceNotQualified;
            }
        } else if (integer_ratio >= 2) {
            if (coarse_ratio == 0) {
                coarse_ratio = integer_ratio;
                coarse_height = row->effective_layer_height;
                coarse_tool = row->resolution.tool->physical_extruder;
            } else if (coarse_ratio != integer_ratio ||
                       !is_approx(coarse_height, row->effective_layer_height) ||
                       coarse_tool != row->resolution.tool->physical_extruder) {
                // All coarse bodies on one object share the engine's single
                // physical band schedule and exact effective height.
                return BodySplitEditorDiagnostic::CadenceNotQualified;
            }
        } else {
            return BodySplitEditorDiagnostic::CadenceNotQualified;
        }
    }
    if (!has_base || coarse_ratio < 2 || base_tool == size_t(-1) || coarse_tool == size_t(-1))
        return BodySplitEditorDiagnostic::CadenceNotQualified;
    if (base_tool == coarse_tool)
        return BodySplitEditorDiagnostic::DistinctPhysicalToolsRequired;

    if (coarse_tool >= resolver_config.nozzle_diameter.size() ||
        base_tool >= resolver_config.nozzle_diameter.size())
        return BodySplitEditorDiagnostic::PhysicalToolUnresolved;
    if (resolver_config.nozzle_diameter.get_at(coarse_tool) <=
        resolver_config.nozzle_diameter.get_at(base_tool))
        return BodySplitEditorDiagnostic::PhysicalNozzleOrderingInvalid;

    // Like the fine tool, the coarse tool takes any number of logical filaments, sharing the one
    // coarse cadence checked above.

    // The same cadence helper Print::validate and body_split_cadence_choices use, so a row
    // qualifies exactly when its cadence is one the editor offers and the slicer admits. A coarse
    // nozzle whose minimum is above the base starts on its own first cell, which needs the lagging
    // tower when a prime tower is on. The first layer is taken at the base height, as in the
    // choice list: this editor sets cadences, not the first layer.
    const bool with_prime_tower = resolver_config.enable_prime_tower.value;
    const bool lagging_tower_available = mixed_nozzle_tower_lagging_available(resolver_config);
    for (const BodySplitEditorRow *row : rows) {
        const size_t nozzle = row->resolution.tool->physical_extruder;
        const BodySplitToolEnvelope envelope = body_split_tool_envelope(
            resolver_config, nozzle, row->base_layer_height, row->effective_layer_height, row->base_layer_height);
        if (!body_split_tool_envelope_admitted(envelope, with_prime_tower, lagging_tower_available) ||
            row->effective_layer_height > row->resolution.tool->nozzle_diameter + EPSILON)
            return BodySplitEditorDiagnostic::CadenceNotQualified;
    }
    if (coarse_ratio_out != nullptr)
        *coarse_ratio_out = coarse_ratio;
    return BodySplitEditorDiagnostic::None;
}

BodySplitEditorDiagnostic qualify_rows(std::vector<BodySplitEditorRow> &rows,
                                      const PrintConfig &resolver_config)
{
    if (rows.empty())
        return BodySplitEditorDiagnostic::ExactlyTwoModelPartsRequired;

    // Qualify actual painted usage without adding fictional model bodies to the editor.
    // A cross-tool colour takes its explicitly assigned body's cadence, or the native
    // base cadence when it paints the sole coarse body. Ghost PrintRegions never enter
    // this list: painted_logical_filaments comes directly from the model facets.
    std::vector<BodySplitEditorRow> painted_rows;
    for (BodySplitEditorRow &row : rows) {
        row.painted_tool_mismatch = false;
        if (!std::isfinite(row.base_layer_height) || row.base_layer_height <= 0. ||
            !std::isfinite(row.effective_layer_height) || row.effective_layer_height <= 0.)
            return BodySplitEditorDiagnostic::LayerHeightInvalid;
        if (row.logical_filament <= 0)
            return BodySplitEditorDiagnostic::LogicalFilamentInvalid;
        if (!row.resolution)
            return BodySplitEditorDiagnostic::PhysicalToolUnresolved;
        for (int logical : row.painted_logical_filaments) {
            const auto target = logical > 0 ? resolve_mixed_nozzle_tool(
                resolver_config, size_t(logical - 1), MixedNozzleResolveScope::PhysicalToolOnly) :
                MixedNozzleToolResolution{};
            std::optional<double> cadence;
            if (target && target.tool->physical_extruder == row.resolution.tool->physical_extruder)
                cadence = row.effective_layer_height;
            else if (target) {
                bool ambiguous = false;
                for (const BodySplitEditorRow &body : rows)
                    if (body.object_id == row.object_id && body.logical_filament == logical) {
                        ambiguous = ambiguous || (cadence && !is_approx(*cadence, body.effective_layer_height));
                        cadence = body.effective_layer_height;
                    }
                // Mirror the slicer: a colour no body is bound to takes the cadence of the bodies
                // on its own physical tool.
                if (!cadence)
                    for (const BodySplitEditorRow &body : rows)
                        if (body.object_id == row.object_id && body.resolution &&
                            body.resolution.tool->physical_extruder == target.tool->physical_extruder) {
                            ambiguous = ambiguous || (cadence && !is_approx(*cadence, body.effective_layer_height));
                            cadence = body.effective_layer_height;
                        }
                const size_t object_body_count = std::count_if(rows.begin(), rows.end(),
                    [&](const BodySplitEditorRow &body) { return body.object_id == row.object_id; });
                if (!cadence && object_body_count == 1)
                    cadence = row.base_layer_height;
                if (cadence && !ambiguous) {
                    const bool target_is_fine = *cadence < row.effective_layer_height;
                    const auto qualified = target_is_fine ?
                        resolve_mixed_nozzle_cadence(resolver_config, *cadence, row.effective_layer_height,
                                                    size_t(logical - 1), size_t(row.logical_filament - 1)) :
                        resolve_mixed_nozzle_cadence(resolver_config, row.effective_layer_height, *cadence,
                                                    size_t(row.logical_filament - 1), size_t(logical - 1));
                    if (!qualified)
                        cadence.reset();
                } else {
                    cadence.reset();
                }
            }
            if (!target || !cadence) {
                row.painted_tool_mismatch = true;
                return BodySplitEditorDiagnostic::PaintedFilamentPhysicalToolMismatch;
            }
            BodySplitEditorRow painted = row;
            painted.logical_filament = logical;
            painted.resolution = target;
            painted.effective_layer_height = *cadence;
            painted.cadence_ratio = *cadence / row.base_layer_height;
            painted.painted_logical_filaments.clear();
            painted_rows.push_back(std::move(painted));
        }
    }
    std::vector<BodySplitEditorRow> qualified_rows = rows;
    qualified_rows.insert(qualified_rows.end(), painted_rows.begin(), painted_rows.end());

    std::map<ObjectID, std::vector<const BodySplitEditorRow *>> by_object;
    for (const BodySplitEditorRow &row : qualified_rows)
        by_object[row.object_id].push_back(&row);

    std::optional<int> plate_coarse_ratio;
    for (const auto &entry : by_object) {
        int object_coarse_ratio = 0;
        const BodySplitEditorDiagnostic diagnostic = qualify_object_rows(entry.second, resolver_config,
                                                                          &object_coarse_ratio);
        if (diagnostic != BodySplitEditorDiagnostic::None)
            return diagnostic;
        if (!plate_coarse_ratio)
            plate_coarse_ratio = object_coarse_ratio;
        else if (*plate_coarse_ratio != object_coarse_ratio)
            return BodySplitEditorDiagnostic::CadenceNotQualified;
    }
    return BodySplitEditorDiagnostic::None;
}

ModelVolume* find_target(Model& model, ObjectID object_id, ObjectID volume_id, ModelObject** found_object = nullptr)
{
    for (ModelObject* object : model.objects) {
        if (object->id() != object_id)
            continue;
        for (ModelVolume* volume : object->volumes) {
            if (volume->id() == volume_id && volume->type() == ModelVolumeType::MODEL_PART) {
                if (found_object != nullptr)
                    *found_object = object;
                return volume;
            }
        }
        return nullptr;
    }
    return nullptr;
}

bool phase_equals(const DynamicPrintConfig& config, RegionalGridPhaseRule value)
{
    const auto* option = config.option<ConfigOptionEnum<RegionalGridPhaseRule>>("regional_grid_phase_rule");
    return option != nullptr && option->value == value;
}

// Whether writing `patch` would change the volume's `key`.
template<typename Option, typename T>
bool volume_value_changes(const ModelVolume& volume, const char* key, const BodySplitValuePatch<T>& patch)
{
    if (!patch.requested)
        return false;
    if (!patch.value.has_value())
        return volume.config.has(key);
    const auto* current = volume.config.get().option<Option>(key);
    return current == nullptr || current->value != *patch.value;
}

template<typename Option, typename T>
void write_volume_value(ModelVolume& volume, const char* key, const std::optional<T>& value)
{
    if (value.has_value())
        volume.config.set_key_value(key, new Option(*value));
    else
        volume.config.erase(key);
}

} // namespace

PrintConfig mixed_nozzle_resolver_config_from_full(const DynamicPrintConfig& full_config)
{
    PrintConfig resolver;
    resolver.apply(full_config, true);
    return resolver;
}

BodySplitEditorModel build_body_split_editor_model(const std::vector<ModelObject*>& objects,
                                                    const PrintConfig& resolver_config,
                                                    const BodySplitPresentation& presentation,
                                                    double inherited_layer_height)
{
    BodySplitEditorModel result;
    for (const ModelObject* object : objects) {
        const double base_height = object->config.has("layer_height") ?
                                   object->config.opt_float("layer_height") : inherited_layer_height;
        for (const ModelVolume* volume : object->volumes) {
            if (volume->type() != ModelVolumeType::MODEL_PART) {
                result.excluded.push_back({object->id(), volume->id(), object->name, volume->name, volume->type()});
                continue;
            }

            BodySplitEditorRow row;
            row.object_id = object->id();
            row.volume_id = volume->id();
            row.object_name = object->name;
            row.volume_name = volume->name;
            row.type = volume->type();
            row.logical_filament = volume->config.has("extruder") ? volume->config.extruder() : 0;
            // A painted part with no filament of its own prints with the object filament, which
            // is what the slicer admits. Unpainted rows keep asking for a choice.
            if (row.logical_filament <= 0 && volume->is_mm_painted())
                row.logical_filament = volume->extruder_id();
            if (const auto* labels = find_presentation(presentation, row.logical_filament)) {
                row.preset_label = labels->preset_label;
                row.material_label = labels->material_label;
            }
            row.base_layer_height = base_height;
            const bool has_height = volume->config.has("regional_layer_height");
            const double configured_height = has_height ? volume->config.opt_float("regional_layer_height") : 0.;
            row.regional_height_explicit = has_height && configured_height != 0.;
            row.effective_layer_height = row.regional_height_explicit ? configured_height : base_height;
            row.cadence_ratio = base_height > 0. ? row.effective_layer_height / base_height : 0.;
            // The fine-skin controls only apply to a coarse-cadence row (an integer ratio >= 2).
            // qualify_rows() below decides whether the pair is otherwise legal.
            const int cadence_ratio_rounded = int(std::lround(row.cadence_ratio));
            row.fine_skin_controls_applicable = std::isfinite(row.cadence_ratio) &&
                is_approx(row.cadence_ratio, double(cadence_ratio_rounded)) && cadence_ratio_rounded >= 2;
            row.fine_skins = volume->config.has("mixed_nozzle_body_fine_skins") ?
                volume->config.get().opt_bool("mixed_nozzle_body_fine_skins") : false;
            row.fine_skin_layers = volume->config.has("mixed_nozzle_body_fine_skin_layers") ?
                volume->config.opt_int("mixed_nozzle_body_fine_skin_layers") : 3;
            if (row.logical_filament > 0)
                row.resolution = resolve_mixed_nozzle_tool(resolver_config, size_t(row.logical_filament - 1),
                                                          MixedNozzleResolveScope::PhysicalToolOnly);
            // Keep explicitly painted facets even when their colour currently equals
            // the body's default; editing that default must not erase their intent.
            for (const size_t painted : volume->get_extruders_from_multi_material_painting())
                row.painted_logical_filaments.push_back(int(painted + 1));
            std::sort(row.painted_logical_filaments.begin(), row.painted_logical_filaments.end());
            row.cadence_choices = body_split_cadence_choices(base_height, resolver_config);
            result.rows.emplace_back(std::move(row));
        }
    }

    result.diagnostic = qualify_rows(result.rows, resolver_config);
    result.supported = result.diagnostic == BodySplitEditorDiagnostic::None;
    return result;
}

bool body_split_rows_have_body_object(const std::vector<BodySplitEditorRow>& rows)
{
    std::map<size_t, size_t> parts_per_object;
    for (const BodySplitEditorRow& row : rows)
        if (!row.painted_logical_filaments.empty() || ++parts_per_object[row.object_id.id] >= 2)
            return true;
    return false;
}

bool body_split_editor_visible(MixedNozzleSlicingMode effective_mode)
{
    return effective_mode == MixedNozzleSlicingMode::BodySplit;
}

bool body_split_editor_target_valid(const BodySplitStableTarget& target,
                                    int current_plate_index,
                                    size_t indexed_plate_id)
{
    return target.plate_index >= 0 && target.plate_index == current_plate_index &&
           target.plate_id != 0 && target.plate_id == indexed_plate_id;
}

int plate_settings_target_index(int event_plate_index, int current_plate_index, size_t plate_count)
{
    if (event_plate_index >= 0 && size_t(event_plate_index) < plate_count)
        return event_plate_index;
    return current_plate_index >= 0 && size_t(current_plate_index) < plate_count ? current_plate_index : -1;
}

std::string body_split_filament_label(size_t one_based_filament, const std::string& preset_label)
{
    return std::to_string(one_based_filament) + ": " + preset_label;
}

std::vector<double> mixed_nozzle_cadence_choices(double base_layer_height, const PrintConfig& resolver_config)
{
    if (!std::isfinite(base_layer_height) || base_layer_height <= 0.)
        return {};
    std::vector<double> choices{base_layer_height};
    const std::vector<double> coarse = mixed_nozzle_admissible_coarse_heights(base_layer_height, resolver_config);
    choices.insert(choices.end(), coarse.begin(), coarse.end());
    return choices;
}

std::vector<double> body_split_cadence_choices(double base_layer_height, const PrintConfig& resolver_config)
{
    auto choices = mixed_nozzle_cadence_choices(base_layer_height, resolver_config);
    if (choices.size() <= 1)
        return choices;

    const size_t fine_nozzle = resolver_config.nozzle_diameter.get_at(0) <
                              resolver_config.nozzle_diameter.get_at(1) ? 0 : 1;
    const size_t coarse_nozzle = 1 - fine_nozzle;
    // The same cadence helper Print::validate uses, so this never offers a cadence the slicer
    // refuses. A coarse nozzle whose minimum is above the base starts its body on its own first
    // cell; with a prime tower that needs the lagging tower, which smooth timelapse and wrapping
    // detection rule out. The first layer is taken at the base height.
    const bool with_prime_tower = resolver_config.enable_prime_tower.value;
    const bool lagging_tower_available = mixed_nozzle_tower_lagging_available(resolver_config);
    const BodySplitToolEnvelope fine = body_split_tool_envelope(
        resolver_config, fine_nozzle, base_layer_height, base_layer_height, base_layer_height);
    if (!body_split_tool_envelope_admitted(fine, with_prime_tower, lagging_tower_available)) {
        choices.resize(1);
        return choices;
    }
    choices.erase(std::remove_if(choices.begin() + 1, choices.end(), [&](double coarse) {
        return !body_split_tool_envelope_admitted(
            body_split_tool_envelope(resolver_config, coarse_nozzle, base_layer_height, coarse, base_layer_height),
            with_prime_tower, lagging_tower_available);
    }), choices.end());
    return choices;
}

BodySplitBoundedText bounded_body_split_text(const std::string& text, size_t max_chars)
{
    // Bound by code point rather than byte: object and volume names are user text, and the
    // row identity separator is an em dash, so a byte-wise cut would emit invalid UTF-8.
    const std::vector<size_t> boundaries = utf8_boundaries(text);
    BodySplitBoundedText result{text, text};
    const size_t characters = boundaries.size() - 1;
    if (characters <= max_chars)
        return result;
    if (max_chars <= 3) {
        result.display.assign(max_chars, '.');
        return result;
    }
    result.display = text.substr(0, boundaries[max_chars - 3]) + "...";
    return result;
}

std::vector<BodySplitBoundedText> body_split_row_identities(const std::vector<BodySplitEditorRow>& rows,
                                                            size_t max_chars)
{
    // Leaf first. Sibling bodies share one parent object name, so a parent-first label spends
    // the whole bounded width on text that is identical in every row.
    std::vector<std::string> full;
    full.reserve(rows.size());
    for (size_t i = 0; i < rows.size(); ++i) {
        const std::string leaf = rows[i].volume_name.empty() ? body_split_row_ordinal(i) : rows[i].volume_name;
        full.push_back(rows[i].object_name.empty() ? leaf : leaf + " \xE2\x80\x94 " + rows[i].object_name);
    }

    std::map<std::string, size_t> occurrences;
    for (const std::string& text : full)
        ++occurrences[text];

    std::vector<BodySplitBoundedText> identities;
    identities.reserve(rows.size());
    for (size_t i = 0; i < rows.size(); ++i) {
        // Ordinal context is a fallback for names that cannot tell the rows apart, and it is
        // prefixed rather than appended so truncation can never remove the differentiator.
        const std::string text = occurrences[full[i]] > 1 ? body_split_row_ordinal(i) + ": " + full[i] : full[i];
        identities.push_back(bounded_body_split_text(text, max_chars));
    }
    return identities;
}

BodySplitEditorDiagnostic body_split_native_selection_diagnostic(
    const std::vector<BodySplitEditorRow>& rows,
    const std::vector<BodySplitNativeSelection>& selections,
    size_t filament_count,
    const PrintConfig& resolver_config)
{
    if (rows.empty())
        return BodySplitEditorDiagnostic::ExactlyTwoModelPartsRequired;
    if (selections.size() != rows.size())
        return BodySplitEditorDiagnostic::StaleTarget;

    using TargetKey = std::pair<ObjectID, ObjectID>;
    std::map<TargetKey, const BodySplitNativeSelection *> captured;
    for (const BodySplitNativeSelection &selection : selections) {
        if (!captured.emplace(TargetKey{selection.object_id, selection.volume_id}, &selection).second)
            return BodySplitEditorDiagnostic::DuplicateTarget;
    }

    std::vector<BodySplitEditorRow> proposed = rows;
    std::set<TargetKey> row_targets;
    for (size_t i = 0; i < rows.size(); ++i) {
        const auto row_target = TargetKey{rows[i].object_id, rows[i].volume_id};
        if (!row_targets.emplace(row_target).second)
            return BodySplitEditorDiagnostic::DuplicateTarget;
        const auto selection_it = captured.find(row_target);
        if (selection_it == captured.end())
            return BodySplitEditorDiagnostic::StaleTarget;
        const BodySplitNativeSelection &selection = *selection_it->second;
        const auto& choices = rows[i].cadence_choices;
        if (selection.filament_selection < 0 || size_t(selection.filament_selection) >= filament_count)
            return BodySplitEditorDiagnostic::LogicalFilamentInvalid;
        if (selection.cadence_selection < 0 || size_t(selection.cadence_selection) >= choices.size())
            return BodySplitEditorDiagnostic::CadenceNotQualified;
        proposed[i].logical_filament = selection.filament_selection + 1;
        proposed[i].effective_layer_height = choices[size_t(selection.cadence_selection)];
        proposed[i].cadence_ratio = proposed[i].effective_layer_height / proposed[i].base_layer_height;
        proposed[i].resolution = resolve_mixed_nozzle_tool(resolver_config,
            size_t(selection.filament_selection), MixedNozzleResolveScope::PhysicalToolOnly);
    }
    return qualify_rows(proposed, resolver_config);
}

static std::optional<BodySplitApplyRequest> build_body_split_native_request(
    const std::vector<BodySplitEditorRow>& rows,
    const std::vector<BodySplitNativeSelection>& selections,
    size_t filament_count,
    RegionalGridPhaseRule phase)
{
    if (rows.empty() || selections.size() != rows.size())
        return std::nullopt;

    using TargetKey = std::pair<ObjectID, ObjectID>;
    std::map<TargetKey, const BodySplitNativeSelection *> captured;
    for (const BodySplitNativeSelection &selection : selections)
        if (!captured.emplace(TargetKey{selection.object_id, selection.volume_id}, &selection).second)
            return std::nullopt;

    BodySplitApplyRequest request;
    std::set<ObjectID> object_ids;
    std::set<TargetKey> row_targets;
    for (size_t i = 0; i < rows.size(); ++i) {
        const BodySplitEditorRow& row = rows[i];
        const auto row_target = TargetKey{row.object_id, row.volume_id};
        if (!row_targets.emplace(row_target).second)
            return std::nullopt;
        const auto selection_it = captured.find(row_target);
        if (selection_it == captured.end())
            return std::nullopt;
        const BodySplitNativeSelection& selection = *selection_it->second;
        const auto& cadences = row.cadence_choices;
        if (selection.filament_selection < 0 || selection.cadence_selection < 0 ||
            size_t(selection.filament_selection) >= filament_count ||
            size_t(selection.cadence_selection) >= cadences.size())
            return std::nullopt;

        // Native choices are zero-based; the model-volume authority is one-based.
        const int logical_filament = selection.filament_selection + 1;
        BodySplitEdit edit;
        edit.object_id = row.object_id;
        edit.volume_id = row.volume_id;
        edit.extruder = BodySplitValuePatch<int>::set(logical_filament);
        edit.regional_layer_height = selection.cadence_selection == 0
            ? BodySplitValuePatch<double>::erase()
            : BodySplitValuePatch<double>::set(cadences[size_t(selection.cadence_selection)]);
        request.edits.emplace_back(std::move(edit));
        object_ids.emplace(row.object_id);
    }
    request.editable_object_ids.assign(object_ids.begin(), object_ids.end());
    request.phase = BodySplitValuePatch<RegionalGridPhaseRule>::set(phase);
    return request;
}

BodySplitLiveRequest build_body_split_live_request(
    const std::vector<BodySplitEditorRow>& live_rows,
    const std::vector<BodySplitNativeSelection>& captured_selections,
    size_t filament_count,
    const PrintConfig& live_resolver_config,
    RegionalGridPhaseRule phase)
{
    BodySplitLiveRequest result;
    if (live_rows.empty() || live_rows.size() != captured_selections.size()) {
        result.diagnostic = BodySplitEditorDiagnostic::StaleTarget;
        return result;
    }

    using TargetKey = std::pair<ObjectID, ObjectID>;
    std::map<TargetKey, const BodySplitNativeSelection *> captured;
    for (const BodySplitNativeSelection &selection : captured_selections)
        if (!captured.emplace(TargetKey{selection.object_id, selection.volume_id}, &selection).second) {
            result.diagnostic = BodySplitEditorDiagnostic::DuplicateTarget;
            return result;
        }

    std::vector<BodySplitNativeSelection> live_selections;
    live_selections.reserve(live_rows.size());
    std::set<TargetKey> live_targets;
    for (const BodySplitEditorRow& row : live_rows) {
        const TargetKey row_target{row.object_id, row.volume_id};
        if (!live_targets.emplace(row_target).second) {
            result.diagnostic = BodySplitEditorDiagnostic::DuplicateTarget;
            return result;
        }
        const auto selection = captured.find(row_target);
        if (selection == captured.end()) {
            result.diagnostic = BodySplitEditorDiagnostic::StaleTarget;
            return result;
        }
        live_selections.push_back(*selection->second);
    }

    result.diagnostic = body_split_native_selection_diagnostic(live_rows, live_selections,
                                                               filament_count, live_resolver_config);
    if (result.diagnostic != BodySplitEditorDiagnostic::None)
        return result;
    result.request = build_body_split_native_request(live_rows, live_selections, filament_count, phase);
    if (!result.request) {
        result.diagnostic = BodySplitEditorDiagnostic::StaleTarget;
        return result;
    }
    result.request->inherited_layer_height = live_rows.empty() ? 0. : live_rows.front().base_layer_height;
    return result;
}

void merge_body_split_fine_skins(BodySplitApplyRequest& request, const std::vector<BodySplitEditorRow>& rows,
                                 size_t widget_count, const std::function<bool(size_t)>& fine_skins,
                                 const std::function<int(size_t)>& fine_skin_layers)
{
    for (size_t i = 0; i < rows.size() && i < widget_count; ++i) {
        if (!rows[i].fine_skin_controls_applicable)
            continue;
        const auto edit = std::find_if(request.edits.begin(), request.edits.end(), [&](const BodySplitEdit& candidate) {
            return candidate.object_id == rows[i].object_id && candidate.volume_id == rows[i].volume_id;
        });
        if (edit == request.edits.end())
            continue;
        edit->fine_skins = BodySplitValuePatch<bool>::set(fine_skins(i));
        edit->fine_skin_layers = BodySplitValuePatch<int>::set(fine_skin_layers(i));
    }
}

BodySplitBeamLayers body_split_beam_layers(const std::vector<BodySplitEditorRow>& rows, ObjectID object_id)
{
    BodySplitBeamLayers result;
    double ratio = 0.;
    for (const BodySplitEditorRow& row : rows)
        if (row.object_id == object_id) {
            result.has_bodies = true;
            ratio = std::max(ratio, row.cadence_ratio);
        }
    if (result.has_bodies && std::isfinite(ratio) && ratio >= 2. &&
        ratio <= double(std::numeric_limits<int>::max()) && is_approx(ratio, std::round(ratio)))
        result.layers = int(std::lround(ratio));
    return result;
}

BodySplitApplyResult apply_body_split_editor_request(Model& model,
                                                     DynamicPrintConfig& plate_config,
                                                     const PrintConfig& resolver_config,
                                                     const BodySplitApplyRequest& request,
                                                     const BodySplitMutationHooks& hooks)
{
    BodySplitApplyResult result;
    if (!request.apply)
        return result;

    std::vector<ModelObject*> editable_objects;
    std::set<ObjectID> object_ids;
    for (ObjectID object_id : request.editable_object_ids) {
        if (!object_ids.emplace(object_id).second) {
            result.diagnostic = BodySplitEditorDiagnostic::DuplicateTarget;
            return result;
        }
        const auto object = std::find_if(model.objects.begin(), model.objects.end(), [object_id](const ModelObject* item) {
            return item->id() == object_id;
        });
        if (object == model.objects.end()) {
            result.diagnostic = BodySplitEditorDiagnostic::StaleTarget;
            return result;
        }
        editable_objects.push_back(*object);
    }

    std::set<std::pair<ObjectID, ObjectID>> targets;
    for (const BodySplitEdit& edit : request.edits) {
        if (object_ids.find(edit.object_id) == object_ids.end()) {
            result.diagnostic = BodySplitEditorDiagnostic::StaleTarget;
            return result;
        }
        if (!targets.emplace(edit.object_id, edit.volume_id).second) {
            result.diagnostic = BodySplitEditorDiagnostic::DuplicateTarget;
            return result;
        }
        if (find_target(model, edit.object_id, edit.volume_id) == nullptr) {
            result.diagnostic = BodySplitEditorDiagnostic::StaleTarget;
            return result;
        }
    }

    BodySplitEditorModel proposed = build_body_split_editor_model(editable_objects, resolver_config, {},
                                                                 request.inherited_layer_height);
    // The beam layer count each object's cadence asks for before these edits.
    std::map<ObjectID, std::optional<int>> beam_layers_before;
    for (const ModelObject* object : editable_objects)
        beam_layers_before[object->id()] = body_split_beam_layers(proposed.rows, object->id()).layers;
    for (const BodySplitEdit& edit : request.edits) {
        auto row = std::find_if(proposed.rows.begin(), proposed.rows.end(), [&edit](const BodySplitEditorRow& candidate) {
            return candidate.object_id == edit.object_id && candidate.volume_id == edit.volume_id;
        });
        if (row == proposed.rows.end()) {
            result.diagnostic = BodySplitEditorDiagnostic::StaleTarget;
            return result;
        }
        if (edit.extruder.requested) {
            row->logical_filament = edit.extruder.value.value_or(0);
            if (row->logical_filament > 0)
                row->resolution = resolve_mixed_nozzle_tool(resolver_config, size_t(row->logical_filament - 1),
                                                            MixedNozzleResolveScope::PhysicalToolOnly);
            else
                row->resolution = {};
        }
        if (edit.regional_layer_height.requested) {
            row->regional_height_explicit = edit.regional_layer_height.value.has_value() &&
                                            *edit.regional_layer_height.value != 0.;
            row->effective_layer_height = row->regional_height_explicit ?
                                          *edit.regional_layer_height.value : row->base_layer_height;
            row->cadence_ratio = row->base_layer_height > 0. ?
                                 row->effective_layer_height / row->base_layer_height : 0.;
        }
        if (edit.fine_skins.requested)
            row->fine_skins = edit.fine_skins.value.value_or(false);
        if (edit.fine_skin_layers.requested)
            row->fine_skin_layers = edit.fine_skin_layers.value.value_or(3);
    }

    result.diagnostic = qualify_rows(proposed.rows, resolver_config);
    if (result.diagnostic != BodySplitEditorDiagnostic::None)
        return result;

    struct PendingEdit {
        ModelObject* object;
        ModelVolume* volume;
        const BodySplitEdit* edit;
        bool extruder_changed;
        bool height_changed;
        bool fine_skins_changed;
        bool fine_skin_layers_changed;
    };
    std::vector<PendingEdit> pending;
    bool any_change = false;
    for (const BodySplitEdit& edit : request.edits) {
        ModelObject* object = nullptr;
        ModelVolume* volume = find_target(model, edit.object_id, edit.volume_id, &object);
        const bool extruder_changed = volume_value_changes<ConfigOptionInt>(*volume, "extruder", edit.extruder);
        const bool height_changed =
            volume_value_changes<ConfigOptionFloat>(*volume, "regional_layer_height", edit.regional_layer_height);
        const bool fine_skins_changed =
            volume_value_changes<ConfigOptionBool>(*volume, "mixed_nozzle_body_fine_skins", edit.fine_skins);
        const bool fine_skin_layers_changed =
            volume_value_changes<ConfigOptionInt>(*volume, "mixed_nozzle_body_fine_skin_layers", edit.fine_skin_layers);
        if (extruder_changed || height_changed || fine_skins_changed || fine_skin_layers_changed) {
            pending.push_back({object, volume, &edit, extruder_changed, height_changed,
                               fine_skins_changed, fine_skin_layers_changed});
            any_change = true;
        }
    }

    // A cadence change moves the beam rows: interlocking beams need one beam layer per fine layer in
    // a coarse layer, and Slice refuses any other count (SRL-A28). Setup sizes them from the same
    // rows, so an edited cadence carries the count with it, on the object, as setup writes it.
    struct PendingBeamLayers {
        ModelObject* object;
        int layers;
    };
    std::vector<PendingBeamLayers> beam_layers;
    for (ModelObject* object : editable_objects) {
        const std::optional<int> after = body_split_beam_layers(proposed.rows, object->id()).layers;
        if (!after || after == beam_layers_before[object->id()])
            continue;
        const auto* current = object->config.get().option<ConfigOptionInt>("interlocking_beam_layer_count");
        if (current != nullptr && current->value == *after)
            continue;
        beam_layers.push_back({object, *after});
        any_change = true;
    }

    const bool plate_changed = request.phase.requested &&
        (request.phase.value.has_value() ? !phase_equals(plate_config, *request.phase.value) :
                                          plate_config.has("regional_grid_phase_rule"));
    any_change = any_change || plate_changed;
    result.applied = true;
    if (!any_change)
        return result;

    if (hooks.snapshot)
        hooks.snapshot();

    for (const PendingEdit& item : pending) {
        if (item.extruder_changed)
            write_volume_value<ConfigOptionInt>(*item.volume, "extruder", item.edit->extruder.value);
        if (item.height_changed)
            write_volume_value<ConfigOptionFloat>(*item.volume, "regional_layer_height",
                                                  item.edit->regional_layer_height.value);
        if (item.fine_skins_changed)
            write_volume_value<ConfigOptionBool>(*item.volume, "mixed_nozzle_body_fine_skins",
                                                 item.edit->fine_skins.value);
        if (item.fine_skin_layers_changed)
            write_volume_value<ConfigOptionInt>(*item.volume, "mixed_nozzle_body_fine_skin_layers",
                                                item.edit->fine_skin_layers.value);
        if (std::find(result.changed_object_ids.begin(), result.changed_object_ids.end(), item.object->id()) ==
            result.changed_object_ids.end())
            result.changed_object_ids.push_back(item.object->id());
    }
    for (const PendingBeamLayers& item : beam_layers) {
        item.object->config.set_key_value("interlocking_beam_layer_count", new ConfigOptionInt(item.layers));
        if (std::find(result.changed_object_ids.begin(), result.changed_object_ids.end(), item.object->id()) ==
            result.changed_object_ids.end())
            result.changed_object_ids.push_back(item.object->id());
    }
    if (plate_changed) {
        if (request.phase.value.has_value())
            plate_config.set_key_value("regional_grid_phase_rule",
                new ConfigOptionEnum<RegionalGridPhaseRule>(*request.phase.value));
        else
            plate_config.erase("regional_grid_phase_rule");
    }

    result.changed = true;
    result.plate_changed = plate_changed;
    if (hooks.notify_and_invalidate)
        hooks.notify_and_invalidate();
    return result;
}

bool BodySplitEditorTransaction::stage(const BodySplitApplyRequest& request)
{
    if (!request.apply)
        return false;
    m_pending = request;
    return true;
}

BodySplitApplyResult BodySplitEditorTransaction::commit(Model& model,
                                                        DynamicPrintConfig& plate_config,
                                                        const PrintConfig& resolver_config,
                                                        const BodySplitMutationHooks& hooks)
{
    if (!m_pending)
        return {};
    const BodySplitApplyRequest request = std::move(*m_pending);
    m_pending.reset();
    if (hooks.transactional_commit)
        return hooks.transactional_commit(request);
    return apply_body_split_editor_request(model, plate_config, resolver_config, request, hooks);
}

BodySplitPlateSettingsAcceptanceResult accept_body_split_plate_settings(
    const BodySplitStableTarget& target,
    int current_plate_index,
    size_t indexed_plate_id,
    MixedNozzleSlicingMode effective_mode,
    BodySplitEditorTransaction& transaction,
    Model& model,
    DynamicPrintConfig& plate_config,
    const PrintConfig& resolver_config,
    const BodySplitMutationHooks& hooks)
{
    BodySplitPlateSettingsAcceptanceResult result;
    if (!body_split_editor_target_valid(target, current_plate_index, indexed_plate_id)) {
        result.acceptance = BodySplitPlateSettingsAcceptance::TargetInvalid;
        return result;
    }
    if (!body_split_editor_visible(effective_mode)) {
        result.acceptance = BodySplitPlateSettingsAcceptance::NotBodyMode;
        return result;
    }
    if (!transaction.has_pending()) {
        result.acceptance = BodySplitPlateSettingsAcceptance::NoPending;
        return result;
    }

    result.apply_result = transaction.commit(model, plate_config, resolver_config, hooks);
    result.acceptance = result.apply_result.applied ? BodySplitPlateSettingsAcceptance::Committed
                                                     : BodySplitPlateSettingsAcceptance::CommitRejected;
    return result;
}

} // namespace Slic3r::GUI
