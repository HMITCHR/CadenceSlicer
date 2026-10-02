#include "MixedNozzleWizardTransaction.hpp"

#include "libslic3r/Print.hpp"

#include <boost/log/trivial.hpp>

#include <algorithm>
#include <cmath>
#include <iterator>
#include <limits>
#include <memory>
#include <set>
#include <sstream>
#include <utility>

namespace Slic3r::GUI {

namespace {

// The project's own nozzle pair when it is usable: two different printable diameters.
const ConfigOptionFloats *saved_project_nozzle_pair(const DynamicPrintConfig &project)
{
    const auto *pair = project.option<ConfigOptionFloats>("nozzle_diameter");
    if (pair == nullptr || pair->values.size() != 2)
        return nullptr;
    const double first = pair->values[0];
    const double second = pair->values[1];
    if (!std::isfinite(first) || first <= 0. || !std::isfinite(second) || second <= 0. ||
        first == second)
        return nullptr;
    return pair;
}

} // namespace

DynamicPrintConfig wizard_plate_effective_config(const PresetBundle &bundle, const PartPlate &plate)
{
    DynamicPrintConfig config = bundle.full_config(true,
        plate.get_real_filament_maps(bundle.project_config),
        plate.get_real_filament_volume_maps(bundle.project_config));
    config.set_key_value("filament_map_mode", new ConfigOptionEnum<FilamentMapMode>(
        plate.get_real_filament_map_mode(bundle.project_config)));
    // construct_full_config drops the project's nozzle pair whenever the printer preset does not
    // itself describe two nozzles, leaving one nozzle and default layer-height limits. Put the
    // project's pair and its saved limits back; a project with no pair is left as composed.
    if (const ConfigOptionFloats *pair = saved_project_nozzle_pair(bundle.project_config)) {
        config.set_key_value("nozzle_diameter", new ConfigOptionFloats(pair->values));
        for (const char *key : {"min_layer_height", "max_layer_height"})
            if (const auto *limits = bundle.project_config.option<ConfigOptionFloats>(key);
                limits != nullptr && limits->values.size() == pair->values.size())
                config.set_key_value(key, new ConfigOptionFloats(limits->values));
    }
    return config;
}

namespace {

std::string serialized_option(const DynamicPrintConfig &config, const std::string &key)
{
    return config.has(key) ? config.opt_serialize(key) : "absent";
}

void set_project_mode(DynamicPrintConfig &config, MixedNozzleSlicingMode mode)
{
    config.set_key_value("mixed_nozzle_slicing_mode",
        new ConfigOptionEnum<MixedNozzleSlicingMode>(mode));
}

ModelObject *find_object(Model &model, std::size_t object_id)
{
    for (ModelObject *object : model.objects)
        if (object != nullptr && object->id().id == object_id)
            return object;
    return nullptr;
}

ModelVolume *find_volume(Model &model, std::size_t object_id, std::size_t volume_id)
{
    for (ModelObject *object : model.objects) {
        if (object == nullptr || object->id().id != object_id)
            continue;
        for (ModelVolume *volume : object->volumes)
            if (volume != nullptr && volume->id().id == volume_id)
                return volume;
        return nullptr;
    }
    return nullptr;
}

const WizardNativeVolumeState *find_volume_state(const std::vector<WizardNativeVolumeState> &volumes,
                                                 std::size_t object_id,
                                                 std::size_t volume_id)
{
    const auto it = std::find_if(volumes.begin(), volumes.end(),
        [object_id, volume_id](const WizardNativeVolumeState &volume) {
            return volume.object_id == object_id && volume.volume_id == volume_id;
        });
    return it == volumes.end() ? nullptr : &*it;
}

WizardNativeVolumeState capture_volume_state(const ModelObject &object, const ModelVolume &volume)
{
    WizardNativeVolumeState state;
    state.object_id = object.id().id;
    state.volume_id = volume.id().id;
    state.type = volume.type();
    state.mesh_identity = volume.get_mesh_shared_ptr().get();
    state.config_timestamp = static_cast<const ModelConfig &>(volume.config).timestamp();
    state.supported_facets_timestamp = volume.supported_facets.timestamp();
    state.seam_facets_timestamp = volume.seam_facets.timestamp();
    state.mmu_segmentation_facets_timestamp = volume.mmu_segmentation_facets.timestamp();
    state.fuzzy_skin_facets_timestamp = volume.fuzzy_skin_facets.timestamp();
    const auto matrix = volume.get_matrix();
    for (std::size_t index = 0; index < state.transform.size(); ++index)
        state.transform[index] = matrix.data()[index];
    return state;
}

WizardNativeObjectState capture_object_state(const ModelObject &object)
{
    WizardNativeObjectState state;
    state.object_id = object.id().id;
    state.config_timestamp = static_cast<const ModelConfig &>(object.config).timestamp();
    state.printable = object.printable;
    state.instances.reserve(object.instances.size());
    for (const ModelInstance *instance : object.instances) {
        if (instance == nullptr)
            continue;
        WizardNativeInstanceState instance_state;
        instance_state.instance_id = instance->id().id;
        instance_state.printable = instance->printable;
        instance_state.auto_drop = instance->auto_drop;
        instance_state.arrange_order = instance->arrange_order;
        instance_state.loaded_id = instance->loaded_id;
        instance_state.use_loaded_id_for_label = instance->use_loaded_id_for_label;
        instance_state.print_volume_state = instance->print_volume_state;
        const auto matrix = instance->get_matrix();
        for (std::size_t index = 0; index < instance_state.transform.size(); ++index)
            instance_state.transform[index] = matrix.data()[index];
        state.instances.push_back(instance_state);
    }
    return state;
}

bool capture_owner_state(const WizardNativeOwners &owners,
                         WizardNativeOwnerState &state,
                         std::string &diagnostic)
{
    if (owners.preset_bundle == nullptr || owners.model == nullptr ||
        owners.current_plate == nullptr || owners.plates.empty()) {
        diagnostic = "Mixed-Nozzle wizard native owners are incomplete.";
        return false;
    }

    std::set<const PartPlate *> unique_plates;
    bool current_found = false;
    for (PartPlate *plate : owners.plates) {
        if (plate == nullptr || !plate->id().valid() || !unique_plates.emplace(plate).second) {
            diagnostic = "Mixed-Nozzle wizard plate owners are invalid or duplicated.";
            return false;
        }
        if (plate == owners.current_plate)
            current_found = true;
    }
    if (!current_found || !owners.current_plate->id().valid()) {
        diagnostic = "Mixed-Nozzle wizard current plate is outside the owner scope.";
        return false;
    }

    state.preset_bundle = owners.preset_bundle;
    state.model = owners.model;
    state.current_plate = owners.current_plate;
    state.plates = owners.plates;
    state.preset_generation = owners.preset_bundle->mixed_nozzle_preset_generation;
    state.printer_name = owners.preset_bundle->printers.get_edited_preset().name;
    state.process_name = owners.preset_bundle->prints.get_edited_preset().name;
    state.project_config = owners.preset_bundle->project_config;
    state.printer_config = owners.preset_bundle->printers.get_edited_preset().config;
    state.process_config = owners.preset_bundle->prints.get_edited_preset().config;
    const std::vector<int> filament_maps = owners.current_plate->get_real_filament_maps(
        state.project_config);
    const std::vector<int> filament_volume_maps = owners.current_plate->get_real_filament_volume_maps(
        state.project_config);
    state.rebind_signature = owners.preset_bundle->mixed_nozzle_rebind_signature(
        filament_maps, filament_volume_maps);

    state.plate_ids.reserve(owners.plates.size());
    state.plate_indices.reserve(owners.plates.size());
    state.plate_locked.reserve(owners.plates.size());
    state.plate_configs.reserve(owners.plates.size());
    for (PartPlate *plate : owners.plates) {
        state.plate_ids.push_back(plate->id().id);
        // PartPlate::get_index() is non-const even though this is a read-only snapshot.
        state.plate_indices.push_back(const_cast<PartPlate *>(plate)->get_index());
        state.plate_locked.push_back(plate->is_locked());
        state.plate_configs.push_back(*plate->config());
        auto &members = state.plate_memberships.emplace_back();
        for (size_t object_index = 0; object_index < owners.model->objects.size(); ++object_index) {
            const ModelObject *object = owners.model->objects[object_index];
            if (object == nullptr)
                continue;
            for (size_t instance_index = 0; instance_index < object->instances.size(); ++instance_index)
                if (object->instances[instance_index] != nullptr &&
                    plate->contain_instance(int(object_index), int(instance_index)))
                    members.emplace_back(object->id().id, object->instances[instance_index]->id().id);
        }
    }

    for (const ModelObject *object : owners.model->objects) {
        if (object == nullptr)
            continue;
        state.objects.push_back(capture_object_state(*object));
        for (const ModelVolume *volume : object->volumes)
            if (volume != nullptr)
                state.volumes.push_back(capture_volume_state(*object, *volume));
    }
    return true;
}

bool same_volume_state(const WizardNativeVolumeState &lhs, const WizardNativeVolumeState &rhs)
{
    return lhs.object_id == rhs.object_id && lhs.volume_id == rhs.volume_id &&
           lhs.type == rhs.type && lhs.mesh_identity == rhs.mesh_identity &&
           lhs.config_timestamp == rhs.config_timestamp &&
           lhs.supported_facets_timestamp == rhs.supported_facets_timestamp &&
           lhs.seam_facets_timestamp == rhs.seam_facets_timestamp &&
           lhs.mmu_segmentation_facets_timestamp == rhs.mmu_segmentation_facets_timestamp &&
           lhs.fuzzy_skin_facets_timestamp == rhs.fuzzy_skin_facets_timestamp &&
           lhs.transform == rhs.transform;
}

bool same_object_state(const WizardNativeObjectState &lhs, const WizardNativeObjectState &rhs)
{
    if (lhs.object_id != rhs.object_id || lhs.config_timestamp != rhs.config_timestamp ||
        lhs.printable != rhs.printable || lhs.instances.size() != rhs.instances.size())
        return false;
    for (std::size_t index = 0; index < lhs.instances.size(); ++index)
        if (lhs.instances[index].instance_id != rhs.instances[index].instance_id ||
            lhs.instances[index].transform != rhs.instances[index].transform ||
            lhs.instances[index].printable != rhs.instances[index].printable ||
            lhs.instances[index].auto_drop != rhs.instances[index].auto_drop ||
            lhs.instances[index].arrange_order != rhs.instances[index].arrange_order ||
            lhs.instances[index].loaded_id != rhs.instances[index].loaded_id ||
            lhs.instances[index].use_loaded_id_for_label != rhs.instances[index].use_loaded_id_for_label ||
            lhs.instances[index].print_volume_state != rhs.instances[index].print_volume_state)
            return false;
    return true;
}

bool same_owner_state(const WizardNativeOwnerState &expected,
                      const WizardNativeOwnerState &actual)
{
    if (expected.preset_bundle != actual.preset_bundle || expected.model != actual.model ||
        expected.current_plate != actual.current_plate || expected.plates != actual.plates ||
        expected.preset_generation != actual.preset_generation ||
        expected.printer_name != actual.printer_name || expected.process_name != actual.process_name ||
        expected.rebind_signature != actual.rebind_signature ||
        expected.project_config != actual.project_config || expected.printer_config != actual.printer_config ||
        expected.process_config != actual.process_config || expected.plate_ids != actual.plate_ids ||
        expected.plate_indices != actual.plate_indices ||
        expected.plate_memberships != actual.plate_memberships ||
        expected.plate_locked != actual.plate_locked || expected.plate_configs != actual.plate_configs ||
        expected.objects.size() != actual.objects.size() || expected.volumes.size() != actual.volumes.size())
        return false;

    for (std::size_t index = 0; index < expected.objects.size(); ++index)
        if (!same_object_state(expected.objects[index], actual.objects[index]))
            return false;

    for (std::size_t index = 0; index < expected.volumes.size(); ++index)
        if (!same_volume_state(expected.volumes[index], actual.volumes[index]))
            return false;
    return true;
}

bool draft_targets_current_owners(const WizardDraft &draft,
                                  const WizardNativeOwnerState &state,
                                  std::string &diagnostic)
{
    if (!draft.signature.selected_printer_id.empty() &&
        draft.signature.selected_printer_id != state.printer_name) {
        diagnostic = "Mixed-Nozzle wizard printer selection is stale.";
        return false;
    }
    if (!draft.signature.selected_process_id.empty() &&
        draft.signature.selected_process_id != state.process_name) {
        diagnostic = "Mixed-Nozzle wizard process selection is stale.";
        return false;
    }
    if (!draft.signature.plate_ids.empty() && draft.signature.plate_ids != state.plate_ids) {
        diagnostic = "Mixed-Nozzle wizard plate scope is stale.";
        return false;
    }
    for (const WizardVolumeSignature &expected : draft.signature.volumes) {
        const WizardNativeVolumeState *actual = find_volume_state(
            state.volumes, expected.object_id, expected.volume_id);
        // A paint edit changes what each nozzle prints just as a settings edit does.
        if (actual == nullptr ||
            (expected.config_revision != 0 && expected.config_revision != actual->config_timestamp) ||
            (expected.paint_revision != 0 && expected.paint_revision != actual->mmu_segmentation_facets_timestamp)) {
            diagnostic = "Mixed-Nozzle wizard model target is stale.";
            return false;
        }
    }
    if (draft.scope.current_plate_id != 0) {
        const auto current = std::find(state.plates.begin(), state.plates.end(), state.current_plate);
        const bool exact_current = current != state.plates.end() &&
            state.plate_ids[std::size_t(current - state.plates.begin())] == draft.scope.current_plate_id;
        if (!exact_current) {
            diagnostic = "Mixed-Nozzle wizard current plate is stale or outside the active target.";
            return false;
        }
    }
    return true;
}

bool is_project_map_key(const std::string &key)
{
    return key == "filament_map" || key == "filament_map_mode" ||
           key == "filament_volume_map" || key == "filament_nozzle_map";
}

bool project_scope_covers_all_plates(const WizardDraft &draft,
                                     const WizardNativeOwnerState &before)
{
    if (!draft.scope.allow_project_binding_changes)
        return false;
    std::set<std::size_t> requested(draft.scope.process_affected_plate_ids.begin(),
                                    draft.scope.process_affected_plate_ids.end());
    const std::set<std::size_t> live(before.plate_ids.begin(), before.plate_ids.end());
    return requested == live;
}

bool native_binding_delta_matches(const WizardKeyDelta &delta,
                                  const WizardNativeOwnerState &before,
                                  const DynamicPrintConfig &after)
{
    if (delta.key != MIXED_NOZZLE_EXPLICIT_KEYS_OPTION &&
        delta.key != MIXED_NOZZLE_PROVENANCE_OPTION)
        return false;
    return serialized_option(before.project_config, delta.key) == delta.old_value &&
           serialized_option(after, delta.key) == delta.new_value;
}

bool apply_native_binding(const WizardDraft &draft,
                          const WizardNativeOwnerState &before,
                          DynamicPrintConfig &project,
                          std::string &diagnostic)
{
    const bool has_native_rows = std::any_of(draft.approved_key_deltas.begin(),
        draft.approved_key_deltas.end(), [](const WizardKeyDelta &delta) {
            return delta.source == "Native binding" || delta.scope == "Project";
        });
    if (!has_native_rows && !draft.native_binding)
        return true;
    if (!draft.native_binding || !project_scope_covers_all_plates(draft, before)) {
        diagnostic = "Mixed-Nozzle wizard native binding requires an explicit project-wide review.";
        return false;
    }
    const WizardNativeBindingApproval &approval = *draft.native_binding;
    if (!approval.plan.offered || approval.plan.num_filaments == 0 ||
        approval.accepted.size() != approval.plan.entries.size() ||
        approval.plan.signature != before.rebind_signature) {
        diagnostic = "Mixed-Nozzle wizard native binding review is stale or incomplete.";
        return false;
    }
    DynamicPrintConfig after = project;
    if (!mixed_nozzle_rebind_write(after, approval.plan, approval.accepted)) {
        diagnostic = "Mixed-Nozzle wizard native binding could not be staged.";
        return false;
    }
    for (const WizardKeyDelta &delta : draft.approved_key_deltas)
        if (delta.source == "Native binding" &&
            (delta.scope != "Project" || !native_binding_delta_matches(delta, before, after))) {
            diagnostic = "Mixed-Nozzle wizard native binding row is outside the accepted plan.";
            return false;
        }
    project = std::move(after);
    return true;
}

bool apply_process_deltas(const WizardDraft &draft,
                          const WizardNativeOwnerState &before,
                          DynamicPrintConfig &process,
                          std::string &diagnostic)
{
    std::set<std::string> seen_keys;
    for (const WizardKeyDelta &delta : draft.approved_key_deltas) {
        if (delta.source == "Native binding") {
            if (delta.scope != "Project") {
                diagnostic = "Mixed-Nozzle wizard native binding delta has an invalid project scope.";
                return false;
            }
            continue;
        }
        if (delta.scope == "Project" || delta.source != "Resolver" || delta.scope != "Shared process") {
            diagnostic = "Mixed-Nozzle wizard process delta has a stale source or scope.";
            return false;
        }
        if (is_project_map_key(delta.key)) {
            diagnostic = "Mixed-Nozzle wizard filament mapping requires native binding staging.";
            return false;
        }
        if (draft.scope.kind == WizardScopeKind::CurrentPlate &&
            !draft.scope.allow_shared_process_changes) {
            diagnostic = "Mixed-Nozzle wizard shared process changes require explicit scope approval.";
            return false;
        }
        if (delta.key.empty() || draft.locked_keys.count(delta.key) != 0) {
            diagnostic = "Mixed-Nozzle wizard process delta is locked.";
            return false;
        }
        if (!seen_keys.emplace(delta.key).second) {
            diagnostic = "Mixed-Nozzle wizard review contains duplicate process deltas.";
            return false;
        }
        DynamicPrintConfig &destination = process;
        const DynamicPrintConfig &baseline = before.process_config;
        if (serialized_option(baseline, delta.key) != delta.old_value) {
            diagnostic = "Mixed-Nozzle wizard process delta is stale.";
            return false;
        }
        try {
            ConfigSubstitutionContext substitutions{ForwardCompatibilitySubstitutionRule::Disable};
            destination.set_deserialize(delta.key, delta.new_value, substitutions);
        } catch (const std::exception &) {
            diagnostic = "Mixed-Nozzle wizard process delta is invalid.";
            return false;
        }
    }
    return true;
}

bool body_review_entry_is_staged(const WizardKeyDelta &delta, const WizardDraft &draft)
{
    if (delta.source != "Body editor" || delta.scope != "Current plate")
        return false;
    return std::any_of(draft.staged_body_edits.begin(), draft.staged_body_edits.end(),
        [&delta](const WizardBodyEdit &edit) {
            const auto value = edit.staged_values.find(delta.key);
            return value != edit.staged_values.end() && value->second == delta.new_value;
        });
}

bool body_targets_current_plate(const WizardNativeOwners &owners,
                                const WizardDraft &draft,
                                std::string &diagnostic)
{
    if (draft.staged_body_edits.empty())
        return true;
    const ModelObjectPtrs objects = owners.current_plate->get_objects_on_this_plate();
    for (const WizardBodyEdit &edit : draft.staged_body_edits) {
        const bool on_current_plate = std::any_of(objects.begin(), objects.end(),
            [&edit](const ModelObject *object) {
                return object != nullptr && object->id().id == edit.object_id;
            });
        if (!on_current_plate) {
            diagnostic = "Mixed-Nozzle wizard Body target is outside the current plate.";
            return false;
        }
    }
    return true;
}

// Only process deltas belong in the process preset; Body editor entries are evidence for the staged
// Body edits. Unknown domains are rejected so a new editor cannot be silently ignored.
bool route_review_entries(const WizardDraft &draft, const WizardReview &review,
                          WizardDraft &reviewed_draft, std::string &diagnostic)
{
    reviewed_draft.approved_key_deltas.clear();
    for (const WizardKeyDelta &delta : review.entries) {
        if (delta.source == "Native binding" && delta.scope == "Project" &&
            (delta.key == MIXED_NOZZLE_EXPLICIT_KEYS_OPTION ||
             delta.key == MIXED_NOZZLE_PROVENANCE_OPTION)) {
            reviewed_draft.approved_key_deltas.push_back(delta);
            continue;
        }
        if (delta.source == "Resolver" && delta.scope == "Shared process") {
            reviewed_draft.approved_key_deltas.push_back(delta);
            continue;
        }
        if (body_review_entry_is_staged(delta, draft))
            continue;
        diagnostic = "Mixed-Nozzle wizard review contains an unsupported or stale delta domain.";
        return false;
    }

    // The review may narrow the current-plate set but cannot widen or replace the draft's scope.
    if (!review.affected_plate_ids.empty() && !draft.scope.process_affected_plate_ids.empty()) {
        for (const std::size_t plate_id : review.affected_plate_ids) {
            if (std::find(draft.scope.process_affected_plate_ids.begin(),
                          draft.scope.process_affected_plate_ids.end(), plate_id) ==
                draft.scope.process_affected_plate_ids.end()) {
                diagnostic = "Mixed-Nozzle wizard review plate scope is stale.";
                return false;
            }
        }
    }
    return true;
}

bool role_bindings_match_effective_config(const WizardDraft &draft,
                                          const DynamicPrintConfig &effective,
                                          std::string &diagnostic)
{
    if (draft.resolved_physical_roles.empty())
        return true;
    // Read after composition: the process preset does not own plate-local maps.
    const auto *filament_map = effective.option<ConfigOptionInts>("filament_map");
    for (const WizardPhysicalRole &role : draft.resolved_physical_roles) {
        if (filament_map == nullptr || role.logical_filament >= filament_map->values.size() ||
            filament_map->values[role.logical_filament] != int(role.physical_extruder + 1)) {
            diagnostic = "Mixed-Nozzle wizard material-to-tool binding was not staged.";
            return false;
        }
    }
    return true;
}

bool unsupported_conversion_intent(const WizardDraft &draft, std::string &diagnostic)
{
    if (draft.selected_candidate_kind &&
        *draft.selected_candidate_kind == WizardCandidateKind::SingleNozzle) {
        diagnostic = "Mixed-Nozzle wizard single-nozzle conversion requires explicit setup staging.";
        return true;
    }
    if (draft.mode == MixedNozzleSlicingMode::Off &&
        (!draft.approved_key_deltas.empty() || !draft.staged_body_edits.empty())) {
        diagnostic = "Mixed-Nozzle wizard Off conversion requires explicit setup staging.";
        return true;
    }
    return false;
}

bool is_cadence_height_key(const std::string &key)
{
    return key == "layer_height" || key == "mixed_nozzle_coarse_layer_height";
}

// Catalogue rows carry the preset's setting_id when it has one and its name otherwise.
const Preset *find_catalogue_process_preset(const PresetCollection &prints, const std::string &id)
{
    for (const Preset &preset : prints.get_presets())
        if ((preset.setting_id.empty() ? preset.name : preset.setting_id) == id)
            return &preset;
    return nullptr;
}

// mixed_nozzle_effective_nozzle_diameters() defers to a printer preset that does not describe two
// nozzles, which is right for the engine but not here: the wizard sets the project's pair up.
std::vector<double> wizard_nozzle_pair(const DynamicPrintConfig &project,
                                       const DynamicPrintConfig &printer)
{
    if (const ConfigOptionFloats *pair = saved_project_nozzle_pair(project))
        return pair->values;
    const auto *effective = mixed_nozzle_effective_nozzle_diameters(project, printer);
    return effective == nullptr ? std::vector<double>{} : effective->values;
}

// A process preset naming the project's pair may be selected although the current printer variant
// marks it incompatible; the printer switch below makes it compatible again.
bool process_declares_pair(const DynamicPrintConfig &process, const std::vector<double> &pair)
{
    if (pair.size() != 2)
        return false;
    const auto *roles = process.option<ConfigOptionFloats>("mixed_nozzle_process_nozzle_diameters");
    if (roles == nullptr || roles->values.size() != 2)
        return false;
    return (is_approx(roles->values[0], pair[0]) && is_approx(roles->values[1], pair[1])) ||
           (is_approx(roles->values[0], pair[1]) && is_approx(roles->values[1], pair[0]));
}

// Every installed printer preset, reduced to what picking a variant for the fine nozzle needs.
std::vector<WizardPrinterVariantRow> printer_variant_rows(const PresetCollection &printers)
{
    std::vector<WizardPrinterVariantRow> rows;
    for (const Preset &preset : printers.get_presets()) {
        if (preset.is_default || preset.name.empty())
            continue;
        WizardPrinterVariantRow row;
        row.name = preset.name;
        if (preset.config.has("printer_model"))
            row.printer_model = preset.config.opt_string("printer_model");
        if (preset.config.has("printer_variant"))
            row.printer_variant = preset.config.opt_string("printer_variant");
        row.visible = preset.is_visible;
        row.system = preset.is_system;
        rows.push_back(std::move(row));
    }
    return rows;
}

// The machine limits stated for each nozzle diameter, so a limit that belongs to another nozzle can
// be replaced rather than guessed.
std::vector<WizardNozzleLimitRow> installed_nozzle_limits(const PresetCollection &printers,
                                                          const std::string &printer_model)
{
    std::vector<WizardNozzleLimitRow> rows;
    for (const Preset &preset : printers.get_presets()) {
        if (preset.is_default)
            continue;
        if (!printer_model.empty() && (!preset.config.has("printer_model") ||
                                       preset.config.opt_string("printer_model") != printer_model))
            continue;
        const auto *nozzles = preset.config.option<ConfigOptionFloats>("nozzle_diameter");
        const auto *minima = preset.config.option<ConfigOptionFloats>("min_layer_height");
        const auto *maxima = preset.config.option<ConfigOptionFloats>("max_layer_height");
        if (nozzles == nullptr || minima == nullptr || maxima == nullptr)
            continue;
        for (std::size_t tool = 0; tool < nozzles->values.size(); ++tool) {
            if (tool >= minima->values.size() || tool >= maxima->values.size())
                break;
            const double diameter = nozzles->values[tool];
            if (!std::isfinite(diameter) || diameter <= 0.)
                continue;
            if (std::any_of(rows.begin(), rows.end(), [diameter](const WizardNozzleLimitRow &row) {
                    return is_approx(row.diameter, diameter); }))
                continue;
            rows.push_back({diameter, minima->values[tool], maxima->values[tool]});
        }
    }
    return rows;
}

// The printer preset's own values say which nozzle each limit belongs to; the composed pair cannot.
std::vector<double> preset_nozzle_values(const DynamicPrintConfig &printer, const char *key)
{
    const auto *value = printer.option<ConfigOptionFloats>(key);
    return value == nullptr ? std::vector<double>{} : value->values;
}

// The layering construct_full_config applies to the project-owned nozzle options.
std::vector<double> composed_nozzle_values(const DynamicPrintConfig &project,
                                           const DynamicPrintConfig &printer, const char *key)
{
    if (const auto *value = project.option<ConfigOptionFloats>(key);
        value != nullptr && !value->values.empty())
        return value->values;
    if (const auto *value = printer.option<ConfigOptionFloats>(key))
        return value->values;
    return {};
}

// Only a catalogue row may redirect to another process preset, never an equal height pair. The row
// must name an installed, visible, compatible preset with exactly the chosen cadence, and the
// review must carry no override that preset cannot hold, since selecting it replaces the process.
std::optional<std::string> wizard_stock_process_selection(const WizardDraft &draft,
                                                          const WizardNativeOwnerState &before,
                                                          const PresetBundle &bundle,
                                                          const std::vector<double> &project_pair)
{
    if (!draft.selected_source_preset_id || draft.selected_source_preset_id->empty() ||
        !draft.chosen_coarse_height)
        return std::nullopt;
    if (draft.selected_candidate_kind &&
        *draft.selected_candidate_kind != WizardCandidateKind::MixedCadence)
        return std::nullopt;
    const Preset *preset = find_catalogue_process_preset(
        bundle.prints, *draft.selected_source_preset_id);
    if (preset == nullptr || !preset->is_visible || preset->name == before.process_name ||
        (!preset->is_compatible && !process_declares_pair(preset->config, project_pair)))
        return std::nullopt;
    const auto *fine = preset->config.option<ConfigOptionFloat>("layer_height");
    const auto *coarse = preset->config.option<ConfigOptionFloat>("mixed_nozzle_coarse_layer_height");
    if (fine == nullptr || coarse == nullptr ||
        !is_approx(fine->value, draft.chosen_fine_height) ||
        !is_approx(coarse->value, *draft.chosen_coarse_height))
        return std::nullopt;
    for (const WizardKeyDelta &delta : draft.approved_key_deltas)
        if (delta.source == "Resolver" && delta.scope == "Shared process" &&
            !is_cadence_height_key(delta.key))
            return std::nullopt;
    return preset->name;
}

MixedNozzleSetupRequest setup_request_for(const WizardDraft &draft)
{
    MixedNozzleSetupRequest request;
    request.commit = draft.scope.kind == WizardScopeKind::ProjectDefault
        ? MixedNozzleSetupCommit::SaveProjectDefault
        : MixedNozzleSetupCommit::ApplyCurrentPlate;
    request.use_project_default = false;
    request.mode = draft.mode;
    // The controller refuses a pair outside project scope or without both envelopes, so a nozzle
    // change cannot half-apply.
    request.nozzle_diameters = draft.requested_nozzle_diameters;
    request.min_layer_heights = draft.requested_min_layer_heights;
    request.max_layer_heights = draft.requested_max_layer_heights;
    request.resolved_printer_name = draft.requested_printer_name;
    request.resolve_default_process = draft.resolve_default_process;
    switch (draft.tower_intent) {
    case WizardTowerIntent::Automatic:
        request.tower_policy = MixedNozzleTowerPolicyRequest{
            draft.mode, MixedNozzleTowerChoice::Automatic,
            MixedNozzleTowerProvenance::DeliberateAutomatic};
        request.resolve_tower_usage = true;
        break;
    case WizardTowerIntent::Enabled:
        request.tower_policy = MixedNozzleTowerPolicyRequest{
            draft.mode, MixedNozzleTowerChoice::Enabled,
            MixedNozzleTowerProvenance::DeliberateAutomatic};
        request.resolve_tower_usage = true;
        break;
    case WizardTowerIntent::Disabled:
        request.tower_policy = MixedNozzleTowerPolicyRequest{
            draft.mode, MixedNozzleTowerChoice::Disabled,
            MixedNozzleTowerProvenance::DeliberateAutomatic};
        request.resolve_tower_usage = true;
        break;
    case WizardTowerIntent::Preserve:
        break;
    }
    return request;
}

bool apply_body_edits(Model &staged_model,
                      const WizardDraft &draft,
                      std::string &diagnostic)
{
    std::set<std::pair<std::size_t, std::size_t>> seen;
    for (const WizardBodyEdit &edit : draft.staged_body_edits) {
        if (!seen.emplace(edit.object_id, edit.volume_id).second) {
            diagnostic = "Mixed-Nozzle wizard Body rows contain a duplicate target.";
            return false;
        }
        ModelVolume *volume = find_volume(staged_model, edit.object_id, edit.volume_id);
        if (volume == nullptr || volume->type() != ModelVolumeType::MODEL_PART) {
            diagnostic = "Mixed-Nozzle wizard Body target is unavailable.";
            return false;
        }
        for (const auto &[key, value] : edit.staged_values) {
            if (key != "extruder" && key != "regional_layer_height" &&
                key != "mixed_nozzle_body_fine_skins" &&
                key != "mixed_nozzle_body_fine_skin_layers") {
                diagnostic = "Mixed-Nozzle wizard Body value is unsupported.";
                return false;
            }
            if (draft.locked_keys.count(key) != 0) {
                diagnostic = "Mixed-Nozzle wizard Body value is locked.";
                return false;
            }
            try {
                ConfigSubstitutionContext substitutions{ForwardCompatibilitySubstitutionRule::Disable};
                volume->config.set_deserialize(key, value, substitutions);
            } catch (const std::exception &) {
                diagnostic = "Mixed-Nozzle wizard Body value is invalid.";
                return false;
            }
        }
    }
    return true;
}

// Joining is one choice per assembly: an object's settings are shared by all its copies, and the
// review says when it reaches copies elsewhere. An assembly with no choice takes the default, beam
// interlocking on for bodies that touch across the two nozzles and current settings otherwise. Only
// an applied setup takes it, so reopening a project changes nothing.
bool stage_body_joining_and_review(Model &staged_model, const WizardDraft &draft,
                                   const WizardNativeOwners &owners,
                                   const DynamicPrintConfig &full_config,
                                   const BodySplitEditorModel &editor,
                                   PreparedWizardApply &prepared)
{
    std::set<std::size_t> current_objects;
    for (const ModelObject *object : owners.current_plate->get_objects_on_this_plate())
        if (object != nullptr)
            current_objects.insert(object->id().id);
    const std::array<const std::set<std::size_t> *, 3> choices{
        &draft.enable_interlocking_objects, &draft.disable_interlocking_objects, &draft.keep_joining_objects};
    for (std::size_t which = 0; which < choices.size(); ++which)
        for (const auto id : *choices[which]) {
            if (current_objects.count(id) == 0) {
                prepared.diagnostic = "The assembly selected for joining is outside the current plate.";
                return false;
            }
            for (std::size_t other = which + 1; other < choices.size(); ++other)
                if (choices[other]->count(id) != 0) {
                    prepared.diagnostic = "An assembly has more than one joining choice.";
                    return false;
                }
        }
    const bool joining_locked = draft.locked_keys.count("interlocking_beam") != 0 ||
                                draft.locked_keys.count("interlocking_beam_layer_count") != 0;
    if (joining_locked && (!draft.enable_interlocking_objects.empty() || !draft.disable_interlocking_objects.empty())) {
        prepared.diagnostic = "The assembly's beam interlocking settings are locked.";
        return false;
    }
    const std::set<std::size_t> touching = wizard_touching_assemblies(staged_model.objects, editor.rows);
    bool joins = false;
    for (const auto id : current_objects) {
        ModelObject *object = find_object(staged_model, id);
        if (object == nullptr) {
            prepared.diagnostic = "The assembly selected for joining is no longer available.";
            return false;
        }
        // The staged body's effective cadence is authoritative, including object overrides.
        const BodySplitBeamLayers beam = body_split_beam_layers(editor.rows, object->id());
        const bool has_bodies = beam.has_bodies;
        const bool enable_chosen = draft.enable_interlocking_objects.count(id) != 0;
        const bool disable = draft.disable_interlocking_objects.count(id) != 0;
        if (!has_bodies) {
            // A single-part object has no second body to join to.
            if (enable_chosen || disable) {
                prepared.diagnostic = object->name + " has one part, so there is nothing to join.";
                return false;
            }
            continue;
        }
        const bool chosen = enable_chosen || disable || draft.keep_joining_objects.count(id) != 0;
        const bool by_default = !chosen && !joining_locked && touching.count(id) != 0;
        const bool enable = enable_chosen || by_default;

        DynamicPrintConfig effective = full_config;
        effective.apply(object->config.get(), true);
        const std::string source = object->config.get().has("interlocking_beam")
            ? "object setting" : "inherited setting";
        const bool before_enabled = effective.opt_bool("interlocking_beam");
        const int before_layers = effective.opt_int("interlocking_beam_layer_count");
        const bool after_enabled = enable || (before_enabled && !disable);
        if (after_enabled && !beam.layers) {
            prepared.diagnostic = "Beam interlocking needs each coarse layer to be a whole number of fine layers.";
            return false;
        }
        const int layers = after_enabled ? *beam.layers : 0;
        if (enable) {
            object->config.set_key_value("interlocking_beam", new ConfigOptionBool(true));
            object->config.set_key_value("interlocking_beam_layer_count", new ConfigOptionInt(layers));
        } else if (disable) {
            object->config.set_key_value("interlocking_beam", new ConfigOptionBool(false));
        } else if (before_enabled && before_layers != layers) {
            // The code goes to the log; Review shows the sentence and the setting to open.
            const std::string sentence = object->name + ": beam interlocking uses " +
                std::to_string(before_layers) + " beam layers, but these layer heights need " + std::to_string(layers) +
                ". Under Joining, choose interlocking beams for this assembly to match, or keep its current layer heights.";
            BOOST_LOG_TRIVIAL(info) << "Mixed nozzle setup review: engine refusal: [" << MIXED_NOZZLE_BEAM_LAYER_MISMATCH_CODE
                                    << "] " << sentence;
            prepared.admission_refusal_key = "interlocking_beam_layer_count";
            prepared.admission_refusal = sentence;
            prepared.diagnostic = sentence;
            return false;
        }
        std::string note = object->name + ": beam interlocking " + (before_enabled ? "on" : "off") +
            " (" + source + ")";
        if (enable) {
            note += " to on, beam layers " + std::to_string(before_layers) + " to " + std::to_string(layers) +
                    " (this assembly only)";
            if (by_default)
                note += ". Its bodies touch, so beam interlocking is on by default";
        } else if (disable) {
            note += " to off (this assembly only, for a part meant to move)";
        } else {
            note += " kept, beam layers " + std::to_string(before_layers);
        }
        note += ". Width " + effective.opt_serialize("interlocking_beam_width") + " mm, depth " +
                effective.opt_serialize("interlocking_depth") + ", orientation " +
                effective.opt_serialize("interlocking_orientation") + " degrees kept.";
        prepared.review_notes.push_back(std::move(note));
        joins = joins || after_enabled;

        // Copies share the object's settings: say so and re-slice every plate holding one.
        std::vector<std::size_t> other_plates;
        for (PartPlate *plate : owners.plates) {
            if (plate == nullptr || plate == owners.current_plate)
                continue;
            const auto objects = plate->get_objects_on_this_plate();
            if (std::any_of(objects.begin(), objects.end(), [id](const ModelObject *candidate) {
                    return candidate != nullptr && candidate->id().id == id;
                }))
                other_plates.push_back(plate->id().id);
        }
        if (object->instances.size() > 1 || !other_plates.empty()) {
            prepared.review_notes.push_back(object->name + ": this joining setting is shared by all " +
                std::to_string(object->instances.size()) + " copies of the object" +
                (other_plates.empty() ? std::string(".") : std::string(", including copies on other plates.")));
            if (enable || disable)
                for (const std::size_t plate_id : other_plates)
                    if (std::find(prepared.affected_plate_ids.begin(), prepared.affected_plate_ids.end(), plate_id) ==
                        prepared.affected_plate_ids.end())
                        prepared.affected_plate_ids.push_back(plate_id);
        }

    }
    if (joins)
        prepared.review_notes.push_back("Beam interlocking is generated only where bodies on the two nozzles touch. "
            "Use it where parts should bond. Moving joints are not detected automatically: if a part is meant to "
            "move, choose Off for its assembly under Joining. Beams do not bridge gaps or guarantee bond "
            "strength, so check the sliced joint.");
    return true;
}

std::size_t logical_filament_count(const WizardDraft &draft)
{
    std::size_t count = 0;
    for (const WizardPhysicalRole &role : draft.resolved_physical_roles)
        count = std::max(count, role.logical_filament + 1);
    if (draft.fine_logical_filament)
        count = std::max(count, *draft.fine_logical_filament + 1);
    if (draft.coarse_logical_filament)
        count = std::max(count, *draft.coarse_logical_filament + 1);
    return std::max<std::size_t>(count, 2);
}

PartPlate *find_plate(const WizardNativeOwners &owners, std::size_t plate_id)
{
    const auto it = std::find_if(owners.plates.begin(), owners.plates.end(),
        [plate_id](const PartPlate *plate) { return plate != nullptr && plate->id().id == plate_id; });
    return it == owners.plates.end() ? nullptr : *it;
}

bool add_plate_deltas(const WizardDraft &draft,
                      const WizardNativeOwners &owners,
                      const MixedNozzleSetupRequest &setup_request,
                      const std::vector<double> &nozzle_diameters,
                      std::size_t logical_count,
                      PreparedWizardApply &prepared)
{
    std::vector<std::size_t> ids = draft.scope.process_affected_plate_ids;
    if (ids.empty())
        ids.push_back(owners.current_plate->id().id);
    std::set<std::size_t> seen;
    for (std::size_t plate_id : ids) {
        if (!seen.emplace(plate_id).second)
            continue;
        PartPlate *plate = find_plate(owners, plate_id);
        if (plate == nullptr) {
            prepared.diagnostic = "Mixed-Nozzle wizard affected plate is unavailable.";
            return false;
        }
        if (plate->is_locked()) {
            prepared.diagnostic = "Mixed-Nozzle wizard affected plate is locked.";
            return false;
        }
        DynamicPrintConfig after = *plate->config();
        // The wizard writes the plate filament map itself, so an automatic map never needs the
        // grouping dialog first. Slots the wizard does not own keep their mapping.
        if (draft.derived_map_mode == fmmManual) {
            const std::vector<int> current = plate->get_real_filament_maps(prepared.before.project_config);
            mixed_nozzle_write_manual_filament_map(after, wizard_derived_filament_map(
                nozzle_diameters, current, std::max(logical_count, current.size()),
                draft.fine_logical_filament, draft.coarse_logical_filament));
        }
        const MixedNozzleSlicingMode before_mode = effective_mixed_nozzle_mode(
            prepared.before.project_config, *plate).effective;
        const MixedNozzleSlicingMode after_mode = proposed_mixed_nozzle_plate_mode(
            prepared.project_config, *plate, setup_request, owners.current_plate);
        const bool apply_plate_mode = setup_request.commit == MixedNozzleSetupCommit::ApplyCurrentPlate &&
            plate == owners.current_plate;
        if (apply_plate_mode) {
            set_project_mode(after, after_mode);
        }
        if (before_mode == MixedNozzleSlicingMode::FeatureSplit && after_mode != before_mode) {
            for (const char *key : {"outer_wall_filament_id", "inner_wall_filament_id",
                                    "internal_solid_filament_id", "top_surface_filament_id",
                                    "bottom_surface_filament_id", "sparse_infill_filament_id"})
                after.erase(key);
        }
        if (after != *plate->config())
            prepared.plate_deltas.push_back({plate, plate_id, std::move(after)});
        prepared.affected_plate_ids.push_back(plate_id);
    }
    return true;
}

bool add_feature_delta(const FeatureSplitApplyRequest &request,
                       const WizardNativeOwners &owners,
                       PreparedWizardApply &prepared)
{
    auto it = std::find_if(prepared.plate_deltas.begin(), prepared.plate_deltas.end(),
        [&request](const WizardPlateConfigDelta &delta) {
            return delta.owner != nullptr && delta.plate_id == request.target.plate_id;
        });
    if (it == prepared.plate_deltas.end()) {
        PartPlate *plate = find_plate(owners, request.target.plate_id);
        if (plate == nullptr || plate != owners.current_plate)
            return false;
        prepared.plate_deltas.push_back({plate, request.target.plate_id, *plate->config()});
        it = std::prev(prepared.plate_deltas.end());
    }
    for (const char *key : {"outer_wall_filament_id", "inner_wall_filament_id",
                            "internal_solid_filament_id", "top_surface_filament_id",
                            "bottom_surface_filament_id"})
        it->config.set_key_value(key, new ConfigOptionInt(request.fine_filament));
    it->config.set_key_value("sparse_infill_filament_id", new ConfigOptionInt(request.coarse_filament));
    return true;
}

bool add_model_deltas(Model &before,
                      const Model &after,
                      PreparedWizardApply &prepared)
{
    for (const ModelObject *after_object : after.objects) {
        if (after_object == nullptr)
            continue;
        ModelObject *before_object = nullptr;
        for (ModelObject *candidate : before.objects)
            if (candidate != nullptr && candidate->id() == after_object->id()) {
                before_object = candidate;
                break;
            }
        if (before_object == nullptr)
            return false;
        if (after_object->config.get() != before_object->config.get())
            prepared.object_deltas.push_back({before_object, after_object->id().id, after_object->config});
        for (const ModelVolume *after_volume : after_object->volumes) {
            if (after_volume == nullptr)
                continue;
            ModelVolume *before_volume = nullptr;
            for (ModelVolume *candidate : before_object->volumes)
                if (candidate != nullptr && candidate->id() == after_volume->id()) {
                    before_volume = candidate;
                    break;
                }
            if (before_volume == nullptr)
                return false;
            if (after_volume->config.get() != before_volume->config.get()) {
                WizardModelConfigDelta delta;
                delta.owner = before_volume;
                delta.object_id = after_object->id().id;
                delta.volume_id = after_volume->id().id;
                delta.config = after_volume->config;
                prepared.model_deltas.push_back(std::move(delta));
            }
        }
    }
    return true;
}

struct FinalTowerPlateContext {
    std::vector<std::unique_ptr<PartPlate>> owned;
    std::vector<PartPlate *> views;
    PartPlate *current {nullptr};
};

bool build_final_tower_plate_context(const WizardNativeOwners &owners,
                                     const std::vector<WizardPlateConfigDelta> &plate_deltas,
                                     Model &staged_model,
                                     FinalTowerPlateContext &context,
                                     std::string &diagnostic)
{
    context.owned.reserve(owners.plates.size());
    context.views.reserve(owners.plates.size());
    for (PartPlate *live_plate : owners.plates) {
        if (live_plate == nullptr) {
            diagnostic = "Mixed-Nozzle wizard final tower plate is unavailable.";
            return false;
        }
        const auto delta = std::find_if(plate_deltas.begin(), plate_deltas.end(),
            [live_plate](const WizardPlateConfigDelta &candidate) {
                return candidate.owner == live_plate;
            });
        auto staged_plate = std::make_unique<PartPlate>(
            nullptr, live_plate->get_origin(), int(live_plate->get_size().x()),
            int(live_plate->get_size().y()), 200, nullptr, &staged_model);
        staged_plate->set_index(live_plate->get_index());
        staged_plate->lock(live_plate->is_locked());
        *staged_plate->config() = delta == plate_deltas.end()
            ? *live_plate->config() : delta->config;

        for (std::size_t object_idx = 0; object_idx < owners.model->objects.size(); ++object_idx) {
            const ModelObject *live_object = owners.model->objects[object_idx];
            if (live_object == nullptr)
                continue;
            if (object_idx >= staged_model.objects.size() || staged_model.objects[object_idx] == nullptr) {
                diagnostic = "Mixed-Nozzle wizard final tower model is unavailable.";
                return false;
            }
            for (std::size_t instance_idx = 0; instance_idx < live_object->instances.size(); ++instance_idx) {
                if (live_plate->contain_instance(int(object_idx), int(instance_idx)) &&
                    staged_plate->add_instance(int(object_idx), int(instance_idx), false) != 0) {
                    diagnostic = "Mixed-Nozzle wizard final tower plate membership is stale.";
                    return false;
                }
            }
        }
        if (live_plate == owners.current_plate)
            context.current = staged_plate.get();
        context.views.push_back(staged_plate.get());
        context.owned.push_back(std::move(staged_plate));
    }
    if (context.current == nullptr) {
        diagnostic = "Mixed-Nozzle wizard final tower current plate is unavailable.";
        return false;
    }
    return true;
}

// The MN process presets for the Body preset choice. Selectable means visible and compatible with
// the pair as currently composed.
std::vector<WizardProcessPresetRow> wizard_process_rows(const PresetCollection &prints)
{
    std::vector<WizardProcessPresetRow> rows;
    for (const Preset &preset : prints.get_presets()) {
        if (preset.is_default ||
            wizard_process_family(preset.name, preset.inherits()) == WizardProcessFamily::Other)
            continue;
        WizardProcessPresetRow row;
        row.name = preset.name;
        row.inherits = preset.inherits();
        if (const auto *fine = preset.config.option<ConfigOptionFloat>("layer_height"))
            row.fine_height = fine->value;
        if (const auto *coarse = preset.config.option<ConfigOptionFloat>("mixed_nozzle_coarse_layer_height"))
            row.coarse_height = coarse->value;
        row.selectable = preset.is_visible && preset.is_compatible;
        rows.push_back(std::move(row));
    }
    return rows;
}

// The rows the process basis reads. A preset naming the project's pair stays on offer while the
// printer variant marks it incompatible; the printer switch makes it compatible again.
std::vector<WizardProcessPresetRow> wizard_basis_rows(const PresetCollection &prints,
                                                      const std::vector<double> &project_pair)
{
    std::vector<WizardProcessPresetRow> rows = wizard_process_rows(prints);
    for (WizardProcessPresetRow &row : rows) {
        if (row.selectable)
            continue;
        for (const Preset &preset : prints.get_presets())
            if (preset.name == row.name) {
                row.selectable = preset.is_visible && process_declares_pair(preset.config, project_pair);
                break;
            }
    }
    return rows;
}

// The Body editor's refusal in plain words. The editor decides; this only reports it.
std::string body_editor_sentence(BodySplitEditorDiagnostic diagnostic)
{
    switch (diagnostic) {
    case BodySplitEditorDiagnostic::None:
        return {};
    case BodySplitEditorDiagnostic::ExactlyTwoModelPartsRequired:
        return "Body Split needs at least two parts in each object, or one part with painted regions.";
    case BodySplitEditorDiagnostic::LayerHeightInvalid:
        return "A body has no usable layer height. Check the layer height in its object settings.";
    case BodySplitEditorDiagnostic::LogicalFilamentInvalid:
        return "Every body needs a material. Pick one for each body on the Materials page.";
    case BodySplitEditorDiagnostic::DistinctLogicalFilamentsRequired:
        return "The bodies must use both nozzles.";
    case BodySplitEditorDiagnostic::PhysicalToolUnresolved:
        return "A body's material is not assigned to a nozzle. Set Filament grouping to Custom and put each material on a nozzle.";
    case BodySplitEditorDiagnostic::DistinctPhysicalToolsRequired:
        return "The bodies must use both nozzles.";
    case BodySplitEditorDiagnostic::PhysicalNozzleOrderingInvalid:
        return "The coarse bodies must be on the larger nozzle.";
    case BodySplitEditorDiagnostic::CadenceNotQualified:
        return "The layer heights do not fit. Each object needs one fine height and one coarse height that is a "
               "whole multiple of it, inside both nozzles' limits.";
    case BodySplitEditorDiagnostic::StaleTarget:
    case BodySplitEditorDiagnostic::DuplicateTarget:
        return "The bodies changed while setup was open. Close setup and open it again.";
    case BodySplitEditorDiagnostic::PaintedFilamentPhysicalToolMismatch:
        return "A painted region cannot be printed as painted: its colour has no nozzle and layer height Body "
               "Split can use there. Repaint it with the Color Painting tool, or change which nozzle that colour "
               "prints on in Filament grouping.";
    }
    return "Body Split cannot use these bodies as they are assigned.";
}

// Two or more model parts, or a painted part. Whether an ordinary part may share the plate is the
// engine's answer.
bool is_body_object(const ModelObject &object)
{
    const auto parts = std::count_if(object.volumes.begin(), object.volumes.end(),
        [](const ModelVolume *volume) { return volume != nullptr && volume->is_model_part(); });
    const bool painted = std::any_of(object.volumes.begin(), object.volumes.end(),
        [](const ModelVolume *volume) { return volume != nullptr && volume->is_model_part() && volume->is_mm_painted(); });
    return parts >= 2 || (parts == 1 && painted);
}

bool is_single_part_object(const ModelObject &object)
{
    return !is_body_object(object) && std::count_if(object.volumes.begin(), object.volumes.end(),
        [](const ModelVolume *volume) { return volume != nullptr && volume->is_model_part(); }) == 1;
}

std::string body_editor_refusal(const std::vector<ModelObject *> &body_objects, const PrintConfig &resolver,
                                double inherited_height, BodySplitEditorDiagnostic diagnostic,
                                const std::vector<std::string> &single_part)
{
    std::vector<std::string> refused;
    for (ModelObject *object : body_objects)
        if (object != nullptr &&
            !build_body_split_editor_model({object}, resolver, {}, inherited_height).supported)
            refused.push_back(object->name);
    std::string text;
    for (std::size_t i = 0; i < refused.size(); ++i)
        text += (i == 0 ? "" : ", ") + refused[i];
    if (!text.empty())
        text += ": ";
    text += body_editor_sentence(diagnostic);
    const std::string hint = wizard_single_part_hint(single_part);
    if (!hint.empty())
        text += " " + hint;
    return text;
}

// The config a slice of `plate` runs with once `prepared` is published. Stages the prepared values
// into `staged_bundle` on the way.
DynamicPrintConfig staged_plate_slice_config(PresetBundle &staged_bundle, PartPlate &plate,
                                             const PreparedWizardApply &prepared)
{
    staged_bundle.project_config = prepared.project_config;
    staged_bundle.prints.get_edited_preset().config = prepared.setup_plan.after.process_config;
    staged_bundle.printers.get_edited_preset().config = prepared.setup_plan.after.printer_config;
    const auto delta = std::find_if(prepared.plate_deltas.begin(), prepared.plate_deltas.end(),
        [&plate](const WizardPlateConfigDelta &candidate) { return candidate.owner == &plate; });
    const DynamicPrintConfig &plate_config =
        delta == prepared.plate_deltas.end() ? *plate.config() : delta->config;
    std::vector<int> maps = plate.get_real_filament_maps(staged_bundle.project_config);
    const auto *map_mode = plate_config.option<ConfigOptionEnum<FilamentMapMode>>("filament_map_mode");
    const auto *map = plate_config.option<ConfigOptionInts>("filament_map");
    if (map_mode != nullptr && map_mode->value == fmmManual && map != nullptr && !map->values.empty())
        maps = map->values;
    for (const WizardPhysicalRole &role : prepared.draft.resolved_physical_roles)
        if (role.logical_filament < maps.size())
            maps[role.logical_filament] = int(role.physical_extruder + 1);
    std::vector<int> volume_maps = plate.get_filament_volume_maps();
    if (volume_maps.empty())
        volume_maps = staged_bundle.get_default_nozzle_volume_types_for_filaments(maps);
    DynamicPrintConfig config = staged_bundle.full_config(false, maps, volume_maps);
    config.apply(plate_config, true);
    config.set_key_value("filament_map", new ConfigOptionInts(maps));
    return config;
}

// Only the instances on this plate are printed, as Slice prints them.
void only_plate_instances_printable(Model &model, PartPlate &plate)
{
    for (std::size_t object_index = 0; object_index < model.objects.size(); ++object_index) {
        ModelObject *object = model.objects[object_index];
        if (object == nullptr)
            continue;
        for (std::size_t instance_index = 0; instance_index < object->instances.size(); ++instance_index)
            if (object->instances[instance_index] != nullptr)
                object->instances[instance_index]->printable =
                    plate.contain_instance(int(object_index), int(instance_index));
    }
}

// The engine's Body Split admission, run on the staged project the way Slice runs it. Nothing is
// sliced. Only a Body Split refusal is reported; other checks stay with Slice.
std::string body_split_admission_refusal(PresetBundle &staged_bundle, const WizardNativeOwners &owners,
                                         const PreparedWizardApply &prepared, const Model &staged_model,
                                         std::string &refused_key)
{
    try {
        PartPlate &plate = *owners.current_plate;
        DynamicPrintConfig config = staged_plate_slice_config(staged_bundle, plate, prepared);
        config.set_key_value("mixed_nozzle_slicing_mode",
            new ConfigOptionEnum<MixedNozzleSlicingMode>(MixedNozzleSlicingMode::BodySplit));

        Model model = staged_model;
        only_plate_instances_printable(model, plate);
        Print print;
        // BackgroundSlicingProcess sets this before it validates; the native tower type the prime
        // tower admission reads depends on it.
        print.is_BBL_printer() = staged_bundle.is_bbl_vendor();
        // Applied twice, as PartPlate does: the second apply sees the objects the first created.
        print.apply(model, config);
        print.apply(model, config);
        const StringObjectException error = print.validate();
        if (error.string.find("[SRL-") == std::string::npos)
            return {};
        const auto *object = dynamic_cast<const ModelObject *>(error.object);
        // A key the Process tab can open; the mode key names nothing to change there.
        if (!error.opt_key.empty() && error.opt_key != "mixed_nozzle_slicing_mode" &&
            print_config_def.get(error.opt_key) != nullptr)
            refused_key = error.opt_key;
        BOOST_LOG_TRIVIAL(info) << "Mixed nozzle setup review: engine refusal: " << error.string;
        return wizard_admission_refusal_line(error.string, error.opt_key,
                                             object == nullptr ? std::string() : object->name);
    } catch (const std::exception &) {
        // Review never refuses on its own failure to ask. Slice still runs the same check.
        return {};
    }
}

} // namespace

std::set<std::size_t> wizard_touching_assemblies(const std::vector<ModelObject *> &objects,
                                                 const std::vector<BodySplitEditorRow> &rows)
{
    // Faces that meet share a coordinate; the tolerance only absorbs float noise.
    constexpr double tolerance = 1e-3;
    const auto boxes_meet = [](const BoundingBoxf3 &lhs, const BoundingBoxf3 &rhs) {
        for (int axis = 0; axis < 3; ++axis)
            if (lhs.min(axis) > rhs.max(axis) + tolerance || rhs.min(axis) > lhs.max(axis) + tolerance)
                return false;
        return true;
    };
    std::set<std::size_t> touching;
    for (const ModelObject *object : objects) {
        if (object == nullptr)
            continue;
        std::vector<std::pair<BoundingBoxf3, std::size_t>> bodies;
        bool cross_paint = false;
        for (const BodySplitEditorRow &row : rows) {
            if (row.object_id != object->id())
                continue;
            const auto volume = std::find_if(object->volumes.begin(), object->volumes.end(),
                [&row](const ModelVolume *candidate) { return candidate != nullptr && candidate->id() == row.volume_id; });
            if (volume == object->volumes.end())
                continue;
            // A body whose material has no nozzle yet counts as its own tool.
            const std::size_t tool = row.resolution.tool ? row.resolution.tool->physical_extruder
                                                         : std::size_t(-1) - bodies.size();
            bodies.emplace_back((*volume)->mesh().transformed_bounding_box((*volume)->get_matrix()), tool);
            cross_paint = cross_paint || std::any_of(row.painted_logical_filaments.begin(),
                row.painted_logical_filaments.end(), [&row](int filament) { return filament != row.logical_filament; });
        }
        bool found = cross_paint;
        for (std::size_t i = 0; !found && i < bodies.size(); ++i)
            for (std::size_t j = i + 1; !found && j < bodies.size(); ++j)
                found = bodies[i].second != bodies[j].second && boxes_meet(bodies[i].first, bodies[j].first);
        if (found)
            touching.insert(object->id().id);
    }
    return touching;
}

std::string wizard_body_process_guard_line(const PresetBundle &bundle)
{
    const Preset &current = bundle.prints.get_edited_preset();
    const DynamicPrintConfig &process = current.config;
    const double fine = process.has("layer_height") ? process.opt_float("layer_height") : 0.;
    std::optional<double> coarse;
    if (process.has("mixed_nozzle_coarse_layer_height"))
        coarse = process.opt_float("mixed_nozzle_coarse_layer_height");
    const std::vector<double> pair = wizard_nozzle_pair(bundle.project_config,
                                                        bundle.printers.get_edited_preset().config);
    const WizardBodyProcessChoice choice = wizard_body_process_choice(
        wizard_process_rows(bundle.prints), current.name, current.inherits(), pair, fine, coarse);
    return wizard_body_process_guard_text(choice, current.name, pair);
}

WizardProcessBasis wizard_process_basis_for(const WizardDraft &draft, const PresetBundle &bundle)
{
    const Preset &current = bundle.prints.get_edited_preset();
    const std::vector<double> pair = draft.requested_nozzle_diameters
        ? *draft.requested_nozzle_diameters
        : wizard_nozzle_pair(bundle.project_config, bundle.printers.get_edited_preset().config);
    return wizard_process_basis(wizard_basis_rows(bundle.prints, pair), draft.mode, current.name,
                                current.inherits(), pair, draft.chosen_fine_height, draft.keep_current_process);
}

WizardPairAlignment wizard_project_pair_alignment(const PresetBundle &bundle)
{
    const DynamicPrintConfig &printer = bundle.printers.get_edited_preset().config;
    const std::string model = printer.has("printer_model")
        ? printer.opt_string("printer_model") : std::string{};
    return wizard_align_to_project_pair(
        preset_nozzle_values(printer, "nozzle_diameter"),
        composed_nozzle_values(bundle.project_config, printer, "min_layer_height"),
        composed_nozzle_values(bundle.project_config, printer, "max_layer_height"),
        wizard_nozzle_pair(bundle.project_config, printer),
        installed_nozzle_limits(bundle.printers, model));
}

// A slot moved to the coarse nozzle keeps the material values of the nozzle it was bound for (for
// example a fine nozzle's flow cap) until rebound. Rebind those slots through the same plan the
// sidebar's "Rebind filament variants..." offers. Nothing changes when no slot moves nozzle.
static void stage_filament_rebind(const WizardDraft &draft, const WizardNativeOwners &owners,
                                  const std::vector<double> &project_pair, PresetBundle &staged_bundle,
                                  PreparedWizardApply &prepared)
{
    if (draft.mode == MixedNozzleSlicingMode::Off || !draft.fine_logical_filament)
        return;
    const std::vector<int> effective_map = draft.derived_filament_map.empty()
        ? owners.current_plate->get_real_filament_maps(prepared.before.project_config)
        : draft.derived_filament_map;
    const std::size_t fine_slot = *draft.fine_logical_filament;
    if (fine_slot >= effective_map.size())
        return;
    const int fine_extruder = effective_map[fine_slot];
    std::set<std::size_t> other_nozzle;
    for (std::size_t slot = 0; slot < effective_map.size(); ++slot)
        if (effective_map[slot] != fine_extruder)
            other_nozzle.insert(slot);
    if (other_nozzle.empty())
        return;
    const auto note_change = [&prepared](const std::string &line) {
        prepared.review_notes.push_back(line);
        prepared.also_changed.push_back(line);
    };
    const MixedNozzleRebindPlan rebind = staged_bundle.mixed_nozzle_rebind_plan(
        effective_map,
        owners.current_plate->get_real_filament_volume_maps(prepared.before.project_config));
    const auto mm = [](double value) {
        std::ostringstream stream;
        stream << value;
        return stream.str();
    };
    // Slots that keep their own preset while a stock profile for the new nozzle exists.
    // Their step 4 line depends on the final ledger, so it is written below.
    struct KeptSlot {
        std::size_t slot;
        std::string preset;
        std::string profile;
        double nozzle;
        double previous;
    };
    std::vector<KeptSlot> kept_slots;
    // A slot whose own preset already claims the coarse machine composes as "base:" and
    // overlays nothing. Point it at the installed profile for its new nozzle instead.
    for (const std::size_t slot : other_nozzle) {
        if (slot >= staged_bundle.filament_presets.size())
            continue;
        const int tool = effective_map[slot];
        if (tool < 1 || std::size_t(tool) > project_pair.size())
            continue;
        const Preset *slot_preset = staged_bundle.filaments.find_preset(
            staged_bundle.filament_presets[slot], false, true);
        if (slot_preset == nullptr)
            continue;
        const double nozzle = project_pair[std::size_t(tool) - 1];
        const auto sibling = staged_bundle.mixed_nozzle_filament_sibling(*slot_preset, nozzle);
        const auto type_of = [](const Preset &preset) {
            const auto *types = preset.config.option<ConfigOptionStrings>("filament_type");
            return types == nullptr || types->values.empty() ? std::string{} : types->values.front();
        };
        const std::string current = staged_bundle.filament_presets[slot];
        const std::string material = "Material " + std::to_string(slot + 1);
        // Only a stock preset is moved, and only to its stock profile for the new
        // nozzle (same material id, vendor and type). A user or custom preset stays;
        // step 4 says either way.
        if (sibling.overlay != nullptr && sibling.ancestor != nullptr &&
            slot_preset->is_system && sibling.ancestor == slot_preset &&
            sibling.overlay->filament_id == sibling.ancestor->filament_id &&
            type_of(*sibling.overlay) == type_of(*slot_preset) &&
            sibling.overlay->name != current) {
            note_change(material + " profile: " + current + " to " + sibling.overlay->name);
            staged_bundle.filament_presets[slot] = sibling.overlay->name;
            prepared.filament_presets_changed = true;
        } else if (sibling.overlay != nullptr) {
            // The composition takes the other nozzle's values from the stock profile's
            // variant columns for every key the project does not keep as set.
            kept_slots.push_back({slot, current, sibling.overlay->name, nozzle,
                                  project_pair.size() == 2 ? project_pair[tool == 1 ? 1 : 0] : nozzle});
        } else if (sibling.unresolved && project_pair.size() == 2) {
            const double previous = project_pair[tool == 1 ? 1 : 0];
            note_change(material + " keeps " + current + ". It has no profile for the " +
                mm(nozzle) + " mm nozzle, so its " + mm(previous) + " mm settings are used. Pick a " +
                mm(nozzle) + " mm profile for slot " + std::to_string(slot + 1) +
                " in the filament list to change that.");
        }
    }
    if (prepared.filament_presets_changed)
        prepared.filament_presets = staged_bundle.filament_presets;
    if (rebind.offered) {
        // The sidebar's default selection, narrowed to the slots moving to the other
        // nozzle.
        std::vector<bool> accepted = rebind.default_selection();
        for (std::size_t entry = 0; entry < accepted.size(); ++entry)
            accepted[entry] = accepted[entry] &&
                other_nozzle.count(rebind.entries[entry].logical_filament) != 0;
        mixed_nozzle_rebind_write(staged_bundle.project_config, rebind, accepted);
    } else {
        // A project converted by the legacy Rebind carries a blanket ledger entry
        // (mixed_nozzle_ledger_entry_is_blanket) that pins the slot to its old nozzle's
        // values. Release it for the slots moving nozzle, keeping the two
        // material-default keys. An ordinary entry keeps every key.
        const std::size_t filament_count = staged_bundle.filament_presets.size();
        const MixedNozzleLedger ledger = mixed_nozzle_read_ledger(
            staged_bundle.project_config, filament_count);
        if (ledger.qualified && !ledger.absent) {
            std::vector<std::set<std::string>> sets = ledger.explicit_keys;
            bool released = false;
            for (const std::size_t slot : other_nozzle) {
                if (slot >= sets.size() ||
                    !mixed_nozzle_ledger_entry_is_blanket(sets[slot]))
                    continue;
                std::set<std::string> kept;
                for (const char *key : {"filament_prime_volume", "filament_change_length"})
                    if (sets[slot].count(key) != 0)
                        kept.insert(key);
                sets[slot] = std::move(kept);
                released = true;
            }
            if (released)
                mixed_nozzle_write_ledger_sets(staged_bundle.project_config, sets,
                                               ledger.provenance);
        }
    }
    // Step 4 says what a kept preset will print with. A key the project keeps as set is
    // never overlaid (compose_for_tool); a legacy slot keeps every value.
    const MixedNozzleLedger final_ledger = mixed_nozzle_read_ledger(
        staged_bundle.project_config, staged_bundle.filament_presets.size());
    for (const KeptSlot &kept : kept_slots) {
        const bool legacy = final_ledger.is_legacy(kept.slot);
        bool keeps_some = legacy;
        if (!legacy && kept.slot < final_ledger.explicit_keys.size())
            for (const std::string &key : final_ledger.explicit_keys[kept.slot])
                if (key != "filament_prime_volume" && key != "filament_change_length")
                    keeps_some = true;
        const bool keeps_flow = legacy ||
            final_ledger.is_explicit(kept.slot, "filament_max_volumetric_speed");
        const std::string head = "Material " + std::to_string(kept.slot + 1) + " keeps " + kept.preset + ".";
        std::string line;
        if (!keeps_some)
            line = head + " On the " + mm(kept.nozzle) + " mm nozzle it uses that nozzle's settings from " +
                   kept.profile + ".";
        else
            line = head + " This project keeps " + (legacy ? "its " : "some of its ") + mm(kept.previous) +
                   " mm nozzle settings as set, so those are used on the " + mm(kept.nozzle) +
                   " mm nozzle" + (keeps_flow ? ", flow limit included" : "") + ". Pick a " +
                   mm(kept.nozzle) + " mm profile for slot " + std::to_string(kept.slot + 1) +
                   " in the filament list to change that.";
        note_change(line);
    }
}

PreparedWizardApply prepare_wizard_apply(const WizardDraft &draft,
                                          const WizardNativeOwners &owners,
                                          WizardApplyAction action)
{
    PreparedWizardApply prepared;
    prepared.action = action;
    prepared.draft = draft;

    if (!capture_owner_state(owners, prepared.before, prepared.diagnostic))
        return prepared;

    if (action == WizardApplyAction::Cancel)
        return prepared;
    if (!draft_targets_current_owners(draft, prepared.before, prepared.diagnostic))
        return prepared;

    if (unsupported_conversion_intent(draft, prepared.diagnostic))
        return prepared;
    if (draft.mode != MixedNozzleSlicingMode::BodySplit &&
        (!draft.enable_interlocking_objects.empty() || !draft.disable_interlocking_objects.empty() ||
         !draft.keep_joining_objects.empty())) {
        prepared.diagnostic = "Assembly joining choices apply only to Body Split.";
        return prepared;
    }

    MixedNozzleSetupRequest setup_request = setup_request_for(draft);
    // The pair belongs to the project, not the selected printer variant. It decides the printer
    // preset Apply moves to, the limits every check reads, and the values the project keeps.
    const std::vector<double> project_pair = draft.requested_nozzle_diameters
        ? *draft.requested_nozzle_diameters
        : wizard_nozzle_pair(prepared.before.project_config, prepared.before.printer_config);
    const std::string selected_printer_model = prepared.before.printer_config.has("printer_model")
        ? prepared.before.printer_config.opt_string("printer_model") : std::string{};
    if (project_pair.size() == 2 && !setup_request.resolved_printer_name) {
        // Empty when no installed or system variant carries the fine nozzle; the preset then stays and the
        // alignment below still supplies the right limits.
        const std::string variant = wizard_printer_variant_for_pair(
            printer_variant_rows(owners.preset_bundle->printers), prepared.before.printer_name,
            selected_printer_model,
            prepared.before.printer_config.has("printer_variant")
                ? prepared.before.printer_config.opt_string("printer_variant") : std::string{},
            std::min(project_pair[0], project_pair[1]));
        if (!variant.empty())
            setup_request.resolved_printer_name = variant;
    }
    // A catalogue row that owns this cadence is selected through setup staging, so the preset
    // change rides the setup-transition snapshot rather than landing as height overrides.
    const std::optional<std::string> stock_process = wizard_stock_process_selection(
        draft, prepared.before, *owners.preset_bundle, project_pair);
    if (stock_process)
        setup_request.resolved_process_name = stock_process;
    // Stage every fallible decision on a detached bundle; live owners stay untouched until
    // publication.
    PresetBundle staged_bundle(*owners.preset_bundle);
    if (!apply_native_binding(draft, prepared.before, staged_bundle.project_config,
                              prepared.diagnostic) ||
        !apply_process_deltas(draft, prepared.before,
                              staged_bundle.prints.get_edited_preset().config,
                              prepared.diagnostic))
        return prepared;
    // Material binding must resolve against the pair under review, before it is committed.
    apply_requested_nozzle_pair(staged_bundle.project_config, setup_request);
    staged_bundle.prints.update_dirty();
    stage_filament_rebind(draft, owners, project_pair, staged_bundle, prepared);
    // Body Split lands on the pair's Body Split process preset, never a Feature Split one.
    // Compatibility is read after the requested pair is staged.
    std::vector<std::string> body_process_lines;
    std::string basis_preset;
    const bool can_switch_process = draft.scope.kind == WizardScopeKind::ProjectDefault ||
                                    draft.scope.allow_shared_process_changes;
    // Feature Split lands on one basis for every row, the pair's MN Feature preset (or its MN Body
    // preset), with the chosen heights on top. Only an explicit catalogue selection of the same
    // family overrides it.
    if (draft.mode == MixedNozzleSlicingMode::FeatureSplit && project_pair.size() == 2 &&
        !(stock_process && wizard_process_family(*stock_process, {}) == WizardProcessFamily::FeatureSplit)) {
        const Preset &current = owners.preset_bundle->prints.get_edited_preset();
        const std::vector<WizardProcessPresetRow> rows = wizard_basis_rows(staged_bundle.prints, project_pair);
        const WizardProcessBasis basis = wizard_process_basis(rows, draft.mode, current.name, current.inherits(),
            project_pair, draft.chosen_fine_height, draft.keep_current_process);
        double preset_fine = 0.;
        std::optional<double> preset_coarse;
        for (const WizardProcessPresetRow &row : rows)
            if (row.name == basis.preset) {
                preset_fine = row.fine_height;
                preset_coarse = row.coarse_height;
            }
        body_process_lines = wizard_process_basis_lines(basis, current.name, project_pair, preset_fine,
            preset_coarse, draft.chosen_fine_height, draft.chosen_coarse_height, can_switch_process);
        basis_preset = basis.preset;
    }
    if (draft.mode == MixedNozzleSlicingMode::BodySplit && project_pair.size() == 2 && !draft.keep_current_process &&
        !(stock_process && wizard_process_family(*stock_process, {}) == WizardProcessFamily::BodySplit)) {
        if (staged_bundle.get_printer_extruder_count() == 2)
            staged_bundle.update_compatible(PresetSelectCompatibleType::Never);
        const Preset &current = owners.preset_bundle->prints.get_edited_preset();
        const WizardBodyProcessChoice choice = wizard_body_process_choice(
            wizard_process_rows(staged_bundle.prints), current.name, current.inherits(), project_pair,
            draft.chosen_fine_height, draft.chosen_coarse_height);
        body_process_lines = wizard_body_process_lines(choice, current.name, project_pair,
            draft.chosen_fine_height, draft.chosen_coarse_height, can_switch_process);
        basis_preset = choice.preset;
    }
    // Selecting a preset replaces the whole process, so the reviewed values go back on top.
    const bool reapply_process_deltas = !basis_preset.empty() && can_switch_process;
    if (reapply_process_deltas)
        setup_request.resolved_process_name = basis_preset;
    // Tower ownership is deferred until the Feature/Body assignments below are applied.
    MixedNozzleSetupRequest setup_request_without_tower = setup_request;
    setup_request_without_tower.tower_policy.reset();
    setup_request_without_tower.resolve_tower_usage = false;
    prepared.setup_plan = stage_mixed_nozzle_setup(
        staged_bundle, owners.plates, setup_request_without_tower, owners.current_plate);
    if (!prepared.setup_plan.diagnostic.empty()) {
        prepared.diagnostic = prepared.setup_plan.diagnostic;
        return prepared;
    }
    // Publication and stale checks compare against the live owners, not the detached baseline.
    prepared.setup_plan.before = mixed_nozzle_setup_signature(*owners.preset_bundle, owners.plates);
    prepared.project_config = prepared.setup_plan.proposed_project_config;

    // Put the project's pair and its limits in front of every check below and into the published
    // project. A limit that belongs to the other nozzle is replaced from the right machine profile.
    const WizardPairAlignment alignment = wizard_align_to_project_pair(
        preset_nozzle_values(prepared.setup_plan.after.printer_config, "nozzle_diameter"),
        composed_nozzle_values(prepared.project_config, prepared.setup_plan.after.printer_config,
                               "min_layer_height"),
        composed_nozzle_values(prepared.project_config, prepared.setup_plan.after.printer_config,
                               "max_layer_height"),
        project_pair,
        installed_nozzle_limits(owners.preset_bundle->printers, selected_printer_model));
    if (alignment.aligned && alignment.changed) {
        prepared.project_config.set_key_value("nozzle_diameter",
            new ConfigOptionFloats(alignment.nozzle_diameters));
        prepared.project_config.set_key_value("min_layer_height",
            new ConfigOptionFloats(alignment.min_layer_heights));
        prepared.project_config.set_key_value("max_layer_height",
            new ConfigOptionFloats(alignment.max_layer_heights));
        prepared.setup_plan.proposed_project_config = prepared.project_config;
    }

    if (draft.scope.kind == WizardScopeKind::ProjectDefault) {
        set_project_mode(prepared.project_config, draft.mode);
        prepared.setup_plan.proposed_project_config = prepared.project_config;
    }

    // A switched preset carries its own heights, so the chosen ones always go on top.
    const auto put_chosen_heights = [&draft](DynamicPrintConfig &process) {
        if (std::isfinite(draft.chosen_fine_height) && draft.chosen_fine_height > 0.)
            process.set_key_value("layer_height", new ConfigOptionFloat(draft.chosen_fine_height));
        if (draft.chosen_coarse_height && std::isfinite(*draft.chosen_coarse_height) && *draft.chosen_coarse_height > 0.)
            process.set_key_value("mixed_nozzle_coarse_layer_height",
                                  new ConfigOptionFloat(*draft.chosen_coarse_height));
    };
    const auto reapply_reviewed_process = [&](DynamicPrintConfig &process) {
        if (!reapply_process_deltas)
            return true;
        if (!apply_process_deltas(draft, prepared.before, process, prepared.diagnostic))
            return false;
        put_chosen_heights(process);
        return true;
    };
    DynamicPrintConfig process_after = prepared.setup_plan.after.process_config;
    if (!reapply_reviewed_process(process_after))
        return prepared;
    if (draft.mode == MixedNozzleSlicingMode::FeatureSplit)
        adopt_project_feature_split_cadence_override(prepared.project_config, process_after);

    // Compose the final detached state for the validators from the staged values, not the live
    // bundle.
    staged_bundle.project_config = prepared.project_config;
    staged_bundle.prints.get_edited_preset().config = process_after;
    staged_bundle.printers.get_edited_preset().config = prepared.setup_plan.after.printer_config;
    DynamicPrintConfig full_config = wizard_plate_effective_config(
        staged_bundle, *owners.current_plate);

    // Process values the Feature Split engine refuses, so Review names them before Slice does.
    const std::vector<WizardProcessFix> process_fixes = wizard_process_fixes(full_config, draft.mode);

    const std::size_t logical_count = logical_filament_count(draft);
    const auto *staged_nozzles = full_config.option<ConfigOptionFloats>("nozzle_diameter");
    const std::vector<double> nozzle_diameters = staged_nozzles == nullptr
        ? std::vector<double>{} : staged_nozzles->values;
    // Every check below must see the map the wizard is about to write, so the engine's static-map
    // check is not raised for a mapping the wizard sets.
    if (draft.derived_map_mode == fmmManual) {
        const auto *current = full_config.option<ConfigOptionInts>("filament_map");
        const std::vector<int> current_map = current == nullptr ? std::vector<int>{} : current->values;
        mixed_nozzle_write_manual_filament_map(full_config, wizard_derived_filament_map(
            nozzle_diameters, current_map, std::max(logical_count, current_map.size()),
            draft.fine_logical_filament, draft.coarse_logical_filament));
    }

    // Stage the draft's roles into the composed map rather than refuse a project whose saved map
    // lags. A role naming a tool this printer lacks is still refused below.
    if (!draft.resolved_physical_roles.empty() && nozzle_diameters.size() >= 2) {
        const auto *current_binding = full_config.option<ConfigOptionInts>("filament_map");
        std::vector<int> staged_map = current_binding == nullptr
            ? std::vector<int>{} : current_binding->values;
        std::size_t slots = std::max(staged_map.size(), logical_count);
        for (const WizardPhysicalRole &role : draft.resolved_physical_roles)
            slots = std::max(slots, role.logical_filament + 1);
        staged_map.resize(slots, 1);
        for (const WizardPhysicalRole &role : draft.resolved_physical_roles) {
            const int tool = int(role.physical_extruder + 1);
            if (tool >= 1 && std::size_t(tool) <= nozzle_diameters.size())
                staged_map[role.logical_filament] = tool;
        }
        if (current_binding == nullptr || current_binding->values != staged_map)
            full_config.set_key_value("filament_map", new ConfigOptionInts(std::move(staged_map)));
    }

    if (!add_plate_deltas(draft, owners, setup_request, nozzle_diameters, logical_count, prepared))
        return prepared;

    if (!role_bindings_match_effective_config(draft, full_config, prepared.diagnostic))
        return prepared;
    const PrintConfig resolver = feature_split_resolver_config_from_full(full_config);
    // Nothing in this model is published directly; add_model_deltas() extracts owner moves.
    Model staged_model = *owners.model;

    if (draft.mode == MixedNozzleSlicingMode::FeatureSplit) {
        if (!draft.fine_logical_filament || !draft.coarse_logical_filament ||
            *draft.fine_logical_filament == *draft.coarse_logical_filament) {
            prepared.diagnostic = "Mixed-Nozzle wizard Feature assignments are incomplete.";
            return prepared;
        }
        const auto state = build_feature_split_editor_state(
            MixedNozzleSlicingMode::FeatureSplit, resolver.nozzle_diameter.size(),
            std::pair<int, int>{int(*draft.fine_logical_filament + 1),
                                int(*draft.coarse_logical_filament + 1)},
            logical_count, resolver);
        if (!state.can_apply) {
            prepared.diagnostic = "Mixed-Nozzle wizard Feature assignment is not admissible.";
            return prepared;
        }
        FeatureSplitSelection selections{
            int(*draft.fine_logical_filament), int(*draft.coarse_logical_filament)};
        prepared.feature_request = build_feature_split_apply_request(
            {owners.current_plate->get_index(), owners.current_plate->id().id},
            selections, logical_count);
        if (!prepared.feature_request) {
            prepared.diagnostic = "Mixed-Nozzle wizard Feature request is invalid.";
            return prepared;
        }
        if (draft.chosen_coarse_height) {
            const auto current_height = feature_split_effective_coarse_layer_height(
                prepared.before.project_config, prepared.before.process_config);
            const bool coarse_height_changes = !current_height ||
                !is_approx(*current_height, *draft.chosen_coarse_height);
            if (coarse_height_changes && draft.scope.kind == WizardScopeKind::CurrentPlate &&
                !draft.scope.allow_shared_process_changes) {
                prepared.diagnostic = "Mixed-Nozzle wizard coarse cadence requires explicit shared-process scope.";
                return prepared;
            }
            if (coarse_height_changes)
                prepared.feature_request->coarse_layer_height = draft.chosen_coarse_height;
            // The same list the cadence page offered, so Apply cannot refuse an offered row.
            const std::string rejection = mixed_nozzle_coarse_height_rejection(
                draft.chosen_fine_height, *draft.chosen_coarse_height, resolver);
            if (!rejection.empty()) {
                prepared.diagnostic = "This coarse layer cannot be used. " + rejection;
                return prepared;
            }
            if (prepared.feature_request->coarse_layer_height)
                process_after.set_key_value("mixed_nozzle_coarse_layer_height",
                    new ConfigOptionFloat(*prepared.feature_request->coarse_layer_height));
        }
        if (!add_feature_delta(*prepared.feature_request, owners, prepared)) {
            prepared.diagnostic = "Mixed-Nozzle wizard Feature target is unavailable.";
            return prepared;
        }
    }

    if (draft.mode == MixedNozzleSlicingMode::BodySplit) {
        if (!body_targets_current_plate(owners, draft, prepared.diagnostic))
            return prepared;
        if (!apply_body_edits(staged_model, draft, prepared.diagnostic))
            return prepared;
        const double inherited_height = process_after.has("layer_height")
            ? process_after.opt_float("layer_height") : 0.;
        // Only objects that can be Body objects go to the editor; a single-part object gets a hint
        // and the engine decides whether the plate may carry it.
        std::vector<ModelObject *> body_objects;
        std::vector<std::string> single_part_objects;
        for (ModelObject *object : staged_model.objects) {
            if (object == nullptr)
                continue;
            if (is_body_object(*object))
                body_objects.push_back(object);
            else if (is_single_part_object(*object))
                single_part_objects.push_back(object->name);
        }
        const BodySplitEditorModel editor = build_body_split_editor_model(
            body_objects, resolver, {}, inherited_height);
        if (!editor.supported) {
            prepared.diagnostic = body_editor_refusal(body_objects, resolver, inherited_height,
                                                      editor.diagnostic, single_part_objects);
            return prepared;
        }
        const std::string single_part_hint = wizard_single_part_hint(single_part_objects);
        if (!single_part_hint.empty())
            prepared.review_notes.push_back(single_part_hint);
        if (!stage_body_joining_and_review(staged_model, draft, owners, full_config, editor, prepared))
            return prepared;
        if (!add_model_deltas(*owners.model, staged_model, prepared)) {
            prepared.diagnostic = "Mixed-Nozzle wizard Body target changed while staging.";
            return prepared;
        }
    }

    if (setup_request.resolve_tower_usage || setup_request.tower_policy) {
        // Re-run the tower policy on the final plate configs. The first pass would see the live
        // assignments and could miss a new tool.
        staged_bundle.project_config = prepared.project_config;
        staged_bundle.prints.get_edited_preset().config = process_after;
        staged_bundle.printers.get_edited_preset().config = prepared.setup_plan.after.printer_config;
        staged_bundle.prints.update_dirty();
        FinalTowerPlateContext final_tower_context;
        if (!build_final_tower_plate_context(owners, prepared.plate_deltas, staged_model,
                                              final_tower_context, prepared.diagnostic))
            return prepared;
        MixedNozzleSetupPlan final_setup_plan = stage_mixed_nozzle_setup(
            staged_bundle, final_tower_context.views, setup_request,
            final_tower_context.current);
        if (!final_setup_plan.diagnostic.empty()) {
            prepared.diagnostic = final_setup_plan.diagnostic;
            return prepared;
        }
        // The ledger uses serialized plate slots, not runtime ObjectIDs; detached plates keep each
        // live get_index(), so these survive save and reopen.
        for (const auto slot : final_setup_plan.resulting_ledger.consuming_plate_ids) {
            if (std::none_of(owners.plates.begin(), owners.plates.end(),
                    [slot](PartPlate *plate) { return plate->get_index() >= 0 &&
                        std::uint64_t(plate->get_index()) == slot; })) {
                prepared.diagnostic = "Mixed-Nozzle wizard tower consumer is outside the live project.";
                return prepared;
            }
        }
        final_setup_plan.before = mixed_nozzle_setup_signature(*owners.preset_bundle, owners.plates);
        final_setup_plan.affected_plate_ids.clear();
        for (const PartPlate *plate : owners.plates)
            if (plate != nullptr)
                final_setup_plan.affected_plate_ids.push_back(plate->id().id);
        prepared.setup_plan = std::move(final_setup_plan);
        prepared.project_config = prepared.setup_plan.proposed_project_config;
        process_after = prepared.setup_plan.after.process_config;
        if (!reapply_reviewed_process(process_after))
            return prepared;
    }

    // Turn off the plain switches so a clean Apply leaves a sliceable process. The rest are named
    // on Review for the user to change.
    for (const WizardProcessFix &fix : process_fixes) {
        if (!fix.correctable)
            continue;
        if (fix.key == "wipe_tower_type")
            // A printer key, stored with the printer settings like the ledger's printer rows.
            prepared.setup_plan.after.printer_config.set_key_value(fix.key, new ConfigOptionEnum<WipeTowerType>(WipeTowerType::Type1));
        else if (fix.key == "purge_in_prime_tower" || fix.key == "enable_filament_ramming")
            prepared.setup_plan.after.printer_config.set_key_value(fix.key, new ConfigOptionBool(false));
        else if (fix.key == "infill_combination_max_layer_height")
            process_after.set_key_value(fix.key, new ConfigOptionFloatOrPercent(0., false));
        else if (fix.key == "sparse_infill_pattern")
            process_after.set_key_value(fix.key, new ConfigOptionEnum<InfillPattern>(ipGrid));
        else if (fix.key == "wall_generator")
            process_after.set_key_value(fix.key, new ConfigOptionEnum<PerimeterGeneratorType>(PerimeterGeneratorType::Classic));
        else if (fix.key == "wipe_tower_rotation_angle" || fix.key == "minimum_sparse_infill_area" ||
                 fix.key == "top_shell_thickness" || fix.key == "bottom_shell_thickness" ||
                 fix.key == "elefant_foot_compensation")
            process_after.set_key_value(fix.key, new ConfigOptionFloat(0.));
        else if (fix.key == "wipe_tower_filament")
            process_after.set_key_value(fix.key, new ConfigOptionInt(0));
        else
            process_after.set_key_value(fix.key, new ConfigOptionBool(false));
    }

    // Setup always puts the named preset's first layer back and lists it on Review, so an old
    // project first layer the fine nozzle may not print is replaced. Off changes nothing else in
    // the process, so there the draft's flag decides.
    if (draft.mode != MixedNozzleSlicingMode::Off || draft.take_preset_first_layer) {
        // The stored preset, not the edited copy: find_preset() returns the edited preset for the
        // selected name, which is where the project's own first layer lives. The third argument
        // asks for the stored one.
        const DynamicPrintConfig *baseline = &prepared.setup_plan.named_process_baseline;
        if (const Preset *stored = owners.preset_bundle->prints.find_preset(
                prepared.setup_plan.after.process_name, false, true))
            baseline = &stored->config;
        for (const char *key : {"initial_layer_print_height", "initial_layer_speed"}) {
            const ConfigOption *value = baseline->option(key);
            if (value != nullptr)
                process_after.set_key_value(key, value->clone());
            // The project layer is composed after the process, so a differing copy there would
            // shadow what was just written. A copy with the same value shadows nothing and stays.
            const ConfigOption *project_value = prepared.project_config.option(key);
            if (project_value != nullptr && (value == nullptr || !(*project_value == *value)))
                prepared.project_config.erase(key);
        }
    }

    // Setup never changes whether and how the part is supported, or which filaments print the
    // support. A new or named process preset carries its own values (support off, Default style), so
    // the project's choice is written over them. Heights, widths and speeds stay the preset's, as they
    // follow the nozzles.
    for (const char *key : {"enable_support", "support_type", "support_style", "support_filament",
                            "support_interface_filament", "support_threshold_angle", "support_threshold_overlap",
                            "enforce_support_layers", "support_on_build_plate_only", "support_critical_regions_only",
                            "support_remove_small_overhang", "bridge_no_support", "support_interface_top_layers",
                            "support_interface_bottom_layers", "support_base_pattern", "support_interface_pattern"})
        if (const ConfigOption *chosen = prepared.before.process_config.option(key))
            process_after.set_key_value(key, chosen->clone());

    // A support interface on a different material from the body gets the settings stock Orca
    // recommends from the sidebar, written by the same helper. Off mode writes nothing.
    {
        DynamicPrintConfig composed = owners.preset_bundle->full_config();
        composed.apply(process_after);
        composed.apply(prepared.project_config);
        if (mixed_nozzle_apply_support_interface_recommendation(composed)) {
            std::vector<std::string> written = mixed_nozzle_support_interface_recommendation_keys();
            written.emplace_back("support_filament");
            for (const std::string &key : written) {
                if (const ConfigOption *value = composed.option(key))
                    process_after.set_key_value(key, value->clone());
                // The project layer is composed after the process, so a copy left there would
                // shadow this.
                prepared.project_config.erase(key);
            }
        }
    }

    prepared.setup_plan.after.process_config = process_after;
    prepared.setup_plan.proposed_project_config = prepared.project_config;
    prepared.printer_selection = owners.preset_bundle->printers.prepare_preset_selection(
        prepared.setup_plan.after.printer_name, prepared.setup_plan.after.printer_config);
    prepared.process_selection = owners.preset_bundle->prints.prepare_preset_selection(
        prepared.setup_plan.after.process_name, prepared.setup_plan.after.process_config);
    if (!prepared.printer_selection || !prepared.process_selection) {
        prepared.diagnostic = "Mixed-Nozzle wizard selected preset is unavailable.";
        return prepared;
    }

    // Compared at the scope the setup writes: project default against the project mode, plate
    // override against what the plate slices with today.
    const std::string mode_switch = wizard_mode_switch_line(
        draft.scope.kind == WizardScopeKind::ProjectDefault
            ? mixed_nozzle_project_default(prepared.before.project_config)
            : effective_mixed_nozzle_mode(prepared.before.project_config, *owners.current_plate).effective,
        draft.mode);
    if (!mode_switch.empty())
        prepared.review_notes.push_back(mode_switch);
    // A variant mismatch is a preset switch Review states, never a refusal.
    const std::string printer_switch = wizard_preset_switch_line("Printer preset",
        prepared.before.printer_name, prepared.setup_plan.after.printer_name);
    if (!printer_switch.empty())
        prepared.review_notes.push_back(printer_switch);
    if (draft.requested_nozzle_diameters && project_pair.size() == 2)
        prepared.review_notes.push_back("Project nozzle pair: left " + ConfigOptionFloat(project_pair[0]).serialize() +
            " mm, right " + ConfigOptionFloat(project_pair[1]).serialize() + " mm.");
    const std::string assumed_limits = wizard_assumed_limits_line(alignment);
    if (!assumed_limits.empty())
        prepared.review_notes.push_back(assumed_limits);
    for (const WizardProcessFix &fix : process_fixes) {
        prepared.review_notes.push_back(fix.line);
        if (fix.correctable)
            prepared.also_changed.push_back(fix.line);
    }
    if (stock_process && !reapply_process_deltas) {
        prepared.review_notes.push_back(wizard_preset_switch_line("Process preset",
            prepared.before.process_name, prepared.setup_plan.after.process_name));
    } else if (!body_process_lines.empty()) {
        for (const std::string &line : body_process_lines)
            prepared.review_notes.push_back(line);
    } else if (std::any_of(draft.approved_key_deltas.begin(), draft.approved_key_deltas.end(),
                   [](const WizardKeyDelta &delta) {
                       return delta.source == "Resolver" && delta.scope == "Shared process" &&
                              is_cadence_height_key(delta.key) && delta.old_value != delta.new_value;
                   })) {
        prepared.review_notes.push_back(
            "No preset matches these layer heights, so they are set on top of " +
            prepared.before.process_name + ".");
    }

    // Review asks the engine what Slice would refuse, before anything is published.
    if (draft.mode == MixedNozzleSlicingMode::BodySplit) {
        prepared.admission_refusal = body_split_admission_refusal(staged_bundle, owners, prepared, staged_model,
                                                                  prepared.admission_refusal_key);
        if (!prepared.admission_refusal.empty())
            prepared.review_notes.push_back(prepared.admission_refusal);
    }

    prepared.changed = prepared.project_config != prepared.before.project_config ||
        !prepared.object_deltas.empty() || prepared.filament_presets_changed ||
        !prepared.plate_deltas.empty() || !prepared.model_deltas.empty() ||
        owners.preset_bundle->printers.prepared_preset_selection_changes_state(*prepared.printer_selection) ||
        owners.preset_bundle->prints.prepared_preset_selection_changes_state(*prepared.process_selection);
    if (prepared.feature_request) {
        const auto current_pair = owners.current_plate->get_effective_feature_split_filaments(
            prepared.before.project_config, logical_count);
        if (!current_pair || current_pair->first != prepared.feature_request->fine_filament ||
            current_pair->second != prepared.feature_request->coarse_filament ||
            prepared.feature_request->coarse_layer_height.has_value())
            prepared.changed = true;
    }
    return prepared;
}

PreparedWizardApply prepare_wizard_apply(const WizardDraft &draft,
                                          const WizardReview &review,
                                          const WizardNativeOwners &owners,
                                          WizardApplyAction action)
{
    WizardDraft reviewed_draft = draft;
    if (review.selected_candidate_kind)
        reviewed_draft.selected_candidate_kind = review.selected_candidate_kind;
    std::string review_diagnostic;
    const bool routed = route_review_entries(draft, review, reviewed_draft, review_diagnostic);

    PreparedWizardApply prepared = prepare_wizard_apply(reviewed_draft, owners, action);
    if (!routed) {
        prepared.changed = false;
        prepared.diagnostic = review_diagnostic;
        return prepared;
    }
    if (!review.can_apply && prepared.diagnostic.empty()) {
        prepared.changed = false;
        prepared.diagnostic = "Mixed-Nozzle wizard review is not admissible.";
    }
    return prepared;
}

WizardApplyResult commit_wizard_apply(PreparedWizardApply prepared,
                                      const WizardNativeOwners &owners,
                                      const WizardApplyHooks &hooks)
{
    WizardApplyResult result;
    result.affected_plate_ids = prepared.affected_plate_ids;
    if (!prepared.diagnostic.empty()) {
        result.diagnostic = prepared.diagnostic;
        return result;
    }
    if (prepared.action == WizardApplyAction::Cancel || !prepared.changed)
        return result;

    WizardNativeOwnerState current;
    if (!capture_owner_state(owners, current, result.diagnostic) ||
        !same_owner_state(prepared.before, current)) {
        result.diagnostic = "Mixed-Nozzle wizard live native state is stale.";
        return result;
    }
    if (prepared.printer_selection &&
        !owners.preset_bundle->printers.prepared_preset_selection_matches_live(*prepared.printer_selection)) {
        result.diagnostic = "Mixed-Nozzle wizard printer preset is stale.";
        return result;
    }
    if (prepared.process_selection &&
        !owners.preset_bundle->prints.prepared_preset_selection_matches_live(*prepared.process_selection)) {
        result.diagnostic = "Mixed-Nozzle wizard process preset is stale.";
        return result;
    }

    const bool setup_state_changed = prepared.before.printer_name != prepared.setup_plan.after.printer_name ||
        prepared.before.process_name != prepared.setup_plan.after.process_name ||
        prepared.before.printer_config != prepared.setup_plan.after.printer_config ||
        prepared.before.process_config != prepared.setup_plan.after.process_config;
    if (setup_state_changed && !hooks.take_snapshot_with_setup_transition) {
        result.diagnostic = "Mixed-Nozzle wizard Apply requires a prepared native Undo transition snapshot.";
        return result;
    }
    if (!hooks.take_snapshot && !hooks.take_snapshot_with_setup_transition) {
        result.diagnostic = "Mixed-Nozzle wizard Apply has no native Undo snapshot boundary.";
        return result;
    }
    // Plate targets need no second look: same_owner_state compared the plate pointers and ids.
    for (const WizardModelConfigDelta &delta : prepared.model_deltas) {
        if (delta.owner == nullptr || find_volume(*owners.model, delta.object_id, delta.volume_id) != delta.owner) {
            result.diagnostic = "Mixed-Nozzle wizard model target is stale.";
            return result;
        }
    }

    for (const WizardObjectConfigDelta &delta : prepared.object_deltas) {
        if (delta.owner == nullptr || find_object(*owners.model, delta.object_id) != delta.owner) {
            result.diagnostic = "Mixed-Nozzle wizard assembly target is stale.";
            return result;
        }
    }

    // Every operation below is a prepared move or an already-validated native setter. No lookup,
    // allocation, or fallible staging is allowed after this callback opens the Undo action.
    bool snapshot_opened = true;
    if (hooks.take_snapshot_with_setup_transition)
        snapshot_opened = hooks.take_snapshot_with_setup_transition(prepared.setup_plan);
    else
        hooks.take_snapshot();
    if (!snapshot_opened) {
        result.diagnostic = "Mixed-Nozzle wizard native Undo snapshot was suppressed.";
        return result;
    }
    owners.preset_bundle->project_config = std::move(prepared.project_config);
    if (prepared.filament_presets_changed)
        owners.preset_bundle->filament_presets = std::move(prepared.filament_presets);
    for (WizardModelConfigDelta &delta : prepared.model_deltas)
        delta.owner->config.assign_config(std::move(delta.config));
    for (WizardObjectConfigDelta &delta : prepared.object_deltas)
        delta.owner->config.assign_config(std::move(delta.config));
    for (WizardPlateConfigDelta &delta : prepared.plate_deltas)
        *delta.owner->config() = std::move(delta.config);
    if (prepared.printer_selection)
        owners.preset_bundle->printers.apply_prepared_preset_selection(
            std::move(*prepared.printer_selection));
    if (prepared.process_selection)
        owners.preset_bundle->prints.apply_prepared_preset_selection(
            std::move(*prepared.process_selection));

    if (hooks.invalidate)
        for (const std::size_t plate_id : prepared.affected_plate_ids)
            hooks.invalidate(plate_id);
    if (hooks.notify_batch_changed)
        hooks.notify_batch_changed();
    result.applied = true;
    result.changed = true;
    return result;
}

MixedNozzleRankingSlice wizard_staged_slice(const PreparedWizardApply &prepared,
                                            const WizardNativeOwners &owners)
{
    MixedNozzleRankingSlice slice;
    if (owners.preset_bundle == nullptr || owners.model == nullptr || owners.current_plate == nullptr) {
        slice.diagnostic = "The plate is not available.";
        return slice;
    }
    try {
        // The same composition the admission check runs. prepare_wizard_apply does not hand its
        // staged bundle out, so the values are staged again here.
        PresetBundle staged_bundle(*owners.preset_bundle);
        if (prepared.filament_presets_changed)
            staged_bundle.filament_presets = prepared.filament_presets;
        PartPlate &plate = *owners.current_plate;
        slice.config = staged_plate_slice_config(staged_bundle, plate, prepared);
        slice.config.set_key_value("mixed_nozzle_slicing_mode",
            new ConfigOptionEnum<MixedNozzleSlicingMode>(prepared.draft.mode));

        auto model = std::make_shared<Model>(*owners.model);
        for (const WizardObjectConfigDelta &delta : prepared.object_deltas)
            if (ModelObject *object = find_object(*model, delta.object_id))
                object->config.assign_config(delta.config);
        for (const WizardModelConfigDelta &delta : prepared.model_deltas)
            if (ModelVolume *volume = find_volume(*model, delta.object_id, delta.volume_id))
                volume->config.assign_config(delta.config);
        only_plate_instances_printable(*model, plate);

        slice.is_bbl_printer = staged_bundle.is_bbl_vendor();
        slice.plate_origin = plate.get_origin();
        slice.plate_id = plate.id().id;
        slice.key = mixed_nozzle_ranking_key(slice.plate_id, *model, slice.config);
        slice.model = std::move(model);
    } catch (const std::exception &) {
        slice = MixedNozzleRankingSlice{};
        slice.diagnostic = "This row could not be prepared for an estimate.";
    }
    return slice;
}

MixedNozzleRankingSlice wizard_candidate_staged_slice(const WizardDraft &draft,
                                                      const WizardCandidate &candidate,
                                                      const std::vector<WizardBodyAssignment> &body,
                                                      const WizardNativeOwners &owners)
{
    WizardDraft staged = wizard_candidate_draft(draft, candidate, body);
    // The row prints the same whatever scope Apply is given, so the estimate does not wait for
    // the shared process approval. Apply still asks for it.
    staged.scope.allow_shared_process_changes = true;
    // The row's heights come from its own deltas; only the project's binding approval rides along.
    staged.approved_key_deltas.erase(std::remove_if(staged.approved_key_deltas.begin(),
        staged.approved_key_deltas.end(),
        [](const WizardKeyDelta &delta) { return delta.source != "Native binding"; }),
        staged.approved_key_deltas.end());
    const WizardReview review = build_wizard_review(staged, candidate);
    const PreparedWizardApply prepared = prepare_wizard_apply(staged, review, owners);
    if (!prepared.diagnostic.empty()) {
        MixedNozzleRankingSlice refused;
        refused.diagnostic = "Setup would refuse this row.";
        return refused;
    }
    MixedNozzleRankingSlice slice = wizard_staged_slice(prepared, owners);
    // The engine would refuse it at Slice; Review says why.
    if (slice.diagnostic.empty() && !prepared.admission_refusal.empty())
        slice.diagnostic = "Slice would refuse this row.";
    return slice;
}

} // namespace Slic3r::GUI
