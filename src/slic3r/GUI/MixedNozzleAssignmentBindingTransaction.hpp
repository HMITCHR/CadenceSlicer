#pragma once

// Stage on native copies, keep prepared config values and stable identities, then publish inside
// the caller's single Undo/Redo boundary.

#include "libslic3r/Model.hpp"
#include "libslic3r/PresetBundle.hpp"
#include "slic3r/GUI/MixedNozzleSetupController.hpp"
#include "slic3r/GUI/MixedNozzleSetupTransaction.hpp"
#include "slic3r/GUI/PartPlate.hpp"

#include <algorithm>
#include <functional>
#include <optional>
#include <set>
#include <string>
#include <utility>
#include <vector>

namespace Slic3r::GUI {

enum class MixedNozzleAssignmentBindingDiagnostic {
    None,
    Cancelled,
    MissingAssignment,
    InvalidBinding,
    StaleBinding,
    StaleTarget,
    StaleConfiguration,
    AssignmentRejected,
    Unsupported,
};

struct MixedNozzleAssignmentStageResult {
    bool applied {false};
    bool changed {false};
};

// Native copies used only while the Body/Feature validators stage their changes. Never crosses the
// live publication boundary.
struct MixedNozzleAssignmentBindingStageState {
    PresetBundle bundle;
    Model model;
    PartPlate plate;

    MixedNozzleAssignmentBindingStageState(const PresetBundle &source_bundle,
                                           const Model &source_model,
                                           PartPlate &source_plate)
        : bundle(source_bundle), model(source_model)
    {
        plate.copy_local_config_from(source_plate);
        plate.set_index(source_plate.get_index());
    }
};

using MixedNozzleAssignmentStage =
    std::function<MixedNozzleAssignmentStageResult(MixedNozzleAssignmentBindingStageState &)>;

// An object config the assignment changed. Only the beam layer count, which follows the edited
// cadence (body_split_beam_layers), may change on an object.
struct MixedNozzleAssignmentBindingObjectValue {
    ObjectID object_id;
    ModelConfig before;
    ModelConfig proposed;
};

inline bool assignment_binding_object_change_supported(const ModelConfig &before, const ModelConfig &after)
{
    const DynamicPrintConfig &lhs = before.get();
    const DynamicPrintConfig &rhs = after.get();
    std::set<std::string> keys;
    for (const std::string &key : lhs.keys()) keys.insert(key);
    for (const std::string &key : rhs.keys()) keys.insert(key);
    for (const std::string &key : keys) {
        const bool same = lhs.has(key) == rhs.has(key) &&
                          (!lhs.has(key) || lhs.opt_serialize(key) == rhs.opt_serialize(key));
        if (!same && key != "interlocking_beam_layer_count")
            return false;
    }
    return true;
}

struct MixedNozzleAssignmentBindingVolumeValue {
    ObjectID object_id;
    ObjectID volume_id;
    // The address is identity evidence only; publication reacquires the volume by both IDs.
    const ModelObject *source_object {nullptr};
    const ModelVolume *source_volume {nullptr};
    // ModelConfig carries the staged timestamp, and ModelConfig::assign_config(ModelConfig&&) keeps
    // the live config object's ID without a second comparison allocation.
    ModelConfig before;
    ModelConfig proposed;
};

// The assignment may change only the staged ModelConfig of its selected Body volumes. The full
// model scope lets publication reject any other live edit (excluded volume, paint, transform, type,
// geometry, object config or membership) before the Undo snapshot.
struct MixedNozzleAssignmentBindingVolumeScope {
    ObjectID volume_id;
    ModelConfig config;
    ModelVolumeType type {ModelVolumeType::INVALID};
    Transform3d transform {Transform3d::Identity()};
    ObjectBase::Timestamp supported_paint_timestamp {0};
    ObjectBase::Timestamp seam_paint_timestamp {0};
    ObjectBase::Timestamp mmu_paint_timestamp {0};
    ObjectBase::Timestamp fuzzy_paint_timestamp {0};
    const void *mesh_identity {nullptr};
    t_model_material_id material_id;
};

struct MixedNozzleAssignmentBindingInstanceScope {
    ObjectID instance_id;
    Transform3d transform {Transform3d::Identity()};
    bool printable {true};
    bool auto_drop {true};
    int arrange_order {0};
    size_t loaded_id {0};
    bool use_loaded_id_for_label {false};
    ModelInstanceEPrintVolumeState print_volume_state {ModelInstancePVS_Inside};
    bool on_source_plate {false};
};

struct MixedNozzleAssignmentBindingObjectScope {
    ObjectID object_id;
    ModelConfig config;
    bool printable {true};
    std::vector<MixedNozzleAssignmentBindingInstanceScope> instances;
    std::vector<MixedNozzleAssignmentBindingVolumeScope> volumes;
};

struct MixedNozzleAssignmentBindingDraft {
    const PresetBundle *source_bundle {nullptr};
    const Model *source_model {nullptr};
    const PartPlate *source_plate {nullptr};
    const DynamicPrintConfig *source_process_config {nullptr};
    std::string process_name;

    size_t plate_id {0};
    int plate_index {-1};
    DynamicPrintConfig project_before;
    DynamicPrintConfig project;
    DynamicPrintConfig process_before;
    DynamicPrintConfig process;
    DynamicPrintConfig plate_before;
    DynamicPrintConfig plate;
    std::vector<MixedNozzleAssignmentBindingObjectScope> model_scope;
    std::vector<MixedNozzleAssignmentBindingVolumeValue> volumes;
    std::vector<MixedNozzleAssignmentBindingObjectValue> objects;

    MixedNozzleRebindSignature binding_signature;
    bool assignment_applied {false};
    bool assignment_changed {false};
    bool binding_changed {false};
    bool cadence_transition {false};
    std::optional<double> cadence_before;
    std::optional<double> cadence_after;

    bool changed() const { return assignment_changed || binding_changed; }
};

struct MixedNozzleAssignmentBindingRequest {
    // Empty when no legacy conversion is needed. A supplied plan is checked against the native
    // signature before the assignment callback runs.
    std::optional<MixedNozzleRebindPlan> rebind_plan;
    std::vector<bool> accepted_rebind;
    MixedNozzleRebindAction rebind_action {MixedNozzleRebindAction::Cancel};
    MixedNozzleAssignmentStage stage_assignment;
};

struct MixedNozzleAssignmentBindingPlan {
    std::optional<MixedNozzleAssignmentBindingDraft> draft;
    MixedNozzleAssignmentBindingDiagnostic diagnostic {
        MixedNozzleAssignmentBindingDiagnostic::None};

    bool changed() const { return draft && draft->changed(); }
};

struct MixedNozzleAssignmentBindingHooks {
    std::function<void()> take_snapshot;
    // Legacy callers record cadence after their ordinary snapshot; the setup transition hook below
    // carries it inside the Undo action.
    std::function<void(std::optional<double>, std::optional<double>)> record_cadence_transition;
    std::function<void()> notify_batch_changed;
    // Preferred: the caller opens the Undo action with a prepared setup transition. Returning false
    // means the snapshot was suppressed, so publication stops before moving any live owner. The
    // void hook above remains for non-Plater callers.
    std::function<bool(const MixedNozzleSetupPlan &)> take_snapshot_with_setup_transition;
};

inline bool assignment_binding_config_matches(const ModelConfig &lhs, const ModelConfig &rhs)
{
    return lhs.timestamp() == rhs.timestamp() && lhs.get() == rhs.get();
}

inline MixedNozzleAssignmentBindingVolumeScope capture_assignment_binding_volume_scope(
    const ModelVolume &volume)
{
    MixedNozzleAssignmentBindingVolumeScope scope;
    scope.volume_id = volume.id();
    scope.config = volume.config;
    scope.type = volume.type();
    scope.transform = volume.get_matrix();
    scope.supported_paint_timestamp = volume.supported_facets.timestamp();
    scope.seam_paint_timestamp = volume.seam_facets.timestamp();
    scope.mmu_paint_timestamp = volume.mmu_segmentation_facets.timestamp();
    scope.fuzzy_paint_timestamp = volume.fuzzy_skin_facets.timestamp();
    scope.mesh_identity = volume.mesh_ptr().get();
    scope.material_id = volume.material_id();
    return scope;
}

inline MixedNozzleAssignmentBindingObjectScope capture_assignment_binding_object_scope(
    const ModelObject &object, PartPlate *source_plate, size_t object_index)
{
    MixedNozzleAssignmentBindingObjectScope scope;
    scope.object_id = object.id();
    scope.config = object.config;
    scope.instances.reserve(object.instances.size());
    for (size_t instance_index = 0; instance_index < object.instances.size(); ++instance_index) {
        const ModelInstance &instance = *object.instances[instance_index];
        MixedNozzleAssignmentBindingInstanceScope instance_scope;
        instance_scope.instance_id = instance.id();
        instance_scope.transform = instance.get_matrix();
        instance_scope.printable = instance.printable;
        instance_scope.auto_drop = instance.auto_drop;
        instance_scope.arrange_order = instance.arrange_order;
        instance_scope.loaded_id = instance.loaded_id;
        instance_scope.use_loaded_id_for_label = instance.use_loaded_id_for_label;
        instance_scope.print_volume_state = instance.print_volume_state;
        instance_scope.on_source_plate = source_plate != nullptr &&
            source_plate->contain_instance(int(object_index), int(instance_index));
        scope.instances.push_back(instance_scope);
    }
    scope.printable = object.printable;
    scope.volumes.reserve(object.volumes.size());
    for (const ModelVolume *volume : object.volumes)
        scope.volumes.push_back(capture_assignment_binding_volume_scope(*volume));
    return scope;
}

inline std::vector<MixedNozzleAssignmentBindingObjectScope>
capture_assignment_binding_model_scope(const Model &model, PartPlate *source_plate)
{
    std::vector<MixedNozzleAssignmentBindingObjectScope> scope;
    scope.reserve(model.objects.size());
    for (size_t object_index = 0; object_index < model.objects.size(); ++object_index)
        scope.push_back(capture_assignment_binding_object_scope(*model.objects[object_index], source_plate,
                                                                object_index));
    return scope;
}

inline bool assignment_binding_transform_matches(const Transform3d &lhs, const Transform3d &rhs)
{
    const Transform3d::Scalar *lhs_value = lhs.data();
    const Transform3d::Scalar *rhs_value = rhs.data();
    for (size_t i = 0; i < 16; ++i)
        if (lhs_value[i] != rhs_value[i])
            return false;
    return true;
}

inline bool assignment_binding_volume_metadata_matches(
    const ModelVolume &volume, const MixedNozzleAssignmentBindingVolumeScope &scope)
{
    return volume.id() == scope.volume_id && volume.type() == scope.type &&
           assignment_binding_transform_matches(volume.get_matrix(), scope.transform) &&
           volume.supported_facets.timestamp() == scope.supported_paint_timestamp &&
           volume.seam_facets.timestamp() == scope.seam_paint_timestamp &&
           volume.mmu_segmentation_facets.timestamp() == scope.mmu_paint_timestamp &&
           volume.fuzzy_skin_facets.timestamp() == scope.fuzzy_paint_timestamp &&
           volume.mesh_ptr().get() == scope.mesh_identity &&
           volume.material_id() == scope.material_id;
}

inline bool assignment_binding_instance_metadata_matches(
    const ModelInstance &instance, const MixedNozzleAssignmentBindingInstanceScope &scope)
{
    return instance.id() == scope.instance_id &&
           assignment_binding_transform_matches(instance.get_matrix(), scope.transform) &&
           instance.printable == scope.printable && instance.auto_drop == scope.auto_drop &&
           instance.arrange_order == scope.arrange_order && instance.loaded_id == scope.loaded_id &&
           instance.use_loaded_id_for_label == scope.use_loaded_id_for_label &&
           instance.print_volume_state == scope.print_volume_state;
}

// Stage on native copies, then discard them. The plan holds only config values, IDs and identity
// evidence.
inline MixedNozzleAssignmentBindingPlan stage_mixed_nozzle_assignment_binding(
    const PresetBundle &live_bundle, const Model &live_model, PartPlate &live_plate,
    const MixedNozzleAssignmentBindingRequest &request)
{
    MixedNozzleAssignmentBindingPlan result;

    if (request.rebind_plan && request.rebind_plan->offered &&
        request.rebind_action != MixedNozzleRebindAction::Apply) {
        result.diagnostic = MixedNozzleAssignmentBindingDiagnostic::Cancelled;
        return result;
    }
    if (!request.stage_assignment) {
        result.diagnostic = MixedNozzleAssignmentBindingDiagnostic::MissingAssignment;
        return result;
    }

    MixedNozzleAssignmentBindingStageState staged(live_bundle, live_model, live_plate);
    const auto maps = live_plate.get_real_filament_maps(live_bundle.project_config);
    const auto volumes = live_plate.get_real_filament_volume_maps(live_bundle.project_config);
    const MixedNozzleRebindSignature signature = live_bundle.mixed_nozzle_rebind_signature(maps, volumes);

    if (request.rebind_plan && request.rebind_plan->offered) {
        const MixedNozzleRebindPlan &rebind = *request.rebind_plan;
        if (rebind.signature != signature) {
            result.diagnostic = MixedNozzleAssignmentBindingDiagnostic::StaleBinding;
            return result;
        }
        if (!mixed_nozzle_rebind_write(staged.bundle.project_config, rebind, request.accepted_rebind)) {
            result.diagnostic = MixedNozzleAssignmentBindingDiagnostic::InvalidBinding;
            return result;
        }
    }

    const MixedNozzleAssignmentStageResult assignment = request.stage_assignment(staged);
    if (!assignment.applied) {
        result.diagnostic = MixedNozzleAssignmentBindingDiagnostic::AssignmentRejected;
        return result;
    }

    MixedNozzleAssignmentBindingDraft draft;
    draft.source_bundle = &live_bundle;
    draft.source_model = &live_model;
    draft.source_plate = &live_plate;
    draft.source_process_config = &live_bundle.prints.get_edited_preset().config;
    draft.process_name = live_bundle.prints.get_edited_preset().name;
    draft.plate_id = live_plate.id().id;
    draft.plate_index = live_plate.get_index();
    draft.project_before = live_bundle.project_config;
    draft.project = staged.bundle.project_config;
    draft.process_before = live_bundle.prints.get_edited_preset().config;
    draft.process = staged.bundle.prints.get_edited_preset().config;
    draft.plate_before = *live_plate.config();
    draft.plate = *staged.plate.config();
    draft.binding_signature = signature;
    draft.assignment_applied = true;
    draft.assignment_changed = assignment.changed || draft.project_before != draft.project ||
                                draft.process_before != draft.process || draft.plate_before != draft.plate;
    draft.binding_changed = draft.project_before != draft.project &&
                            request.rebind_plan && request.rebind_plan->offered;

    // Publication supports in-place volume config changes and an object's beam layer count. Capture
    // every object and volume value first so any other edit is caught by the live preflight.
    draft.model_scope = capture_assignment_binding_model_scope(live_model, &live_plate);
    if (staged.model.objects.size() != live_model.objects.size()) {
        result.diagnostic = MixedNozzleAssignmentBindingDiagnostic::Unsupported;
        return result;
    }
    for (size_t object_index = 0; object_index < live_model.objects.size(); ++object_index) {
        const ModelObject *source_object = live_model.objects[object_index];
        const ModelObject *staged_object = staged.model.objects[object_index];
        if (source_object == nullptr || staged_object == nullptr ||
            source_object->id() != staged_object->id()) {
            result.diagnostic = MixedNozzleAssignmentBindingDiagnostic::Unsupported;
            return result;
        }
        const MixedNozzleAssignmentBindingObjectScope &object_scope = draft.model_scope[object_index];
        if (!assignment_binding_config_matches(source_object->config, staged_object->config)) {
            if (!assignment_binding_object_change_supported(source_object->config, staged_object->config)) {
                result.diagnostic = MixedNozzleAssignmentBindingDiagnostic::Unsupported;
                return result;
            }
            draft.objects.push_back({source_object->id(), ModelConfig(source_object->config),
                                     ModelConfig(staged_object->config)});
        }
        if (source_object->printable != staged_object->printable ||
            source_object->instances.size() != staged_object->instances.size() ||
            object_scope.instances.size() != source_object->instances.size() ||
            source_object->volumes.size() != staged_object->volumes.size() ||
            object_scope.volumes.size() != source_object->volumes.size()) {
            result.diagnostic = MixedNozzleAssignmentBindingDiagnostic::Unsupported;
            return result;
        }
        for (size_t instance_index = 0; instance_index < source_object->instances.size(); ++instance_index) {
            const ModelInstance *source_instance = source_object->instances[instance_index];
            const ModelInstance *staged_instance = staged_object->instances[instance_index];
            const MixedNozzleAssignmentBindingInstanceScope &instance_scope =
                object_scope.instances[instance_index];
            if (source_instance == nullptr || staged_instance == nullptr ||
                !assignment_binding_instance_metadata_matches(*staged_instance, instance_scope)) {
                result.diagnostic = MixedNozzleAssignmentBindingDiagnostic::Unsupported;
                return result;
            }
        }
        for (size_t volume_index = 0; volume_index < source_object->volumes.size(); ++volume_index) {
            const ModelVolume *source_volume = source_object->volumes[volume_index];
            const ModelVolume *staged_volume = staged_object->volumes[volume_index];
            if (source_volume == nullptr || staged_volume == nullptr ||
                source_volume->id() != staged_volume->id()) {
                result.diagnostic = MixedNozzleAssignmentBindingDiagnostic::Unsupported;
                return result;
            }
            const MixedNozzleAssignmentBindingVolumeScope &volume_scope =
                object_scope.volumes[volume_index];
            if (!assignment_binding_volume_metadata_matches(*staged_volume, volume_scope)) {
                result.diagnostic = MixedNozzleAssignmentBindingDiagnostic::Unsupported;
                return result;
            }
            if (!assignment_binding_config_matches(source_volume->config, staged_volume->config)) {
                draft.volumes.push_back({source_object->id(), source_volume->id(), source_object,
                                         source_volume, ModelConfig(source_volume->config),
                                         ModelConfig(staged_volume->config)});
            }
        }
    }
    if (!draft.volumes.empty() || !draft.objects.empty())
        draft.assignment_changed = true;

    if (draft.project_before.has("mixed_nozzle_coarse_layer_height") ||
        draft.process_before != draft.process) {
        const auto *before = draft.project_before.option<ConfigOptionFloat>(
            "mixed_nozzle_coarse_layer_height");
        const auto *after = draft.project.option<ConfigOptionFloat>(
            "mixed_nozzle_coarse_layer_height");
        const auto *process_before = draft.process_before.option<ConfigOptionFloat>(
            "mixed_nozzle_coarse_layer_height");
        const auto *process_after = draft.process.option<ConfigOptionFloat>(
            "mixed_nozzle_coarse_layer_height");
        draft.cadence_before = before ? std::optional<double>{before->value} :
            (process_before ? std::optional<double>{process_before->value} : std::nullopt);
        draft.cadence_after = after ? std::optional<double>{after->value} :
            (process_after ? std::optional<double>{process_after->value} : std::nullopt);
        draft.cadence_transition = draft.cadence_before != draft.cadence_after ||
                                   draft.project_before.has("mixed_nozzle_coarse_layer_height");
    }

    result.draft = std::move(draft);
    return result;
}

inline PartPlate *find_assignment_binding_plate(const std::vector<PartPlate *> &current_plates,
                                                size_t plate_id, int plate_index)
{
    for (PartPlate *plate : current_plates)
        if (plate != nullptr && plate->get_index() == plate_index && plate->id().id == plate_id)
            return plate;
    return nullptr;
}

inline ModelVolume *find_assignment_binding_volume(Model &model, ObjectID object_id, ObjectID volume_id,
                                                   ModelObject **object_out = nullptr)
{
    for (ModelObject *object : model.objects) {
        if (object->id() != object_id)
            continue;
        if (object_out != nullptr)
            *object_out = object;
        for (ModelVolume *volume : object->volumes)
            if (volume->id() == volume_id)
                return volume;
        return nullptr;
    }
    return nullptr;
}

// Adapts the assignment draft to the setup-transition snapshot. The printer never changes, so its
// before and after are equal; the staged project and process configs are complete. No live owner is
// touched.
inline MixedNozzleSetupPlan assignment_binding_setup_snapshot_plan(
    const PresetBundle &live_bundle, const MixedNozzleAssignmentBindingDraft &draft)
{
    MixedNozzleSetupPlan plan;
    const Preset &printer = live_bundle.printers.get_edited_preset();
    plan.before.printer_name = printer.name;
    plan.before.printer_config = printer.config;
    plan.before.process_name = draft.process_name;
    plan.before.process_config = draft.process_before;
    plan.before.project_config = draft.project_before;
    plan.before.preset_generation = live_bundle.mixed_nozzle_preset_generation;
    plan.before.plate_ids.push_back(draft.plate_id);

    plan.after.printer_name = plan.before.printer_name;
    plan.after.printer_config = plan.before.printer_config;
    plan.after.process_name = draft.process_name;
    plan.after.process_config = draft.process;
    plan.proposed_project_config = draft.project;
    plan.affected_plate_ids.push_back(draft.plate_id);
    return plan;
}

// All validation happens before the snapshot. Publication then performs only prepared no-throw
// moves on reacquired targets, followed by the caller's Undo notification.
inline bool commit_mixed_nozzle_assignment_binding(
    PresetBundle &live_bundle, Model &live_model, const std::vector<PartPlate *> &current_plates,
    MixedNozzleAssignmentBindingPlan &plan,
    const MixedNozzleAssignmentBindingHooks &hooks,
    MixedNozzleAssignmentBindingDiagnostic *diagnostic = nullptr)
{
    auto reject = [&](MixedNozzleAssignmentBindingDiagnostic value) {
        if (diagnostic != nullptr)
            *diagnostic = value;
        return false;
    };
    if (!plan.draft)
        return reject(plan.diagnostic == MixedNozzleAssignmentBindingDiagnostic::None
                          ? MixedNozzleAssignmentBindingDiagnostic::AssignmentRejected
                          : plan.diagnostic);
    MixedNozzleAssignmentBindingDraft &draft = *plan.draft;
    if (&live_bundle != draft.source_bundle || &live_model != draft.source_model)
        return reject(MixedNozzleAssignmentBindingDiagnostic::StaleTarget);
    PartPlate *plate = find_assignment_binding_plate(current_plates, draft.plate_id, draft.plate_index);
    if (plate == nullptr)
        return reject(MixedNozzleAssignmentBindingDiagnostic::StaleTarget);
    if (live_bundle.project_config != draft.project_before ||
        &live_bundle.prints.get_edited_preset().config != draft.source_process_config ||
        live_bundle.prints.get_edited_preset().name != draft.process_name ||
        live_bundle.prints.get_edited_preset().config != draft.process_before ||
        *plate->config() != draft.plate_before)
        return reject(MixedNozzleAssignmentBindingDiagnostic::StaleConfiguration);

    const auto maps = plate->get_real_filament_maps(live_bundle.project_config);
    const auto volumes = plate->get_real_filament_volume_maps(live_bundle.project_config);
    if (live_bundle.mixed_nozzle_rebind_signature(maps, volumes) != draft.binding_signature)
        return reject(MixedNozzleAssignmentBindingDiagnostic::StaleBinding);

    // Reacquire the Body scope by stable IDs while the live graph is untouched; the retained source
    // pointers are never dereferenced.
    if (draft.model_scope.size() != live_model.objects.size())
        return reject(MixedNozzleAssignmentBindingDiagnostic::StaleTarget);
    for (size_t object_index = 0; object_index < live_model.objects.size(); ++object_index) {
        ModelObject *object = live_model.objects[object_index];
        const MixedNozzleAssignmentBindingObjectScope &object_scope = draft.model_scope[object_index];
        if (object == nullptr || object->id() != object_scope.object_id ||
            object->instances.size() != object_scope.instances.size() ||
            object->volumes.size() != object_scope.volumes.size())
            return reject(MixedNozzleAssignmentBindingDiagnostic::StaleTarget);
        if (!assignment_binding_config_matches(object->config, object_scope.config) ||
            object->printable != object_scope.printable)
            return reject(MixedNozzleAssignmentBindingDiagnostic::StaleConfiguration);
        for (size_t instance_index = 0; instance_index < object->instances.size(); ++instance_index) {
            const ModelInstance *instance = object->instances[instance_index];
            const MixedNozzleAssignmentBindingInstanceScope &instance_scope =
                object_scope.instances[instance_index];
            if (instance == nullptr || instance->id() != instance_scope.instance_id)
                return reject(MixedNozzleAssignmentBindingDiagnostic::StaleTarget);
            if (!assignment_binding_instance_metadata_matches(*instance, instance_scope))
                return reject(MixedNozzleAssignmentBindingDiagnostic::StaleConfiguration);
            if (plate->contain_instance(int(object_index), int(instance_index)) !=
                instance_scope.on_source_plate)
                return reject(MixedNozzleAssignmentBindingDiagnostic::StaleTarget);
        }
        for (size_t volume_index = 0; volume_index < object->volumes.size(); ++volume_index) {
            ModelVolume *volume = object->volumes[volume_index];
            const MixedNozzleAssignmentBindingVolumeScope &volume_scope =
                object_scope.volumes[volume_index];
            if (volume == nullptr || volume->id() != volume_scope.volume_id)
                return reject(MixedNozzleAssignmentBindingDiagnostic::StaleTarget);
            if (!assignment_binding_volume_metadata_matches(*volume, volume_scope))
                return reject(MixedNozzleAssignmentBindingDiagnostic::StaleConfiguration);
            if (!assignment_binding_config_matches(volume->config, volume_scope.config))
                return reject(MixedNozzleAssignmentBindingDiagnostic::StaleConfiguration);
        }
    }

    // Resolve every edited volume and object before the snapshot, so the loops after it cannot fail
    // partway. Their configs were matched against the captured scope above; `before` is a copy of the
    // same value, timestamp included.
    std::vector<ModelVolume *> publication_volumes;
    publication_volumes.reserve(draft.volumes.size());
    for (const MixedNozzleAssignmentBindingVolumeValue &value : draft.volumes) {
        ModelObject *object = nullptr;
        ModelVolume *volume = find_assignment_binding_volume(live_model, value.object_id,
                                                             value.volume_id, &object);
        if (volume == nullptr || object == nullptr)
            return reject(MixedNozzleAssignmentBindingDiagnostic::StaleTarget);
        publication_volumes.push_back(volume);
    }

    std::vector<ModelObject *> publication_objects;
    publication_objects.reserve(draft.objects.size());
    for (const MixedNozzleAssignmentBindingObjectValue &value : draft.objects) {
        const auto found = std::find_if(live_model.objects.begin(), live_model.objects.end(),
            [&value](const ModelObject *object) { return object != nullptr && object->id() == value.object_id; });
        if (found == live_model.objects.end())
            return reject(MixedNozzleAssignmentBindingDiagnostic::StaleTarget);
        publication_objects.push_back(*found);
    }

    if (!draft.changed()) {
        if (diagnostic != nullptr)
            *diagnostic = MixedNozzleAssignmentBindingDiagnostic::None;
        return true;
    }

    // Missing callbacks are an integration error; reject before the snapshot.
    const bool prepared_snapshot_hook = bool(hooks.take_snapshot_with_setup_transition);
    if ((!hooks.take_snapshot && !prepared_snapshot_hook) ||
        !hooks.notify_batch_changed ||
        (draft.cadence_transition && !prepared_snapshot_hook && !hooks.record_cadence_transition))
        return reject(MixedNozzleAssignmentBindingDiagnostic::Unsupported);

    bool snapshot_opened = true;
    if (hooks.take_snapshot_with_setup_transition) {
        const MixedNozzleSetupPlan snapshot_plan = assignment_binding_setup_snapshot_plan(live_bundle, draft);
        snapshot_opened = hooks.take_snapshot_with_setup_transition(snapshot_plan);
    } else {
        hooks.take_snapshot();
    }
    if (!snapshot_opened)
        return reject(MixedNozzleAssignmentBindingDiagnostic::Unsupported);

    // All prebuilt values: DynamicPrintConfig move assignment is noexcept and
    // assign_config(ModelConfig&&) keeps the live config ID.
    live_bundle.project_config = std::move(draft.project);
    live_bundle.prints.get_edited_preset().config = std::move(draft.process);
    *plate->config() = std::move(draft.plate);
    for (size_t i = 0; i < publication_volumes.size(); ++i)
        publication_volumes[i]->config.assign_config(std::move(draft.volumes[i].proposed));
    for (size_t i = 0; i < publication_objects.size(); ++i)
        publication_objects[i]->config.assign_config(std::move(draft.objects[i].proposed));

    if (draft.cadence_transition && !prepared_snapshot_hook)
        hooks.record_cadence_transition(draft.cadence_before, draft.cadence_after);
    hooks.notify_batch_changed();
    if (diagnostic != nullptr)
        *diagnostic = MixedNozzleAssignmentBindingDiagnostic::None;
    return true;
}

} // namespace Slic3r::GUI
