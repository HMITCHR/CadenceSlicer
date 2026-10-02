#pragma once

// Shared by the wizard Apply tests: the installed vendor profiles, the commit hook log, a project
// built on real printer/process/material presets, the wizard's own draft pipeline (the steps the
// Plater and the setup dialog run, without the GUI), and a slice of the plate the way Slice runs it.

#include <catch2/catch_all.hpp>

#include "slic3r/GUI/MixedNozzleSetupController.hpp"
#include "slic3r/GUI/MixedNozzleWizardTransaction.hpp"
#include "libslic3r/AppConfig.hpp"
#include "libslic3r/GCode/GCodeProcessor.hpp"
#include "libslic3r/Print.hpp"

#include "test_utils.hpp"

#include <algorithm>
#include <cmath>
#include <fstream>
#include <iterator>
#include <map>
#include <optional>
#include <set>
#include <sstream>
#include <string>
#include <vector>

namespace Slic3r::GUI::WizardTest {

// A vendor's system presets plus Orca's filament library, as the app loads them. Loaded once per
// vendor; every caller gets its own copy.
inline PresetBundle installed_vendor_profiles(const std::string &vendor)
{
    static std::map<std::string, PresetBundle> loaded;
    auto found = loaded.find(vendor);
    if (found == loaded.end()) {
        PresetBundle library;
        library.load_vendor_configs_from_json(PROFILES_DIR, PresetBundle::ORCA_FILAMENT_LIBRARY,
            PresetBundle::LoadSystem, ForwardCompatibilitySubstitutionRule::Disable);
        PresetBundle bundle;
        bundle.load_vendor_configs_from_json(PROFILES_DIR, vendor, PresetBundle::LoadSystem,
            ForwardCompatibilitySubstitutionRule::Disable, &library);
        bundle.load_installed_printers(AppConfig{});
        found = loaded.emplace(vendor, bundle).first;
    }
    return found->second;
}

inline PresetBundle installed_bbl_profiles() { return installed_vendor_profiles("BBL"); }

struct HookLog {
    int snapshots {0};
    int prepared_snapshots {0};
    int notifications {0};
    std::vector<std::size_t> invalidated;
};

inline WizardApplyHooks hooks_for(HookLog &log)
{
    WizardApplyHooks hooks;
    hooks.take_snapshot = [&log] { ++log.snapshots; };
    hooks.take_snapshot_with_setup_transition = [&log](const MixedNozzleSetupPlan &) {
        ++log.snapshots;
        ++log.prepared_snapshots;
        return true;
    };
    hooks.invalidate = [&log](std::size_t plate_id) { log.invalidated.push_back(plate_id); };
    hooks.notify_batch_changed = [&log] { ++log.notifications; };
    return hooks;
}

// The printer, process and materials a fresh project starts on, and the nozzle pair it holds.
struct ProfileSpec {
    std::string vendor;
    std::string printer;
    std::string process;
    // One per material slot; the extra names are only made visible (a nozzle's own profile).
    std::vector<std::string> materials;
    std::vector<std::string> also_visible_materials;
    std::vector<double> nozzles;
    std::vector<double> min_heights;
    std::vector<double> max_heights;
    Vec3d bed {350., 320., 325.};
    // Where the prime tower stands on the plate. The app keeps the tower on the plate; a bare
    // bundle keeps the profile default, which need not be inside this printer's shared area.
    Vec2d tower {200., 150.};
};

// One plate on real presets, mode Off, each slot on its own nozzle (the first two) with a Manual
// map, and no objects yet.
struct WizardProject {
    PresetBundle bundle;
    Model model;
    PartPlate plate;
    WizardNativeOwners owners;

    explicit WizardProject(const ProfileSpec &spec)
        : bundle(installed_vendor_profiles(spec.vendor)),
          plate(nullptr, Vec3d::Zero(), int(spec.bed.x()), int(spec.bed.y()), int(spec.bed.z()), nullptr, &model),
          owners{&bundle, &model, {&plate}, &plate}
    {
        plate.set_index(0);
        Preset *printer = bundle.printers.find_preset(spec.printer, false, true);
        REQUIRE(printer != nullptr);
        printer->is_visible = true;
        Preset *process = bundle.prints.find_preset(spec.process, false, true);
        REQUIRE(process != nullptr);
        process->is_visible = true;
        std::vector<std::string> visible = spec.materials;
        visible.insert(visible.end(), spec.also_visible_materials.begin(), spec.also_visible_materials.end());
        for (const std::string &name : visible) {
            Preset *material = bundle.filaments.find_preset(name, false, true);
            INFO(name);
            REQUIRE(material != nullptr);
            material->is_visible = true;
        }
        REQUIRE(bundle.printers.select_preset_by_name(spec.printer, true));
        REQUIRE(bundle.prints.select_preset_by_name(spec.process, true));
        REQUIRE(bundle.filaments.select_preset_by_name(spec.materials.front(), true));
        REQUIRE(bundle.printers.get_edited_preset().name == spec.printer);
        REQUIRE(bundle.prints.get_edited_preset().name == spec.process);

        const std::size_t slots = spec.materials.size();
        bundle.set_num_filaments(unsigned(slots), std::string("#FFFFFF"));
        bundle.filament_presets = spec.materials;

        auto &project = bundle.project_config;
        project.set_key_value("nozzle_diameter", new ConfigOptionFloats(spec.nozzles));
        project.set_key_value("min_layer_height", new ConfigOptionFloats(spec.min_heights));
        project.set_key_value("max_layer_height", new ConfigOptionFloats(spec.max_heights));
        std::vector<int> map(slots, 1);
        for (std::size_t slot = 0; slot < slots; ++slot)
            map[slot] = int(std::min(slot, spec.nozzles.size() - 1) + 1);
        project.set_key_value("filament_map", new ConfigOptionInts(map));
        project.set_key_value("filament_map_mode", new ConfigOptionEnum<FilamentMapMode>(fmmManual));
        project.set_key_value("filament_volume_map", new ConfigOptionInts(std::vector<int>(slots, 0)));
        std::vector<std::string> colours;
        for (std::size_t slot = 0; slot < slots; ++slot)
            colours.push_back(slot % 2 == 0 ? "#FF0000" : "#0000FF");
        project.set_key_value("filament_colour", new ConfigOptionStrings(colours));
        // The app sizes the purge matrix when the slot count changes: one slots x slots block per
        // nozzle, Orca's usual 140 mm3 between two different materials.
        const std::size_t nozzles = spec.nozzles.size();
        std::vector<double> matrix(nozzles * slots * slots, 140.);
        for (std::size_t nozzle = 0; nozzle < nozzles; ++nozzle)
            for (std::size_t slot = 0; slot < slots; ++slot)
                matrix[nozzle * slots * slots + slot * slots + slot] = 0.;
        project.set_key_value("flush_volumes_matrix", new ConfigOptionFloats(matrix));
        project.set_key_value("flush_multiplier", new ConfigOptionFloats(std::vector<double>(nozzles, 1.)));
        project.set_key_value("mixed_nozzle_slicing_mode",
            new ConfigOptionEnum<MixedNozzleSlicingMode>(MixedNozzleSlicingMode::Off));
        project.set_key_value("wipe_tower_x", new ConfigOptionFloats{spec.tower.x()});
        project.set_key_value("wipe_tower_y", new ConfigOptionFloats{spec.tower.y()});
        bundle.update_compatible(PresetSelectCompatibleType::Never);

        DynamicPrintConfig &plate_config = *plate.config();
        plate_config.set_key_value("filament_map_mode", new ConfigOptionEnum<FilamentMapMode>(fmmManual));
        plate_config.set_key_value("filament_map", new ConfigOptionInts(map));
    }

    // One object with one part per entry, each a box of `size` at `offset` (object space) on its
    // own slot (1-based; 0 sets none). The object sits at `position` on the plate.
    struct Part {
        std::string name;
        Vec3d size;
        Vec3d offset;
        int slot;
    };
    ModelObject *add_object(const std::string &name, const std::vector<Part> &parts, const Vec2d &position)
    {
        ModelObject *object = model.add_object();
        object->name = name;
        for (const Part &part : parts) {
            ModelVolume *volume = object->add_volume(make_cube(part.size.x(), part.size.y(), part.size.z()),
                                                     ModelVolumeType::MODEL_PART, false);
            volume->name = part.name;
            volume->set_offset(part.offset);
            if (part.slot > 0)
                volume->config.set_key_value("extruder", new ConfigOptionInt(part.slot));
        }
        ModelInstance *instance = object->add_instance();
        instance->set_offset(Vec3d(position.x(), position.y(), 0.));
        plate.add_instance(int(model.objects.size() - 1), 0, false);
        return object;
    }
};

// A dual H2D with a 0.2 mm left and a 0.6 mm right nozzle, its stock 0.2 mm process, and two PLA
// slots on the 0.2 mm profile. The limits are what the installed profiles give that pair.
inline ProfileSpec h2d_02_06_spec()
{
    ProfileSpec spec;
    spec.vendor = "BBL";
    spec.printer = "Bambu Lab H2D 0.2 nozzle";
    spec.process = "0.10mm Standard @BBL H2D 0.2 nozzle";
    spec.materials = {"Bambu PLA Basic @BBL H2D 0.2 nozzle", "Bambu PLA Basic @BBL H2D 0.2 nozzle"};
    spec.also_visible_materials = {"Bambu PLA Basic @BBL H2D 0.6 nozzle"};
    spec.nozzles = {0.2, 0.6};
    spec.min_heights = {0.04, 0.12};
    spec.max_heights = {0.14, 0.42};
    spec.tower = Vec2d(260., 120.);
    return spec;
}

// Step 4's Joining choice for a multi-part object. Default is the dialog's own: interlocking beams
// when its bodies touch across the two nozzles, the current settings otherwise.
enum class JoiningChoice { Default, Keep, Beams, Off };

// What the user picks in the wizard.
struct SetupChoice {
    MixedNozzleSlicingMode mode {MixedNozzleSlicingMode::FeatureSplit};
    std::size_t fine_slot {0};
    std::size_t coarse_slot {1};
    double fine_height {0.10};
    // The coarse row, by its ratio to the fine layer.
    int ratio {2};
    WizardTowerIntent tower {WizardTowerIntent::Automatic};
    bool keep_current_process {false};
    // An entry that carries a new nozzle pair (the sidebar diameter change); locks the scope.
    std::optional<std::vector<double>> requested_nozzles;
    std::vector<double> requested_min_heights;
    std::vector<double> requested_max_heights;
    // More options' scope, as the user leaves it.
    WizardScopeKind scope {WizardScopeKind::CurrentPlate};
    JoiningChoice joining {JoiningChoice::Default};
    // Step 1's Fine or Coarse per part, in part order; empty takes the defaults.
    std::vector<WizardBodyRole> body_roles;
    // More options' exact slot per part (0-based); empty or nullopt keeps the role's material.
    std::vector<std::optional<std::size_t>> body_slots;
};

// The draft the Plater opens the wizard with: signature and scope of the current plate.
inline WizardDraft wizard_entry_draft(const WizardProject &project)
{
    WizardDraft draft;
    const PresetBundle &bundle = project.bundle;
    draft.signature.selected_printer_id = bundle.printers.get_edited_preset().name;
    draft.signature.selected_process_id = bundle.prints.get_edited_preset().name;
    draft.signature.selected_material_ids = bundle.filament_presets;
    for (const PartPlate *plate : project.owners.plates) {
        draft.signature.plate_ids.push_back(plate->id().id);
        draft.scope.process_affected_plate_ids.push_back(plate->id().id);
    }
    for (const ModelObject *object : project.model.objects)
        for (const ModelVolume *volume : object->volumes) {
            WizardVolumeSignature signature;
            signature.object_id = object->id().id;
            signature.volume_id = volume->id().id;
            signature.config_revision = static_cast<const ModelConfig &>(volume->config).timestamp();
            signature.paint_revision = volume->mmu_segmentation_facets.timestamp();
            const auto matrix = volume->get_matrix();
            std::copy(matrix.data(), matrix.data() + 16, signature.transform.begin());
            draft.signature.volumes.push_back(signature);
        }
    draft.scope.kind = WizardScopeKind::CurrentPlate;
    draft.scope.current_plate_id = project.plate.id().id;
    return draft;
}

// The wizard from entry to Apply, the way the Plater and the dialog run it: materials and the map
// they derive, each material's nozzle, More options, the cadence page row, the body roles, Review,
// then prepare and commit. Every step must pass; the committed result is returned.
inline WizardApplyResult run_wizard_setup(WizardProject &project, const SetupChoice &choice)
{
    INFO("test: " << Catch::getResultCapture().getCurrentTestName());
    INFO("setup on " << project.bundle.printers.get_edited_preset().name << ": "
         << (choice.mode == MixedNozzleSlicingMode::BodySplit ? "Body Split" : "Feature Split")
         << " fine " << choice.fine_height << " ratio " << choice.ratio);
    WizardDraft draft = wizard_entry_draft(project);
    draft.requested_nozzle_diameters = choice.requested_nozzles;
    draft.requested_min_layer_heights = choice.requested_min_heights;
    draft.requested_max_layer_heights = choice.requested_max_heights;

    // The pages describe the requested pair when the entry carries one.
    PresetBundle entry_bundle = project.bundle;
    if (choice.requested_nozzles) {
        entry_bundle.project_config.set_key_value("nozzle_diameter", new ConfigOptionFloats(*choice.requested_nozzles));
        entry_bundle.project_config.set_key_value("min_layer_height", new ConfigOptionFloats(choice.requested_min_heights));
        entry_bundle.project_config.set_key_value("max_layer_height", new ConfigOptionFloats(choice.requested_max_heights));
        entry_bundle.update_compatible(PresetSelectCompatibleType::Never);
    }
    const DynamicPrintConfig full = wizard_plate_effective_config(entry_bundle, project.plate);
    FullPrintConfig effective;
    effective.apply(full, true);
    const std::vector<int> plate_map = effective.filament_map.values;

    draft.mode = choice.mode;
    draft.fine_logical_filament = choice.fine_slot;
    draft.coarse_logical_filament = choice.coarse_slot;
    draft.derived_filament_map = wizard_derived_filament_map(effective.nozzle_diameter.values, plate_map,
        project.bundle.filament_presets.size(), choice.fine_slot, choice.coarse_slot);
    draft.derived_map_mode = fmmManual;
    effective.filament_map_mode.value = fmmManual;
    effective.filament_map.values = draft.derived_filament_map;
    for (const auto &[slot, role] : std::vector<std::pair<std::size_t, std::string>>{
             {choice.fine_slot, "fine"}, {choice.coarse_slot, "coarse"}}) {
        const MixedNozzleToolResolution resolved = resolve_mixed_nozzle_tool(effective, slot,
            MixedNozzleResolveScope::PhysicalToolOnly);
        INFO("slot " << slot + 1 << " has no nozzle of its own");
        REQUIRE(resolved);
        draft.resolved_physical_roles.push_back({slot, resolved.tool->physical_extruder,
                                                 resolved.tool->nozzle_diameter, role});
    }

    const bool scope_locked = choice.requested_nozzles.has_value();
    if (scope_locked)
        draft.scope.kind = WizardScopeKind::ProjectDefault;
    WizardMoreOptions options = wizard_more_options(draft, false, scope_locked);
    options.scope = choice.scope;
    options.tower = choice.tower;
    options.keep_current_process = choice.keep_current_process;
    wizard_apply_more_options(draft, options);
    draft.scope.allow_shared_process_changes = true;

    draft.chosen_fine_height = choice.fine_height;
    const std::vector<WizardCandidate> candidates = build_wizard_candidates(draft, effective, {});
    const WizardCadencePage page = wizard_cadence_page(candidates, choice.fine_height, effective);
    const WizardCandidate *candidate = nullptr;
    for (const WizardCadenceRow &row : page.rows)
        if (row.ratio == choice.ratio)
            candidate = &candidates[row.candidate_index];
    INFO("the cadence page offers no row at ratio " << choice.ratio << ": " << page.exclusions);
    REQUIRE(candidate != nullptr);

    std::vector<WizardBodyAssignment> body;
    if (choice.mode == MixedNozzleSlicingMode::BodySplit) {
        std::vector<WizardBodyRoleRow> rows;
        for (const ModelObject *object : project.model.objects)
            for (const ModelVolume *volume : object->volumes) {
                if (!volume->is_model_part())
                    continue;
                WizardBodyRoleRow row;
                row.object_id = object->id().id;
                row.volume_id = volume->id().id;
                row.current_slot = volume->config.has("extruder") ? volume->config.opt_int("extruder") - 1 : -1;
                if (row.current_slot >= 0)
                    if (const MixedNozzleToolResolution resolved = resolve_mixed_nozzle_tool(
                            effective, std::size_t(row.current_slot), MixedNozzleResolveScope::PhysicalToolOnly))
                        row.current_physical = resolved.tool->physical_extruder;
                row.volume = volume->mesh().stats().volume;
                rows.push_back(row);
            }
        const std::size_t fine_physical = draft.resolved_physical_roles[0].physical_extruder;
        const std::size_t coarse_physical = draft.resolved_physical_roles[1].physical_extruder;
        // An exact slot resolves its nozzle on the map setup will write, as the dialog does.
        std::vector<std::optional<WizardBodySlotOverride>> overrides;
        for (const std::optional<std::size_t> &slot : choice.body_slots) {
            if (!slot) {
                overrides.emplace_back();
                continue;
            }
            const MixedNozzleToolResolution resolved = resolve_mixed_nozzle_tool(effective, *slot,
                MixedNozzleResolveScope::PhysicalToolOnly);
            REQUIRE(resolved);
            overrides.push_back(WizardBodySlotOverride{*slot, resolved.tool->physical_extruder});
        }
        body = wizard_body_roles_to_slots(rows,
            choice.body_roles.empty() ? wizard_default_body_roles(rows, fine_physical) : choice.body_roles,
            choice.fine_slot, choice.coarse_slot, fine_physical, coarse_physical, overrides);
    }
    draft = wizard_candidate_draft(draft, *candidate, body);
    if (choice.mode == MixedNozzleSlicingMode::BodySplit) {
        // Every multi-part object gets its Joining choice, as step 4 shows them.
        std::vector<ModelObject *> objects = project.plate.get_objects_on_this_plate();
        const BodySplitEditorModel editor = build_body_split_editor_model(objects,
            feature_split_resolver_config_from_full(full), {}, choice.fine_height);
        const std::set<std::size_t> touching = wizard_touching_assemblies(objects, editor.rows);
        for (const ModelObject *object : objects) {
            std::size_t parts = 0;
            for (const ModelVolume *volume : object->volumes)
                parts += volume->is_model_part() ? 1 : 0;
            if (parts < 2)
                continue;
            const std::size_t id = object->id().id;
            JoiningChoice joining = choice.joining;
            if (joining == JoiningChoice::Default)
                joining = touching.count(id) ? JoiningChoice::Beams : JoiningChoice::Keep;
            (joining == JoiningChoice::Beams ? draft.enable_interlocking_objects
             : joining == JoiningChoice::Off ? draft.disable_interlocking_objects
                                             : draft.keep_joining_objects).insert(id);
        }
    }

    const WizardReview review = build_wizard_review(draft, *candidate);
    for (const std::string &error : review.errors)
        UNSCOPED_INFO("review: " << error);
    REQUIRE(review.can_apply);
    PreparedWizardApply prepared = prepare_wizard_apply(draft, review, project.owners);
    const auto config_value = [](const DynamicPrintConfig &config, const char *key) {
        return config.has(key) ? config.opt_serialize(key) : std::string("absent");
    };
    INFO("prepared names printer=" << prepared.setup_plan.after.printer_name
         << " process=" << prepared.setup_plan.after.process_name
         << " live printer=" << project.bundle.printers.get_edited_preset().name
         << " live process=" << project.bundle.prints.get_edited_preset().name);
    INFO("prepared process first=" << config_value(prepared.setup_plan.after.process_config, "initial_layer_print_height")
         << " layer=" << config_value(prepared.setup_plan.after.process_config, "layer_height")
         << " min=" << config_value(prepared.setup_plan.after.process_config, "min_layer_height")
         << " max=" << config_value(prepared.setup_plan.after.process_config, "max_layer_height"));
    INFO("prepared project nozzles=" << config_value(prepared.project_config, "nozzle_diameter")
         << " min=" << config_value(prepared.project_config, "min_layer_height")
         << " max=" << config_value(prepared.project_config, "max_layer_height"));
    INFO("prepared printer nozzles=" << config_value(prepared.setup_plan.after.printer_config, "nozzle_diameter")
         << " min=" << config_value(prepared.setup_plan.after.printer_config, "min_layer_height")
         << " max=" << config_value(prepared.setup_plan.after.printer_config, "max_layer_height"));
    const Preset *prepared_printer = project.bundle.printers.find_preset(
        prepared.setup_plan.after.printer_name, false, true);
    const Preset *prepared_process = project.bundle.prints.find_preset(
        prepared.setup_plan.after.process_name, false, true);
    INFO("prepared preset lookup printer=" << (prepared_printer != nullptr)
         << " visible=" << (prepared_printer != nullptr && prepared_printer->is_visible)
         << " process=" << (prepared_process != nullptr)
         << " visible=" << (prepared_process != nullptr && prepared_process->is_visible));
    if (!prepared.admission_refusal.empty()) {
        PresetBundle staged(project.bundle);
        staged.project_config = prepared.project_config;
        staged.prints.get_edited_preset().config = prepared.setup_plan.after.process_config;
        staged.printers.get_edited_preset().config = prepared.setup_plan.after.printer_config;
        std::vector<int> maps = project.plate.get_real_filament_maps(staged.project_config);
        DynamicPrintConfig staged_full = staged.full_config(false, maps,
            project.plate.get_filament_volume_maps());
        const auto plate_delta = std::find_if(prepared.plate_deltas.begin(), prepared.plate_deltas.end(),
            [&project](const WizardPlateConfigDelta &delta) { return delta.owner == &project.plate; });
        staged_full.apply(plate_delta == prepared.plate_deltas.end()
            ? *project.plate.config() : plate_delta->config, true);
        INFO("admission composition first=" << config_value(staged_full, "initial_layer_print_height")
             << " layer=" << config_value(staged_full, "layer_height")
             << " coarse=" << config_value(staged_full, "mixed_nozzle_coarse_layer_height")
             << " nozzles=" << config_value(staged_full, "nozzle_diameter")
             << " min=" << config_value(staged_full, "min_layer_height")
             << " max=" << config_value(staged_full, "max_layer_height"));
    }
    INFO("prepare: " << prepared.diagnostic);
    REQUIRE(prepared.diagnostic.empty());
    INFO("engine refusal at Review: " << prepared.admission_refusal);
    REQUIRE(prepared.admission_refusal.empty());
    HookLog log;
    const WizardApplyResult result = commit_wizard_apply(std::move(prepared), project.owners, hooks_for(log));
    INFO("commit: " << result.diagnostic);
    REQUIRE(result.applied);
    return result;
}

// A slice of the plate as Slice runs it: the composed plate config with the plate's own settings
// on top (BackgroundSlicingProcess::apply), applied twice, validated, processed and exported.
struct PlateSlice {
    DynamicPrintConfig config;
    // Print::validate()'s refusal; empty when admitted.
    std::string refusal;
    std::string gcode;
    // Each printing region's sparse infill and outer wall filament (1-based), for failure messages.
    std::vector<std::pair<int, int>> region_filaments;
    // The Feature Split whole-print time decision (first line of the engine's digest), if it ran.
    std::string economics;
    // Feature bands by outcome and reason ("outcome:reason" -> count), from the advisor snapshot.
    std::map<std::string, int> band_reasons;
    // The tower brim the engine resolved (mm), 0 without a tower.
    float tower_brim_width {0.f};
    // The G-code processor's moves and statistics from the export.
    std::vector<GCodeProcessorResult::MoveVertex> moves;
    PrintEstimatedStatistics statistics;
};

// The config Plater hands the engine at Slice: per-filament presets expanded by the engine, the
// plate's own settings on top. A pre-collapsed full config would read every material's variant
// columns through the first filament.
inline DynamicPrintConfig plate_slice_config(WizardProject &project, PartPlate &plate)
{
    DynamicPrintConfig config;
    if (project.bundle.get_printer_extruder_count() > 1) {
        std::vector<int> maps = plate.get_real_filament_maps(project.bundle.project_config);
        std::vector<int> volume_maps = plate.get_filament_volume_maps();
        if (volume_maps.empty())
            volume_maps = project.bundle.get_default_nozzle_volume_types_for_filaments(maps);
        config = project.bundle.full_config(false, maps, volume_maps);
    } else
        config = project.bundle.full_config(false);
    config.apply(*plate.config());
    return config;
}

inline DynamicPrintConfig plate_slice_config(WizardProject &project)
{
    return plate_slice_config(project, project.plate);
}

inline PlateSlice slice_plate(WizardProject &project)
{
    PlateSlice out;
    out.config = plate_slice_config(project);
    Print print;
    print.set_status_silent();
    print.is_BBL_printer() = project.bundle.is_bbl_vendor();
    print.set_plate_origin(project.plate.get_origin());
    print.apply(project.model, out.config);
    print.apply(project.model, out.config);
    out.refusal = print.validate().string;
    if (!out.refusal.empty())
        return out;
    print.process();
    const std::string &digest = print.feature_economics_whole_print_digest();
    out.economics = digest.substr(0, digest.find('\n'));
    for (const PrintObject *object : print.objects())
        for (std::size_t region = 0; region < object->num_printing_regions(); ++region)
            out.region_filaments.emplace_back(object->printing_region(region).config().sparse_infill_filament_id.value,
                                              object->printing_region(region).config().outer_wall_filament_id.value);
    for (const PrintObject *object : print.objects())
        if (const auto observations = object->mixed_nozzle_advisor_observations())
            for (const auto &band : observations->feature_bands)
                ++out.band_reasons[std::to_string(int(band.outcome)) + ":" + band.reason];
    if (print.has_wipe_tower())
        out.tower_brim_width = print.wipe_tower_data().brim_width;
    ScopedTemporaryFile file(".gcode");
    GCodeProcessorResult result;
    print.export_gcode(file.string(), &result, nullptr);
    out.moves = std::move(result.moves);
    out.statistics = result.print_statistics;
    std::ifstream stream(file.string());
    out.gcode.assign(std::istreambuf_iterator<char>(stream), std::istreambuf_iterator<char>());
    return out;
}

} // namespace Slic3r::GUI::WizardTest
