
#include <catch2/catch_all.hpp>

#include "wizard_apply_support.hpp"

#include "fff_print/mixed_nozzle_harness.hpp"
#include "slic3r/GUI/MixedNozzleAssignmentBindingTransaction.hpp"
#include "slic3r/GUI/FeatureSplitEditorNative.hpp"
#include "slic3r/GUI/MixedNozzleSetupTransaction.hpp"
#include "slic3r/GUI/MixedNozzleTowerPolicy.hpp"
#include "slic3r/GUI/MixedNozzleSidebarController.hpp"
#include "slic3r/GUI/MixedNozzleDecisionReport.hpp"
#include "libslic3r/Format/bbs_3mf.hpp"
#include "libslic3r/MixedNozzleBinding.hpp"
#include "libslic3r/MixedNozzleConfig.hpp"
#include "libslic3r/miniz_extension.hpp"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <limits>
#include <map>
#include <set>
#include <string>
#include <vector>

using namespace Slic3r;
using namespace Slic3r::GUI;
using namespace Slic3r::GUI::WizardTest;
using Slic3r::Emitted::EmittedFacts;
using Slic3r::Emitted::extract_emitted_facts;

namespace {

constexpr double kHeightTolerance = 0.011;

bool mark_native_archive_as_cadence(const std::string &source, const std::string &destination)
{
    MZ_Archive input, output;
    if (!open_zip_reader(&input.arch, source))
        return false;
    if (!open_zip_writer(&output.arch, destination)) {
        close_zip_reader(&input.arch);
        return false;
    }
    bool okay = true, found = false;
    for (mz_uint index = 0; index < mz_zip_reader_get_num_files(&input.arch) && okay; ++index) {
        mz_zip_archive_file_stat stat;
        okay = mz_zip_reader_file_stat(&input.arch, index, &stat);
        if (!okay)
            break;
        if (std::string(stat.m_filename) != "3D/3dmodel.model") {
            okay = mz_zip_writer_add_from_zip_reader(&output.arch, &input.arch, index);
            continue;
        }
        std::string xml(std::size_t(stat.m_uncomp_size), '\0');
        okay = mz_zip_reader_extract_to_mem(&input.arch, index, xml.data(), xml.size(), 0);
        if (!okay)
            break;
        const std::string application = "<metadata name=\"Application\">";
        const std::size_t begin = xml.find(application);
        const std::size_t end = begin == std::string::npos ? begin : xml.find("</metadata>", begin);
        if (begin == std::string::npos || end == std::string::npos) {
            okay = false;
            break;
        }
        xml.replace(begin + application.size(), end - begin - application.size(), "CadenceSlicer-2.3.2");
        // The native writer also emits an Orca version marker. Remove it so the
        // importer must recognize the actual Application value.
        const std::string other = "<metadata name=\"OrcaSlicer\">";
        const std::size_t other_begin = xml.find(other);
        if (other_begin != std::string::npos) {
            const std::size_t other_end = xml.find("</metadata>", other_begin);
            if (other_end == std::string::npos) {
                okay = false;
                break;
            }
            xml.erase(other_begin, other_end + std::strlen("</metadata>") - other_begin);
        }
        okay = mz_zip_writer_add_mem(&output.arch, stat.m_filename, xml.data(), xml.size(), MZ_DEFAULT_COMPRESSION);
        found = okay;
    }
    okay = okay && found && mz_zip_writer_finalize_archive(&output.arch);
    const bool output_closed = close_zip_writer(&output.arch);
    const bool input_closed = close_zip_reader(&input.arch);
    return okay && output_closed && input_closed;
}

bool object_role(ExtrusionRole role)
{
    return role != erWipeTower && role != erSkirt && role != erBrim && role != erSupportMaterial &&
           role != erSupportMaterialInterface && role != erSupportTransition && role != erCustom &&
           role != erNone && role != erMixed;
}

struct ToolPrint {
    std::size_t facts {0};
    float max_wall_height {0.f};
    float max_infill_height {0.f};
    std::set<ExtrusionRole> roles;
};

std::map<unsigned char, ToolPrint> object_print_by_tool(const EmittedFacts &facts)
{
    unsigned int first_layer = std::numeric_limits<unsigned int>::max();
    for (const auto &fact : facts.role_facts)
        if (object_role(fact.role))
            first_layer = std::min(first_layer, fact.layer_id);
    std::map<unsigned char, ToolPrint> tools;
    for (const auto &fact : facts.role_facts) {
        if (!object_role(fact.role) || fact.layer_id == first_layer)
            continue;
        ToolPrint &tool = tools[fact.physical_tool];
        ++tool.facts;
        if (fact.role == erExternalPerimeter || fact.role == erPerimeter)
            tool.max_wall_height = std::max(tool.max_wall_height, fact.height);
        if (fact.role == erInternalInfill)
            tool.max_infill_height = std::max(tool.max_infill_height, fact.height);
        tool.roles.insert(fact.role);
    }
    return tools;
}

unsigned char physical_of(const PlateSlice &slice, std::size_t slot)
{
    const auto *map = slice.config.option<ConfigOptionInts>("filament_map");
    REQUIRE(map != nullptr);
    REQUIRE(slot < map->values.size());
    return static_cast<unsigned char>(map->values[slot] - 1);
}

struct TowerFootprint {
    std::size_t moves {0};
    float first_layer_width {0.f};
    float upper_width {0.f};
};

TowerFootprint tower_footprint(const std::vector<GCodeProcessorResult::MoveVertex> &moves)
{
    std::map<unsigned int, std::pair<float, float>> x_range;
    TowerFootprint footprint;
    for (const auto &move : moves) {
        if (move.type != EMoveType::Extrude || move.extrusion_role != erWipeTower)
            continue;
        ++footprint.moves;
        auto [it, inserted] = x_range.try_emplace(move.layer_id, move.position.x(), move.position.x());
        it->second.first = std::min(it->second.first, move.position.x());
        it->second.second = std::max(it->second.second, move.position.x());
    }
    if (x_range.empty())
        return footprint;
    footprint.first_layer_width = x_range.begin()->second.second - x_range.begin()->second.first;
    for (auto it = std::next(x_range.begin()); it != x_range.end(); ++it)
        footprint.upper_width = std::max(footprint.upper_width, it->second.second - it->second.first);
    return footprint;
}

void add_cube(WizardProject &project, const Vec2d &position = Vec2d(20., 20.),
              const Vec3d &size = Vec3d(180., 180., 12.))
{
    project.add_object("box", {{"box", size, Vec3d::Zero(), 1}}, position);
}

void add_two_bodies(WizardProject &project, const Vec2d &position = Vec2d(100., 100.))
{
    project.add_object("two bodies", {{"left body", Vec3d(10., 20., 10.), Vec3d::Zero(), 1},
                                      {"right body", Vec3d(10., 20., 10.), Vec3d(15., 0., 0.), 2}},
                       position);
}

void turn_off_current_plate(WizardProject &project)
{
    const MixedNozzleSetupRequest off = mixed_nozzle_off_request(false);
    const MixedNozzleSetupPlan plan = stage_mixed_nozzle_setup(project.bundle, project.owners.plates, off,
                                                               &project.plate);
    INFO(plan.diagnostic);
    REQUIRE(plan.diagnostic.empty());
    const MixedNozzleSetupHooks hooks{[] {}, [](PartPlate &) {}, [] {}};
    REQUIRE(commit_mixed_nozzle_setup_transaction(project.bundle, project.owners.plates, plan, off, hooks,
                                                  &project.plate));
    REQUIRE(project.plate.config()->opt_enum<MixedNozzleSlicingMode>("mixed_nozzle_slicing_mode") ==
            MixedNozzleSlicingMode::Off);
}

PlateSlice require_slice(WizardProject &project)
{
    return admitted_slice(project);
}

DynamicPrintConfig workflow_print_config(WizardProject &project)
{
    DynamicPrintConfig config = plate_slice_config(project);
    mixed_nozzle_fixtures::fill_per_filament_values(config);
    return config;
}

CadenceTest::Facts slice_reused(Print &print, WizardProject &project)
{
    const DynamicPrintConfig config = workflow_print_config(project);
    print.apply(project.model, config);
    print.apply(project.model, config);
    CadenceTest::Facts facts = CadenceTest::slice(print);
    INFO(facts.refusal.string);
    REQUIRE(facts.refusal.string.empty());
    REQUIRE_FALSE(facts.gcode.empty());
    return facts;
}

// A reopened project sliced as Slice slices it: the given config, applied twice.
PlateSlice slice_reopened(const ReopenedProject &reopened, const DynamicPrintConfig &config)
{
    Print print;
    print.set_status_silent();
    print.is_BBL_printer() = true;
    print.apply(reopened.model, config);
    print.apply(reopened.model, config);
    const CadenceTest::Facts facts = CadenceTest::slice(print);
    INFO(facts.refusal.string);
    REQUIRE(facts.refusal.string.empty());
    PlateSlice slice;
    slice.config = config;
    slice.gcode = facts.gcode;
    slice.moves = facts.moves;
    return slice;
}

// The object roads of a slice that changes tools, on exactly the fine and the coarse slot's tools.
std::map<unsigned char, ToolPrint> two_tool_print(const PlateSlice &slice, unsigned char fine, unsigned char coarse)
{
    REQUIRE(fine != coarse);
    const EmittedFacts facts = extract_emitted_facts(slice.gcode);
    CHECK_FALSE(facts.tool_changes.empty());
    const auto tools = object_print_by_tool(facts);
    REQUIRE(tools.count(fine) == 1);
    REQUIRE(tools.count(coarse) == 1);
    CHECK(tools.size() == 2);
    return tools;
}

void check_feature_split_print(const PlateSlice &slice, std::size_t fine_slot, std::size_t coarse_slot,
                               double fine_height, double coarse_height)
{
    const unsigned char fine = physical_of(slice, fine_slot), coarse = physical_of(slice, coarse_slot);
    const auto tools = two_tool_print(slice, fine, coarse);
    CHECK(tools.at(fine).roles.count(erExternalPerimeter) == 1);
    CHECK(tools.at(coarse).roles.count(erInternalInfill) == 1);
    CHECK_THAT(tools.at(fine).max_wall_height, Catch::Matchers::WithinAbs(fine_height, kHeightTolerance));
    CHECK_THAT(tools.at(coarse).max_infill_height, Catch::Matchers::WithinAbs(coarse_height, kHeightTolerance));
}

void check_body_split_print(const PlateSlice &slice, std::size_t fine_slot, std::size_t coarse_slot,
                            double fine_height, double coarse_height, bool fine_on_left = true)
{
    const unsigned char fine = physical_of(slice, fine_slot), coarse = physical_of(slice, coarse_slot);
    const auto tools = two_tool_print(slice, fine, coarse);
    CHECK_THAT(tools.at(fine).max_wall_height, Catch::Matchers::WithinAbs(fine_height, kHeightTolerance));
    CHECK_THAT(tools.at(coarse).max_wall_height, Catch::Matchers::WithinAbs(coarse_height, kHeightTolerance));

    // Each body's outer walls stand on its own side.
    float fine_left = std::numeric_limits<float>::max(), fine_right = std::numeric_limits<float>::lowest();
    float coarse_left = fine_left, coarse_right = fine_right;
    for (const auto &move : slice.moves) {
        if (move.type != EMoveType::Extrude || move.extrusion_role != erExternalPerimeter)
            continue;
        if (move.physical_tool_id == fine) {
            fine_left = std::min(fine_left, move.position.x());
            fine_right = std::max(fine_right, move.position.x());
        } else if (move.physical_tool_id == coarse) {
            coarse_left = std::min(coarse_left, move.position.x());
            coarse_right = std::max(coarse_right, move.position.x());
        }
    }
    if (fine_on_left)
        CHECK(fine_right < coarse_left);
    else
        CHECK(coarse_right < fine_left);
}

// The tower choice setup reopens on (Change... opens More options on, for a project).
WizardTowerIntent tower_intent(const PresetBundle &bundle, const DynamicPrintConfig &plate_config)
{
    DynamicPrintConfig full = bundle.full_config();
    full.apply(plate_config);
    const auto *mask = full.option<ConfigOptionStrings>("different_settings_to_system");
    return mixed_nozzle_default_tower_intent(bundle.project_config, mask == nullptr ? std::vector<std::string>{} : mask->values,
                                             full.opt_enum<MixedNozzleSlicingMode>("mixed_nozzle_slicing_mode"));
}

SetupChoice feature_choice(double fine_height, int ratio)
{
    SetupChoice choice;
    choice.mode = MixedNozzleSlicingMode::FeatureSplit;
    choice.fine_height = fine_height;
    choice.ratio = ratio;
    return choice;
}

SetupChoice body_choice(double fine_height, int ratio)
{
    SetupChoice choice = feature_choice(fine_height, ratio);
    choice.mode = MixedNozzleSlicingMode::BodySplit;
    return choice;
}

// Stages and commits a Plate Settings edit as the dialog does; true when it committed with one
// Undo snapshot.
bool commit_binding(WizardProject &project, const MixedNozzleAssignmentBindingRequest &request)
{
    auto plan = stage_mixed_nozzle_assignment_binding(project.bundle, project.model, project.plate, request);
    int snapshots = 0;
    MixedNozzleAssignmentBindingHooks hooks;
    hooks.take_snapshot = [&snapshots] { ++snapshots; };
    hooks.take_snapshot_with_setup_transition = [&snapshots](const MixedNozzleSetupPlan &) {
        ++snapshots;
        return true;
    };
    hooks.record_cadence_transition = [](std::optional<double>, std::optional<double>) {};
    hooks.notify_batch_changed = [] {};
    MixedNozzleAssignmentBindingDiagnostic diagnostic = MixedNozzleAssignmentBindingDiagnostic::None;
    INFO("stage diagnostic " << int(plan.diagnostic));
    const bool committed = commit_mixed_nozzle_assignment_binding(project.bundle, project.model, project.owners.plates,
                                                                  plan, hooks, &diagnostic);
    INFO("commit diagnostic " << int(diagnostic));
    if (committed)
        CHECK(snapshots == 1);
    return committed;
}

// The plate's full config, and the resolver `from_full` builds from it with the plate's own filament map.
template<class FromFull>
std::pair<DynamicPrintConfig, PrintConfig> editor_configs(PresetBundle &bundle, PartPlate &plate, FromFull from_full)
{
    const auto maps = plate.get_real_filament_maps(bundle.project_config);
    DynamicPrintConfig full = bundle.full_config(true, maps, plate.get_real_filament_volume_maps(bundle.project_config));
    PrintConfig resolver = from_full(full);
    resolver.filament_map_mode.value = plate.get_real_filament_map_mode(bundle.project_config);
    resolver.filament_map.values = maps;
    return {std::move(full), std::move(resolver)};
}

void commit_feature_editor(WizardProject &project, std::size_t fine_slot, std::size_t coarse_slot,
                           double coarse_height)
{
    PresetBundle &bundle = project.bundle;
    PartPlate &plate = project.plate;
    const auto [full, resolver] = editor_configs(bundle, plate, feature_split_resolver_config_from_full);
    const MixedNozzleSlicingMode mode = effective_mixed_nozzle_mode(bundle.project_config, plate).effective;
    REQUIRE(mode == MixedNozzleSlicingMode::FeatureSplit);
    const std::size_t nozzle_count = full.option<ConfigOptionFloats>("nozzle_diameter")->values.size();
    const FeatureSplitStableTarget target {plate.get_index(), plate.id().id};
    auto live = build_feature_split_native_live_apply(target, {int(fine_slot), int(coarse_slot)},
        plate.get_index(), plate.id().id, mode, nozzle_count, bundle.filament_presets.size(), resolver);
    REQUIRE(live.request.has_value());
    live.request->coarse_layer_height = coarse_height;
    const FeatureSplitApplyRequest feature_request = *live.request;

    MixedNozzleAssignmentBindingRequest request;
    request.stage_assignment = [feature_request, mode, nozzle_count](
                                   MixedNozzleAssignmentBindingStageState &state) {
        const auto [staged_full, staged_resolver] = editor_configs(state.bundle, state.plate, feature_split_resolver_config_from_full);
        const double base = staged_full.has("layer_height") ? staged_full.opt_float("layer_height") : 0.;
        const FeatureSplitApplyResult staged = commit_feature_split_editor_request(
            state.bundle.project_config, state.bundle.prints.get_edited_preset().config, state.plate, mode,
            state.plate.get_index(), feature_request.target.plate_id, nozzle_count, staged_resolver,
            feature_split_coarse_cadence_choices(base, staged_resolver), feature_request);
        return MixedNozzleAssignmentStageResult{staged.applied, staged.changed};
    };
    REQUIRE(commit_binding(project, request));
}

struct BodyEdit {
    std::size_t slot;
    double height;
};

bool commit_body_editor(WizardProject &project, const std::map<std::string, BodyEdit> &edits)
{
    PresetBundle &bundle = project.bundle;
    PartPlate &plate = project.plate;
    const auto [full, resolver] = editor_configs(bundle, plate, mixed_nozzle_resolver_config_from_full);
    const double inherited = full.has("layer_height") ? full.opt_float("layer_height") : 0.;
    const BodySplitEditorModel live = build_body_split_editor_model(plate.get_objects_on_this_plate(), resolver, {},
                                                                    inherited);
    REQUIRE(live.rows.size() == 2);
    std::vector<BodySplitNativeSelection> selections;
    for (const BodySplitEditorRow &row : live.rows) {
        REQUIRE(edits.count(row.volume_name) == 1);
        const BodyEdit &edit = edits.at(row.volume_name);
        int cadence = -1;
        for (std::size_t i = 0; i < row.cadence_choices.size(); ++i)
            if (std::abs(row.cadence_choices[i] - edit.height) < 1e-9)
                cadence = int(i);
        INFO("the editor offers no " << edit.height << " mm layer for " << row.volume_name);
        REQUIRE(cadence >= 0);
        selections.push_back({row.object_id, row.volume_id, int(edit.slot), cadence});
    }
    const BodySplitLiveRequest live_request = build_body_split_live_request(live.rows, selections,
        bundle.filament_presets.size(), resolver, RegionalGridPhaseRule::KeepNominal);
    REQUIRE(live_request.request.has_value());
    BodySplitApplyRequest body_request = *live_request.request;
    body_request.inherited_layer_height = inherited;

    MixedNozzleAssignmentBindingRequest request;
    request.stage_assignment = [body_request](MixedNozzleAssignmentBindingStageState &state) {
        const PrintConfig staged_resolver = editor_configs(state.bundle, state.plate, mixed_nozzle_resolver_config_from_full).second;
        const BodySplitApplyResult staged = apply_body_split_editor_request(
            state.model, *state.plate.config(), staged_resolver, body_request);
        return MixedNozzleAssignmentStageResult{staged.applied, staged.changed};
    };
    return commit_binding(project, request);
}

ProfileSpec j1_spec()
{
    ProfileSpec spec;
    spec.vendor = "Snapmaker";
    spec.printer = "Snapmaker J1 (0.2 nozzle)";
    spec.process = "0.10 Standard @Snapmaker J1 (0.2 nozzle)";
    spec.materials = {"Snapmaker J1 PLA", "Snapmaker J1 PLA"};
    spec.nozzles = {0.2, 0.6};
    spec.min_heights = {0.06, 0.15};
    spec.max_heights = {0.14, 0.42};
    spec.bed = Vec3d(324., 200., 200.);
    spec.tower = Vec2d(180., 80.);
    return spec;
}

ProfileSpec ratrig_idex_spec()
{
    ProfileSpec spec;
    spec.vendor = "Ratrig";
    spec.printer = "RatRig V-Core 4 IDEX 300 0.4 nozzle";
    spec.process = "0.20mm Quality @RatRig V-Core 4 IDEX 0.4";
    spec.materials = {"RatRig Generic PLA", "RatRig Generic PLA"};
    spec.nozzles = {0.4, 0.8};
    spec.min_heights = {0.06, 0.12};
    spec.max_heights = {0.30, 0.50};
    spec.bed = Vec3d(300., 300., 300.);
    spec.tower = Vec2d(240., 150.);
    return spec;
}

ProfileSpec four_tool_toolchanger_spec()
{
    ProfileSpec spec;
    spec.vendor = "Custom";
    spec.printer = "MyToolChanger 0.2 nozzle";
    spec.process = "0.12mm Fine @MyToolChanger";
    spec.materials.assign(4, "Generic PLA @MyToolChanger");
    spec.nozzles = {0.4, 0.2, 0.4, 0.6};
    spec.min_heights = {0.08, 0.04, 0.08, 0.12};
    spec.max_heights = {0.32, 0.16, 0.32, 0.42};
    spec.bed = Vec3d(350., 350., 300.);
    spec.tower = Vec2d(240., 150.);
    return spec;
}

} // namespace

TEST_CASE("Feature Split set up by the wizard slices walls fine and sparse infill coarse",
          "[TestRebuild][E2E][FeatureSplit][Tower][TowerIntent][Advisor][Reuse]")
{
    // The part on slot 1, or on no slot of its own.
    const int part_slot = GENERATE(1, 0);
    CAPTURE(part_slot);
    WizardProject project(h2d_02_06_spec());
    project.add_object("box", {{"box", Vec3d(180., 180., 12.), Vec3d::Zero(), part_slot}}, Vec2d(20., 20.));
    REQUIRE(tower_intent(project.bundle, *project.plate.config()) == WizardTowerIntent::Automatic);
    run_wizard_setup(project, feature_choice(0.10, 4));
    const PlateSlice slice = require_slice(project);
    check_feature_split_print(slice, 0, 1, 0.10, 0.40);
    // The Automatic tower: unrotated, at most 35 mm wide, with the engine's own brim.
    REQUIRE(slice.config.opt_bool("enable_prime_tower"));
    CHECK(slice.config.opt_float("wipe_tower_rotation_angle") == 0.);
    CHECK(slice.config.opt_float("prime_tower_brim_width") < 0.);
    const TowerFootprint tower = tower_footprint(slice.moves);
    REQUIRE(tower.moves > 0);
    CHECK(tower.upper_width <= 35.f + 0.5f);
    CHECK(tower.first_layer_width >= tower.upper_width);
    CHECK_THAT(slice.tower_brim_width, Catch::Matchers::WithinAbs(3., 0.25));
    // Setup reopens on the Automatic tower.
    CHECK(tower_intent(project.bundle, *project.plate.config()) == WizardTowerIntent::Automatic);
    if (part_slot == 0)
        return;

    // Turning the mode off clears the decision report on the reused Print as soon as Apply invalidates it, before the
    // Off slice has processed any layers.
    Print print;
    print.set_status_silent();
    print.is_BBL_printer() = true;
    const CadenceTest::Facts mixed = slice_reused(print, project);
    REQUIRE_FALSE(mixed.emitted.role_facts.empty());
    REQUIRE_FALSE(print.objects().empty());
    MixedNozzleDecisionInput input;
    auto observations = print.objects().front()->mixed_nozzle_advisor_observations();
    input.observations = observations.get();
    const MixedNozzleDecisionSummary before = build_decision_summary(input);
    REQUIRE(before.available);
    REQUIRE(before.band_count > 0);
    turn_off_current_plate(project);
    print.apply(project.model, workflow_print_config(project));
    REQUIRE_FALSE(print.objects().empty());
    observations = print.objects().front()->mixed_nozzle_advisor_observations();
    input.observations = observations.get();
    const MixedNozzleDecisionSummary after = build_decision_summary(input);
    CHECK_FALSE(after.available);
    CHECK(after.band_count == 0);
    const CadenceTest::Facts ordinary = CadenceTest::slice(print);
    INFO(ordinary.refusal.string);
    REQUIRE(ordinary.refusal.string.empty());
    REQUIRE_FALSE(ordinary.emitted.role_facts.empty());
    CHECK(ordinary.gcode != mixed.gcode);
}

TEST_CASE("Body Split set up by the wizard slices each body on its own nozzle at its own layer, also after reopening",
          "[TestRebuild][E2E][BodySplit][Wizard][Persistence][Materials][TowerIntent]")
{
    WizardProject project(h2d_02_06_spec());
    add_two_bodies(project);
    run_wizard_setup(project, body_choice(0.10, 3));
    REQUIRE(project.plate.config()->has("mixed_nozzle_slicing_mode"));
    const PlateSlice slice = require_slice(project);
    check_body_split_print(slice, 0, 1, 0.10, 0.30);
    // The sidebar's Sliced line reads this slice's nozzle changes and time.
    MixedNozzleSidebarFacts facts;
    mixed_nozzle_sidebar_read_slice_statistics(slice.statistics, facts);
    REQUIRE(facts.nozzle_changes);
    CHECK(*facts.nozzle_changes > 0);
    CHECK(facts.print_seconds.value_or(0.) > 0.);

    {
        // A 3MF saved without the per-preset dirty-key mask, the project default Off and the plate Body Split: its
        // material values belong to the bound materials, not the stock preset, and the plate slices on them.
        DynamicPrintConfig saved = project.bundle.full_config(false);
        saved.set_key_value("mixed_nozzle_slicing_mode", new ConfigOptionEnum<MixedNozzleSlicingMode>(MixedNozzleSlicingMode::Off));
        auto *flow = saved.option<ConfigOptionFloats>("filament_flow_ratio", true);
        REQUIRE((flow != nullptr && flow->values.size() >= 2));
        std::fill(flow->values.begin(), flow->values.end(), 0.93);
        for (const char *key : {"nozzle_temperature", "nozzle_temperature_initial_layer"}) {
            auto *temperature = saved.option<ConfigOptionInts>(key, true);
            REQUIRE((temperature != nullptr && temperature->values.size() >= 2));
            std::fill(temperature->values.begin(), temperature->values.end(), 237);
        }
        saved.erase("different_settings_to_system");
        for (const char *key : {"filament_flow_ratio", "nozzle_temperature", "nozzle_temperature_initial_layer"})
            mixed_nozzle_ledger_add_key(saved, 2, {0, 1}, key);

        ScopedTemporaryDir backup("rebuild_plate_material_3mf");
        project.model.set_backup_path(backup.string());
        ScopedTemporaryFile archive(".3mf");
        const std::string path = archive.string();
        PlateData plate;
        plate.plate_index = 0;
        plate.config = *project.plate.config();
        plate.filament_maps = project.plate.get_real_filament_maps(project.bundle.project_config);
        save_project_3mf(project.model, saved, plate, path);
        const ReopenedProject loaded(path);
        REQUIRE(loaded.config.opt_enum<MixedNozzleSlicingMode>("mixed_nozzle_slicing_mode") == MixedNozzleSlicingMode::Off);
        REQUIRE(loaded.plate_config.opt_enum<MixedNozzleSlicingMode>("mixed_nozzle_slicing_mode") ==
                MixedNozzleSlicingMode::BodySplit);

        PresetBundle reopened = installed_bbl_profiles();
        reopened.load_config_model(path, loaded.config);
        DynamicPrintConfig effective = reopened.full_config();
        const auto *reopened_flow = effective.option<ConfigOptionFloats>("filament_flow_ratio");
        const auto *reopened_temperature = effective.option<ConfigOptionInts>("nozzle_temperature");
        REQUIRE((reopened_flow != nullptr && reopened_flow->values.size() >= 2));
        REQUIRE((reopened_temperature != nullptr && reopened_temperature->values.size() >= 2));
        for (std::size_t slot = 0; slot < 2; ++slot) {
            CHECK_THAT(reopened_flow->values[slot], Catch::Matchers::WithinAbs(0.93, 1e-5));
            CHECK(reopened_temperature->values[slot] == 237);
        }
        effective.apply(loaded.plate_config);
        check_body_split_print(slice_reopened(loaded, effective), 0, 1, 0.10, 0.30);
    }

    // An older saved project keeps its hand-off ramming length (12) after the vendor profile changed it (now 4), though
    // its mask flagged the key for no filament: reopened, saved and reopened again, and with a blanket ledger from an
    // older Rebind, which recorded every ledger key as explicit.
    const size_t n = project.bundle.filament_presets.size();
    REQUIRE(n >= 2);
    DynamicPrintConfig older = project.bundle.full_config(false);
    auto *ram = older.option<ConfigOptionFloats>("filament_change_length", true);
    REQUIRE((ram != nullptr && !ram->values.empty()));
    std::fill(ram->values.begin(), ram->values.end(), 12.);
    older.option<ConfigOptionStrings>("different_settings_to_system", true)->values.assign(mixed_nozzle_mask_length(n), std::string());
    const auto reopen = [](const DynamicPrintConfig &config) {
        PresetBundle bundle = installed_bbl_profiles();
        bundle.load_config_model("tower-intent.3mf", config);
        return bundle;
    };
    const auto check_saved_value_kept = [n](const DynamicPrintConfig &full) {
        const auto *loaded = full.option<ConfigOptionFloats>("filament_change_length");
        REQUIRE((loaded != nullptr && loaded->values.size() >= n));
        for (size_t i = 0; i < n; ++i)
            CHECK_THAT(loaded->values[i], Catch::Matchers::WithinAbs(12., 1e-6));
    };
    PresetBundle reopened = reopen(older);
    check_saved_value_kept(reopened.full_config());
    check_saved_value_kept(reopen(reopened.full_config(false)).full_config());
    DynamicPrintConfig blanket = older;
    const MixedNozzleLedger ledger = mixed_nozzle_read_ledger(blanket, n);
    REQUIRE(ledger.qualified);
    std::vector<std::set<std::string>> sets(n, mixed_nozzle_ledger_key_set());
    for (auto &set : sets)
        set.erase("filament_extruder_variant");
    mixed_nozzle_write_ledger_sets(blanket, sets, ledger.provenance);
    check_saved_value_kept(reopen(blanket).full_config());
}

TEST_CASE("X2D Body Split prints its fine body on the Bowden tool",
          "[TestRebuild][E2E][BodySplit][X2D]")
{
    ProfileSpec spec;
    spec.vendor = "BBL";
    spec.printer = "Bambu Lab X2D 0.2 nozzle";
    spec.process = "0.10mm Standard @BBL X2D 0.2 nozzle";
    spec.materials = {"Bambu PLA Basic @BBL X2D 0.2 nozzle",
                      "Bambu PLA Basic @BBL X2D 0.2 nozzle"};
    spec.also_visible_materials = {"Bambu PLA Basic @BBL X2D"};
    spec.nozzles = {0.6, 0.2};
    spec.min_heights = {0.12, 0.04};
    spec.max_heights = {0.42, 0.14};
    spec.bed = Vec3d(256., 256., 256.);
    spec.tower = Vec2d(190., 190.);
    WizardProject project(spec);
    add_two_bodies(project);
    SetupChoice choice = body_choice(0.12, 3);
    choice.fine_slot = 1;
    choice.coarse_slot = 0;
    run_wizard_setup(project, choice);
    const PlateSlice slice = require_slice(project);
    check_body_split_print(slice, 1, 0, 0.12, 0.36, false);
    CHECK(physical_of(slice, 1) == 1);
    CHECK(physical_of(slice, 0) == 0);
}

TEST_CASE("Body Split at a cadence the Body preset's beams do not fit applies with interlocking beams chosen",
          "[TestRebuild][E2E][BodySplit][Joining]")
{
    WizardProject project(h2d_02_06_spec());
    add_two_bodies(project);
    SetupChoice choice = body_choice(0.10, 2);
    choice.joining = JoiningChoice::Beams;
    run_wizard_setup(project, choice);
    const PlateSlice slice = require_slice(project);
    const ModelObject *object = project.model.objects.front();
    CHECK(object->config.get().opt_bool("interlocking_beam"));
    CHECK(object->config.get().opt_int("interlocking_beam_layer_count") == 2);
    check_body_split_print(slice, 0, 1, 0.10, 0.20);
}

TEST_CASE("A Plate Settings material and coarse layer change after Feature Split setup slices the core on them",
          "[TestRebuild][E2E][FeatureSplit][Editor]")
{
    ProfileSpec spec = h2d_02_06_spec();
    spec.materials.push_back(spec.also_visible_materials.front());
    WizardProject project(spec);
    add_cube(project);
    run_wizard_setup(project, feature_choice(0.10, 3));
    // The core moves to slot 3 at 0.40 mm. On this cube the engine prices setup's 0.30 mm as slower
    // than all fine and prints no coarse roads, while 0.40 mm earns them.
    commit_feature_editor(project, 0, 2, 0.40);
    const PlateSlice slice = require_slice(project);
    check_feature_split_print(slice, 0, 2, 0.10, 0.40);
    const EmittedFacts facts = extract_emitted_facts(slice.gcode);
    CHECK(std::any_of(facts.role_facts.begin(), facts.role_facts.end(), [](const auto &fact) {
        return fact.role == erInternalInfill && fact.logical_filament == 2;
    }));
}

TEST_CASE("A Plate Settings body edit after Body Split setup slices each body as edited",
          "[TestRebuild][E2E][BodySplit][Editor]")
{
    WizardProject project(h2d_02_06_spec());
    add_two_bodies(project);
    run_wizard_setup(project, body_choice(0.10, 3));
    SECTION("the bodies swap nozzles") {
        REQUIRE(commit_body_editor(project, {{"left body", {1, 0.30}}, {"right body", {0, 0.10}}}));
        check_body_split_print(require_slice(project), 0, 1, 0.10, 0.30, false);
    }
    SECTION("a coarse layer change keeps the beams sliceable") {
        REQUIRE(commit_body_editor(project, {{"left body", {0, 0.10}}, {"right body", {1, 0.20}}}));
        check_body_split_print(require_slice(project), 0, 1, 0.10, 0.20);
    }
}

TEST_CASE("Rebinding a Body material refreshes the emitted cells on the same Print",
          "[TestRebuild][E2E][BodySplit][Editor][Reuse]")
{
    const ProfileSpec spec = four_tool_toolchanger_spec();
    WizardProject project(spec);
    DynamicPrintConfig &printer = project.bundle.printers.get_edited_preset().config;
    // The installed profile has five heads. Resize every per-head list to the fitted four,
    // as the printer tab does, before setting their diameters.
    printer.set_num_extruders(unsigned(spec.nozzles.size()));
    printer.set_key_value("nozzle_diameter", new ConfigOptionFloats(spec.nozzles));
    printer.set_key_value("min_layer_height", new ConfigOptionFloats(spec.min_heights));
    printer.set_key_value("max_layer_height", new ConfigOptionFloats(spec.max_heights));
    DynamicPrintConfig &process = project.bundle.prints.get_edited_preset().config;
    process.set_key_value("min_layer_height", new ConfigOptionFloats(spec.min_heights));
    process.set_key_value("max_layer_height", new ConfigOptionFloats(spec.max_heights));
    process.set_key_value("initial_layer_print_height", new ConfigOptionFloat(.12));
    project.bundle.prints.load_preset("", "Rebuild fitted 0.12 @MyToolChanger", process, true);
    project.add_object("two bodies", {{"left body", Vec3d(10., 20., 8.), Vec3d::Zero(), 2},
                                      {"right body", Vec3d(10., 20., 8.), Vec3d(15., 0., 0.), 4}},
                       Vec2d(100., 100.));
    SetupChoice choice = body_choice(.12, 2);
    choice.fine_slot = 1;   // physical head 1, 0.2 mm
    choice.coarse_slot = 3; // physical head 3, 0.6 mm
    choice.joining = JoiningChoice::Off;
    choice.keep_current_process = true;
    run_wizard_setup(project, choice);
    Print print;
    print.set_status_silent();
    print.is_BBL_printer() = false;
    const auto reslice = [&] {
        const DynamicPrintConfig config = workflow_print_config(project);
        print.apply(project.model, config);
        const CadenceTest::Facts facts = CadenceTest::slice(print);
        INFO(facts.refusal.string);
        REQUIRE(facts.refusal.string.empty());
        REQUIRE_FALSE(facts.gcode.empty());
        return facts;
    };
    const auto wall_tools = [](const CadenceTest::Facts &facts, double x_min, double x_max) {
        std::set<unsigned> tools;
        for (const auto &move : facts.moves)
            if (move.type == EMoveType::Extrude && move.extrusion_role == erExternalPerimeter &&
                move.position.x() > x_min && move.position.x() < x_max &&
                move.position.y() > 99. && move.position.y() < 121. &&
                move.position.z() > .25f)
                tools.insert(move.physical_tool_id);
        return tools;
    };
    const CadenceTest::Facts before = reslice();
    REQUIRE(wall_tools(before, 99., 111.) == std::set<unsigned>{1});
    REQUIRE(wall_tools(before, 114., 126.) == std::set<unsigned>{3});

    // Move only the left body from the 0.2 mm head to the first 0.4 mm head (both lay 0.12 mm): the second export
    // on this same Print must rebuild native ownership.
    REQUIRE(commit_body_editor(project, { {"left body", {0, .12}},
                                          {"right body", {3, .24}} }));
    const CadenceTest::Facts after = reslice();
    CHECK(wall_tools(after, 99., 111.) == std::set<unsigned>{0});
    CHECK(wall_tools(after, 114., 126.) == std::set<unsigned>{3});
    CHECK_FALSE(after.emitted.tool_changes.empty());
    CHECK(after.gcode != before.gcode);
}

namespace {

// A plate whose second filament first prints at Z 3: a 4 mm body on slot 1 with a 1 mm top inlay on
// slot 2, like text set into the top of a part.
void add_top_inlay(WizardProject &project)
{
    project.add_object("inlay", {{"body", Vec3d(30., 20., 4.), Vec3d::Zero(), 1},
                                 {"top", Vec3d(10., 10., 1.), Vec3d(10., 5., 3.), 2}},
                       Vec2d(100., 100.));
}

// Layer changes before the first prime-tower block: 1 when the tower starts on the first layer.
std::size_t layers_before_first_tower_block(const std::string &gcode)
{
    const std::size_t tower = gcode.find("; WIPE_TOWER_START");
    if (tower == std::string::npos)
        return 0;
    std::size_t layers = 0;
    for (std::size_t at = gcode.find("; CHANGE_LAYER"); at != std::string::npos && at < tower;
         at = gcode.find("; CHANGE_LAYER", at + 1))
        ++layers;
    return layers;
}

void check_off_tower_starts_on_first_layer(WizardProject &project)
{
    // Stock OrcaSlicer hides "No sparse layers" here; left on it would hold the tower back to the first change.
    CHECK_FALSE(project.bundle.prints.get_edited_preset().config.opt_bool("wipe_tower_no_sparse_layers"));
    const PlateSlice slice = require_slice(project);
    REQUIRE(slice.gcode.find("; WIPE_TOWER_START") != std::string::npos);
    CHECK(layers_before_first_tower_block(slice.gcode) == 1);
}

} // namespace

TEST_CASE("Turn off puts back the prime tower settings and role filaments setup changed, and the stock tower starts on layer 1",
          "[TestRebuild][E2E][Off][Tower]")
{
    SECTION("Feature Split on the stock process") {
        // The stock H2D process has tower interface features on: setup turns them off, Turn off puts them back.
        const std::string process = "0.10mm Standard @BBL H2D 0.2 nozzle";
        WizardProject project(h2d_02_06_spec());
        const auto edited = [&project]() -> DynamicPrintConfig & {
            return project.bundle.prints.get_edited_preset().config;
        };
        REQUIRE(edited().opt_bool("enable_tower_interface_features"));
        const double width_before = edited().opt_float(MIXED_NOZZLE_TOWER_WIDTH_KEY);
        add_top_inlay(project);
        SetupChoice choice = feature_choice(0.10, 3);
        choice.keep_current_process = true;
        run_wizard_setup(project, choice);
        REQUIRE(project.bundle.prints.get_edited_preset().name == process);
        CHECK_FALSE(edited().opt_bool("enable_tower_interface_features"));
        // Automatic owns it like the other tower settings, so Undo and Turn off restore it from the ledger.
        const auto ledger = MixedNozzleTowerPolicyLedger::parse(
            project.bundle.project_config.opt_string(MIXED_NOZZLE_TOWER_POLICY_OPTION));
        REQUIRE(ledger.has_value());
        CHECK(ledger->managed_keys.count(MIXED_NOZZLE_TOWER_INTERFACE_KEY) == 1);
        // Setup gives the plate its own fine and coarse role filaments; Off prints every feature with the
        // object's filament, as stock OrcaSlicer does, so Turn off takes them away again.
        CHECK(project.plate.config()->has("sparse_infill_filament_id"));
        double width_after = width_before;
        SECTION("untouched tower settings return to the preset's own values") {}
        SECTION("a tower width the user set after setup stays") {
            detach_mixed_nozzle_tower_key(project.bundle.project_config, MIXED_NOZZLE_TOWER_WIDTH_KEY, process);
            edited().set_key_value(MIXED_NOZZLE_TOWER_WIDTH_KEY, new ConfigOptionFloat(60.));
            width_after = 60.;
        }
        turn_off_current_plate(project);
        for (const char *role : {"outer_wall_filament_id", "inner_wall_filament_id", "internal_solid_filament_id",
                                 "top_surface_filament_id", "bottom_surface_filament_id", "sparse_infill_filament_id"}) {
            CAPTURE(role);
            CHECK_FALSE(project.plate.config()->has(role));
        }
        REQUIRE(project.bundle.prints.get_edited_preset().name == process);
        CHECK(edited().opt_bool("enable_tower_interface_features"));
        CHECK_THAT(edited().opt_float(MIXED_NOZZLE_TOWER_WIDTH_KEY), Catch::Matchers::WithinAbs(width_after, 0.01));
        check_off_tower_starts_on_first_layer(project);
    }
    SECTION("Feature Split on a process preset with ooze prevention on") {
        // Setup turns ooze prevention off outside the ledger; Turn off gives the preset's value back.
        WizardProject project(h2d_02_06_spec());
        const std::string process = project.bundle.prints.get_edited_preset().name;
        Preset *stored = project.bundle.prints.find_preset(process, false, true);
        REQUIRE(stored != nullptr);
        stored->config.set_key_value("ooze_prevention", new ConfigOptionBool(true));
        project.bundle.prints.get_edited_preset().config.set_key_value("ooze_prevention", new ConfigOptionBool(true));
        add_top_inlay(project);
        SetupChoice choice = feature_choice(0.10, 3);
        choice.keep_current_process = true;
        run_wizard_setup(project, choice);
        CHECK_FALSE(project.bundle.prints.get_edited_preset().config.opt_bool("ooze_prevention"));
        turn_off_current_plate(project);
        CHECK(project.bundle.prints.get_edited_preset().config.opt_bool("ooze_prevention"));
    }
    SECTION("Body Split on an MN Feature process, whose tower holds back sparse layers") {
        WizardProject project(h2d_02_06_spec());
        REQUIRE(project.bundle.prints.select_preset_by_name("MN Feature 0.2-0.6 0.10-0.30 Standard @BBL", true));
        REQUIRE(project.bundle.prints.get_edited_preset().config.opt_bool("wipe_tower_no_sparse_layers"));
        add_top_inlay(project);
        SetupChoice choice = body_choice(0.10, 3);
        choice.keep_current_process = true;
        run_wizard_setup(project, choice);
        turn_off_current_plate(project);
        check_off_tower_starts_on_first_layer(project);
    }
}

TEST_CASE("Snapmaker J1 and RatRig V-Core 4 IDEX slice after Feature Split and Body Split setup",
          "[TestRebuild][E2E][OtherPrinters]")
{
    const bool j1 = GENERATE(true, false);
    CAPTURE(j1);
    WizardProject project(j1 ? j1_spec() : ratrig_idex_spec());
    SECTION("Feature Split") {
        if (j1)
            add_cube(project, Vec2d(30., 30.), Vec3d(80., 80., 30.));
        else
            add_cube(project);
        run_wizard_setup(project, feature_choice(j1 ? 0.10 : 0.08, j1 ? 3 : 5));
        check_feature_split_print(require_slice(project), 0, 1, j1 ? 0.10 : 0.08, j1 ? 0.30 : 0.40);
    }
    SECTION("Body Split") {
        // The J1's own process has Arachne walls.
        if (j1)
            REQUIRE(project.bundle.full_config().opt_enum<PerimeterGeneratorType>("wall_generator") ==
                    PerimeterGeneratorType::Arachne);
        add_two_bodies(project, j1 ? Vec2d(80., 80.) : Vec2d(80., 100.));
        run_wizard_setup(project, body_choice(j1 ? 0.10 : 0.12, 2));
        check_body_split_print(require_slice(project), 0, 1, j1 ? 0.10 : 0.12, j1 ? 0.20 : 0.24);
    }
}

TEST_CASE("Four-tool Body setup uses nonadjacent fine and coarse heads and a published material selection",
          "[TestRebuild][E2E][BodySplit][Toolchanger][Preset]")
{
    const ProfileSpec spec = four_tool_toolchanger_spec();
    WizardProject project(spec);
    // The installed toolchanger supplies the machine settings; this user's four
    // fitted nozzles are 0.4 / 0.2 / 0.4 / 0.6 mm. The parked heads are narrower
    // than the coarse head, so tower planning can use the printed pair.
    DynamicPrintConfig &printer = project.bundle.printers.get_edited_preset().config;
    // The installed profile has five heads. Resize every per-head list to the fitted four,
    // as the printer tab does, before setting their diameters.
    printer.set_num_extruders(unsigned(spec.nozzles.size()));
    printer.set_key_value("nozzle_diameter", new ConfigOptionFloats(spec.nozzles));
    printer.set_key_value("min_layer_height", new ConfigOptionFloats(spec.min_heights));
    printer.set_key_value("max_layer_height", new ConfigOptionFloats(spec.max_heights));
    // The published process starts as a 0.2 mm single-head profile. Give its
    // four fitted heads the same explicit operating limits as the machine;
    // otherwise process composition can retain the original fine-head max.
    DynamicPrintConfig &process = project.bundle.prints.get_edited_preset().config;
    process.set_key_value("min_layer_height", new ConfigOptionFloats(spec.min_heights));
    process.set_key_value("max_layer_height", new ConfigOptionFloats(spec.max_heights));
    // Its stock 0.20 mm first layer exceeds the selected 0.2 mm head's
    // 0.16 mm maximum. Save a 0.12 mm process: wizard Apply restores the
    // selected preset's stored first layer, not unsaved editor changes.
    process.set_key_value("initial_layer_print_height", new ConfigOptionFloat(0.12));
    project.bundle.prints.load_preset("", "Rebuild fitted 0.12 @MyToolChanger", process, true);

    PresetCollection &materials = project.bundle.filaments;
    const std::string selected_material = "Rebuild PLA @MyToolChanger";
    DynamicPrintConfig material = materials.get_edited_preset().config;
    Preset &target = materials.load_preset("", selected_material, material, false);
    target.is_visible = true;
    material.set_key_value("nozzle_temperature", new ConfigOptionInts{235});
    material.set_key_value("nozzle_temperature_initial_layer", new ConfigOptionInts{235});
    auto prepared = materials.prepare_preset_selection(selected_material, material);
    REQUIRE(prepared);
    materials.apply_prepared_preset_selection(std::move(*prepared));
    project.bundle.filament_presets[1] = selected_material;
    REQUIRE(materials.get_edited_preset().name == selected_material);

    project.add_object("shared material bodies",
        {{"fine body", Vec3d(12., 18., 8.), Vec3d::Zero(), 2},
         {"coarse body", Vec3d(20., 18., 8.), Vec3d(20., 0., 0.), 2}}, Vec2d(100., 100.));
    const ModelObject *object = project.model.objects.front();
    std::vector<WizardBodyRoleRow> rows;
    for (const ModelVolume *volume : object->volumes) {
        WizardBodyRoleRow body;
        body.object_id = object->id().id;
        body.volume_id = volume->id().id;
        body.current_slot = 1;
        body.current_physical = 1;
        body.volume = volume->mesh().stats().volume;
        rows.push_back(body);
    }
    const auto roles = wizard_default_body_roles(rows, 1);
    const WizardPickerDefaults defaults = wizard_body_picker_defaults(
        rows, roles, spec.nozzles, {1, 2, 3, 4}, {"PLA", "PLA", "PLA", "PLA"},
        project.bundle.filament_presets, std::nullopt, std::nullopt);
    REQUIRE(defaults.fine);
    REQUIRE(defaults.coarse);
    CHECK(*defaults.fine == 1);
    CHECK(*defaults.coarse == 3);

    SetupChoice choice = body_choice(0.12, 2);
    choice.fine_slot = *defaults.fine;
    choice.coarse_slot = *defaults.coarse;
    choice.joining = JoiningChoice::Off;
    choice.keep_current_process = true;
    const DynamicPrintConfig entry = wizard_plate_effective_config(project.bundle, project.plate);
    INFO("fitted entry first=" << entry.opt_float("initial_layer_print_height")
         << " min=" << entry.opt_serialize("min_layer_height")
         << " max=" << entry.opt_serialize("max_layer_height")
         << " fine slot=" << choice.fine_slot << " coarse slot=" << choice.coarse_slot);
    run_wizard_setup(project, choice);
    const PlateSlice printed = require_slice(project);
    check_body_split_print(printed, 1, 3, 0.12, 0.24);
    CHECK(printed.config.option<ConfigOptionInts>("nozzle_temperature")->values[1] == 235);

    DynamicPrintConfig parked_widest = printed.config;
    parked_widest.set_key_value("nozzle_diameter", new ConfigOptionFloats{0.8, 0.2, 0.4, 0.6});
    Print rejected;
    rejected.set_status_silent();
    rejected.is_BBL_printer() = false;
    rejected.apply(project.model, parked_widest);
    CHECK(rejected.validate().opt_key == "nozzle_diameter");

    // An ordinary multipart neighbor happens to use a third physical head.
    // Its two volumes share one filament and have no regional cadence; they
    // must not be mistaken for another Body Split pair.
    project.add_object("ordinary neighbor",
        {{"left ordinary", Vec3d(10., 18., 4.), Vec3d::Zero(), 4},
         {"right ordinary", Vec3d(10., 18., 4.), Vec3d(14., 0., 0.), 4}},
        Vec2d(220., 40.));
    const PlateSlice with_neighbor = require_slice(project);
    const EmittedFacts original_plan = extract_emitted_facts(printed.gcode);
    const EmittedFacts neighbor_plan = extract_emitted_facts(with_neighbor.gcode);
    REQUIRE(original_plan.plan_grids.size() == 2);
    CHECK(neighbor_plan.plan_grids.size() == original_plan.plan_grids.size());
    size_t ordinary_roads = 0, wrong_ordinary = 0;
    for (const auto &move : with_neighbor.moves) {
        if (move.type != EMoveType::Extrude || move.extrusion_role != erExternalPerimeter ||
            move.position.x() < 219. || move.position.x() > 245. ||
            move.position.y() < 39. || move.position.y() > 59.)
            continue;
        ++ordinary_roads;
        wrong_ordinary += move.physical_tool_id != 3 || std::abs(move.height - .12f) > .011f;
    }
    CAPTURE(ordinary_roads, wrong_ordinary);
    CHECK(ordinary_roads > 0);
    CHECK(wrong_ordinary == 0);
}

TEST_CASE("H2D Arachne walls and elephant foot compensation on no layers survive Body Split setup and slice",
          "[TestRebuild][E2E][BodySplit][Arachne]")
{
    WizardProject project(h2d_02_06_spec());
    DynamicPrintConfig &process = project.bundle.prints.get_edited_preset().config;
    process.set_key_value("wall_generator", new ConfigOptionEnum<PerimeterGeneratorType>(PerimeterGeneratorType::Arachne));
    process.set_key_value("elefant_foot_compensation", new ConfigOptionFloat(0.1));
    process.set_key_value("elefant_foot_compensation_layers", new ConfigOptionInt(0));
    add_two_bodies(project);
    SetupChoice choice = body_choice(0.10, 3);
    choice.keep_current_process = true;
    run_wizard_setup(project, choice);
    const PlateSlice slice = require_slice(project);
    CHECK(slice.config.opt_enum<PerimeterGeneratorType>("wall_generator") == PerimeterGeneratorType::Classic);
    CHECK_THAT(slice.config.opt_float("elefant_foot_compensation"), Catch::Matchers::WithinAbs(0.1, 0.001));
    CHECK(slice.config.opt_int("elefant_foot_compensation_layers") == 0);
    check_body_split_print(slice, 0, 1, 0.10, 0.30);

    // Slicing refuses Arachne walls again, and beams at a layer count this cadence does not fit, by the
    // setting that prevents it.
    const auto refusal_key = [&project](const DynamicPrintConfig &config) {
        Print print;
        print.set_status_silent();
        print.is_BBL_printer() = true;
        print.apply(project.model, config);
        return print.validate().opt_key;
    };
    DynamicPrintConfig arachne = slice.config;
    arachne.set_key_value("wall_generator", new ConfigOptionEnum<PerimeterGeneratorType>(PerimeterGeneratorType::Arachne));
    CHECK(refusal_key(arachne) == "wall_generator");
    ModelObject *object = project.model.objects.front();
    object->config.set_key_value("interlocking_beam", new ConfigOptionBool(true));
    object->config.set_key_value("interlocking_beam_layer_count", new ConfigOptionInt(2));
    CHECK(refusal_key(slice.config) == "interlocking_beam_layer_count");
}

TEST_CASE("Setup opened from a nozzle change writes the project default even when More options says this plate",
          "[TestRebuild][E2E][FeatureSplit][ScopeLock]")
{
    ProfileSpec spec = h2d_02_06_spec();
    spec.nozzles = {0.2, 0.2};
    spec.min_heights = {0.04, 0.04};
    spec.max_heights = {0.14, 0.14};
    WizardProject project(spec);
    add_cube(project);
    SetupChoice choice = feature_choice(0.10, 4);
    choice.requested_nozzles = std::vector<double>{0.2, 0.6};
    choice.requested_min_heights = {0.04, 0.12};
    choice.requested_max_heights = {0.14, 0.42};
    choice.scope = WizardScopeKind::CurrentPlate;
    run_wizard_setup(project, choice);

    CHECK(mixed_nozzle_project_default(project.bundle.project_config) == MixedNozzleSlicingMode::FeatureSplit);
    CHECK(project.bundle.project_config.option<ConfigOptionFloats>("nozzle_diameter")->values ==
          std::vector<double>{0.2, 0.6});
    const PlateSlice slice = require_slice(project);
    check_feature_split_print(slice, 0, 1, 0.10, 0.40);
}

TEST_CASE("A wizard Body Split project retains its objects and printed bodies after Cadence 3MF import",
          "[TestRebuild][Wizard][Persistence][BodySplit]")
{
    WizardProject project(h2d_02_06_spec());
    const std::string process_name = "Process \"fine\" \\ custom";
    DynamicPrintConfig custom_process = project.bundle.prints.get_edited_preset().config;
    custom_process.set_key_value("prime_tower_width", new ConfigOptionFloat(60.));
    project.bundle.prints.load_preset("", process_name, custom_process, true);
    REQUIRE(project.bundle.prints.get_edited_preset().name == process_name);
    add_two_bodies(project);
    SetupChoice choice = body_choice(0.10, 3);
    choice.keep_current_process = true;
    run_wizard_setup(project, choice);
    REQUIRE(project.bundle.prints.get_edited_preset().name == process_name);
    REQUIRE(project.bundle.prints.get_edited_preset().config.opt_float("prime_tower_width") < 60.);
    PlateSlice original = require_slice(project);
    check_body_split_print(original, 0, 1, 0.10, 0.30);

    // Legacy projects can carry no process nozzle-role list. Preserve that
    // empty value through native persistence while the plate still prints.
    original.config.set_key_value("mixed_nozzle_process_nozzle_diameters", new ConfigOptionFloats{});
    project.plate.config()->set_key_value("mixed_nozzle_process_nozzle_diameters", new ConfigOptionFloats{});

    ScopedTemporaryDir backup("rebuild_workflow_3mf");
    project.model.set_backup_path(backup.string());
    ScopedTemporaryFile archive(".3mf"), branded(".3mf");
    PlateData plate;
    plate.plate_index = 0;
    plate.config = *project.plate.config();
    save_project_3mf(project.model, original.config, plate, archive.string());
    REQUIRE(mark_native_archive_as_cadence(archive.string(), branded.string()));
    {
        const ReopenedProject loaded(branded.string());
        REQUIRE(loaded.model.objects.size() == 1);
        CHECK(loaded.model.objects.front()->name == "two bodies");
        REQUIRE(loaded.model.objects.front()->instances.size() == 1);
        const Vec3d loaded_position = loaded.model.objects.front()->instances.front()->get_offset();
        CHECK_THAT(loaded_position.x(), Catch::Matchers::WithinAbs(100., .01));
        CHECK_THAT(loaded_position.y(), Catch::Matchers::WithinAbs(100., .01));
        const DynamicPrintConfig loaded_config = loaded.slice_config();
        const auto *roles = loaded_config.option<ConfigOptionFloats>("mixed_nozzle_process_nozzle_diameters");
        REQUIRE(roles != nullptr);
        CHECK(roles->values.empty());
        check_body_split_print(slice_reopened(loaded, loaded_config), 0, 1, 0.10, 0.30);

        // The loaded tower policy still owns the escaped process name: Turn off restores the user's
        // 60 mm tower only when that ownership survived both JSON layers.
        const auto *loaded_policy = loaded_config.option<ConfigOptionString>(MIXED_NOZZLE_TOWER_POLICY_OPTION);
        REQUIRE(loaded_policy != nullptr);
        project.bundle.project_config.set_key_value(MIXED_NOZZLE_TOWER_POLICY_OPTION,
            new ConfigOptionString(loaded_policy->value));
        project.bundle.prints.get_edited_preset().config.set_key_value("prime_tower_width",
            new ConfigOptionFloat(loaded_config.opt_float("prime_tower_width")));
        REQUIRE(project.bundle.prints.get_edited_preset().config.opt_float("prime_tower_width") < 60.);
        // Opened with every plate Off, the project releases the ledger the same way, as loading does.
        DynamicPrintConfig all_off = loaded_config;
        CHECK(release_stale_mixed_nozzle_tower(all_off, all_off, false, false));
        CHECK_THAT(all_off.opt_float("prime_tower_width"), Catch::Matchers::WithinAbs(60., .01));
        CHECK_FALSE(release_stale_mixed_nozzle_tower(all_off, all_off, false, false));
        turn_off_current_plate(project);
        CHECK_THAT(project.bundle.prints.get_edited_preset().config.opt_float("prime_tower_width"),
                   Catch::Matchers::WithinAbs(60., .01));
    }

    // A copied object in a Cadence project reopens as one object with two instances; a foreign
    // archive would be split into separate objects.
    ModelObject *object = project.model.objects.front();
    ModelInstance *copy = object->add_instance(*object->instances.front());
    copy->set_offset(object->instances.front()->get_offset() + Vec3d(40., 0., 0.));
    ScopedTemporaryFile copies(".3mf"), branded_copies(".3mf");
    save_project_3mf(project.model, original.config, plate, copies.string());
    REQUIRE(mark_native_archive_as_cadence(copies.string(), branded_copies.string()));
    const ReopenedProject reopened_copies(branded_copies.string());
    REQUIRE(reopened_copies.model.objects.size() == 1);
    CHECK(reopened_copies.model.objects.front()->instances.size() == 2);
}

TEST_CASE("Remembered renamed presets keep the chosen nozzle materials and their native sibling flow",
          "[TestRebuild][Wizard][Persistence][Materials]")
{
    const auto slice_with_sibling_flow = [](double coarse_flow) {
        const ProfileSpec spec = h2d_02_06_spec();
        WizardProject project(spec);
        Preset *process = project.bundle.prints.find_preset(spec.process, false, true);
        Preset *fine_material = project.bundle.filaments.find_preset(spec.materials.front(), false, true);
        Preset *coarse_material = project.bundle.filaments.find_preset(
            "Bambu PLA Basic @BBL H2D 0.6 nozzle", false, true);
        REQUIRE(process != nullptr);
        REQUIRE(fine_material != nullptr);
        REQUIRE(coarse_material != nullptr);
        process->renamed_from.push_back("Remembered fine process");
        fine_material->renamed_from.push_back("Remembered fine PLA");

        // A native import can rename the 0.6 mm sibling's inheritance; its own flow shows in the coarse roads.
        coarse_material->config.set_key_value(BBL_JSON_KEY_INHERITS,
            new ConfigOptionString("Renamed Bambu PLA root"));
        coarse_material->config.set_key_value("filament_flow_ratio",
            new ConfigOptionFloatsNullable{coarse_flow, coarse_flow});
        AppConfig remembered;
        const DynamicPrintConfig &machine = project.bundle.printers.get_edited_preset().config;
        remembered.set_variant(spec.vendor, machine.opt_string("printer_model"),
                               machine.opt_string("printer_variant"), true);
        remembered.set_printer_setting(spec.printer, PRESET_PRINT_NAME, "Remembered fine process");
        remembered.set_printer_setting(spec.printer, PRESET_FILAMENT_NAME, "Remembered fine PLA");
        remembered.set_printer_setting(spec.printer, "filament_01", "Remembered fine PLA");
        project.bundle.load_installed_printers(remembered);
        project.bundle.update_selections(remembered);
        REQUIRE(project.bundle.prints.get_selected_preset_name() == spec.process);
        REQUIRE(project.bundle.filaments.get_selected_preset_name() == spec.materials.front());
        REQUIRE(project.bundle.filament_presets.size() >= 2);
        CHECK(project.bundle.filament_presets[1] == spec.materials.front());

        add_two_bodies(project);
        // Keep the remembered process itself, not an installed Body preset.
        project.bundle.prints.get_edited_preset().config.set_key_value(
            "initial_layer_print_height", new ConfigOptionFloat(.10));
        SetupChoice choice = body_choice(.10, 3);
        choice.keep_current_process = true;
        const WizardApplyResult applied = run_wizard_setup(project, choice);
        REQUIRE(applied.applied);
        const PlateSlice slice = require_slice(project);
        check_body_split_print(slice, 0, 1, .10, .30);
        double coarse_mm3_per_mm = 0.;
        size_t coarse_roads = 0;
        for (const auto &move : slice.moves)
            if (move.type == EMoveType::Extrude && move.extrusion_role == erExternalPerimeter &&
                move.physical_tool_id == 1 && move.position.z() > .31f) {
                coarse_mm3_per_mm += move.mm3_per_mm;
                ++coarse_roads;
            }
        REQUIRE(coarse_roads > 0);
        return std::pair<double, size_t>{coarse_mm3_per_mm / double(coarse_roads), coarse_roads};
    };

    const auto normal = slice_with_sibling_flow(.98);
    const auto reduced = slice_with_sibling_flow(.64);
    CAPTURE(normal.first, reduced.first, normal.second, reduced.second);
    CHECK(normal.second == reduced.second);
    CHECK(reduced.first < normal.first * .8);
}

TEST_CASE("Keeping a newly installed nozzle pair commits its arrays even when the setup mode stays Off",
          "[TestRebuild][Wizard][Transaction]")
{
    WizardProject project(h2d_02_06_spec());
    add_cube(project, Vec2d(100., 100.), Vec3d(12., 12., 4.));
    project.bundle.project_config.set_key_value("nozzle_diameter", new ConfigOptionFloats{0.2, 0.4});
    project.bundle.project_config.set_key_value("min_layer_height", new ConfigOptionFloats{0.04, 0.08});
    project.bundle.project_config.set_key_value("max_layer_height", new ConfigOptionFloats{0.14, 0.28});
    const MixedNozzleSetupRequest request = mixed_nozzle_keep_pair_request(
        project.bundle.project_config, {0.2, 0.6}, {0.04, 0.12}, {0.14, 0.42});
    int snapshots = 0;
    int notifications = 0;
    MixedNozzleSetupHooks hooks;
    hooks.take_snapshot = [&] { ++snapshots; };
    hooks.invalidate = [](PartPlate &) {};
    hooks.notify_batch_changed = [&] { ++notifications; };
    const MixedNozzleSetupPlan plan = stage_mixed_nozzle_setup(
        project.bundle, project.owners.plates, request, &project.plate);
    INFO(plan.diagnostic);
    REQUIRE(plan.diagnostic.empty());
    REQUIRE(commit_mixed_nozzle_setup_transaction(
        project.bundle, project.owners.plates, plan, request, hooks, &project.plate));
    CHECK(snapshots == 1);
    CHECK(notifications == 1);
    const PlateSlice printed = require_slice(project);
    CHECK_FALSE(printed.gcode.empty());
    REQUIRE(printed.config.option<ConfigOptionFloats>("nozzle_diameter") != nullptr);
    CHECK(printed.config.option<ConfigOptionFloats>("nozzle_diameter")->values ==
          std::vector<double>{0.2, 0.6});
    CHECK(printed.config.option<ConfigOptionFloats>("min_layer_height")->values ==
          std::vector<double>{0.04, 0.12});
    CHECK(printed.config.option<ConfigOptionFloats>("max_layer_height")->values ==
          std::vector<double>{0.14, 0.42});
    CHECK_FALSE(commit_mixed_nozzle_setup_transaction(
        project.bundle, project.owners.plates, request, hooks, &project.plate));
    CHECK(snapshots == 1);
    CHECK(notifications == 1);
}

namespace {

// A fresh H2D project: 0.2 mm Standard left, 0.4 mm High Flow right, six slots alternating left and
// right as the sidebar grid shows them. Slot 4 is PETG Basic on the right, slot 6 PETG HF.
ProfileSpec h2d_02_04_high_flow_spec()
{
    ProfileSpec spec = h2d_02_06_spec();
    const std::string pla = "Bambu PLA Basic @BBL H2D 0.2 nozzle";
    spec.materials = {pla, pla, pla, "Bambu PETG Basic @BBL H2D 0.2 nozzle", pla, "Bambu PETG HF @BBL H2D 0.2 nozzle"};
    spec.also_visible_materials = {"Bambu PLA Basic @BBL H2D", "Bambu PETG Basic @BBL H2D 0.4 nozzle",
                                   "Bambu PETG HF @BBL H2D 0.4 nozzle"};
    spec.nozzles = {0.2, 0.4};
    spec.min_heights = {0.04, 0.08};
    spec.max_heights = {0.14, 0.28};
    return spec;
}

const std::vector<int> kHighFlowRight{int(NozzleVolumeType::nvtStandard), int(NozzleVolumeType::nvtHighFlow)};

void prepare_fresh_high_flow_project(WizardProject &project)
{
    DynamicPrintConfig &config = project.bundle.project_config;
    const std::vector<int> map{1, 2, 1, 2, 1, 2};
    config.set_key_value("filament_map", new ConfigOptionInts(map));
    project.plate.config()->set_key_value("filament_map", new ConfigOptionInts(map));
    config.set_key_value("nozzle_volume_type", new ConfigOptionEnumsGeneric(kHighFlowRight));
    // PETG HF is not rated for the default Cool Plate.
    config.set_key_value("curr_bed_type", new ConfigOptionEnum<BedType>(btPTE));
}

// The slot the object list shows for a part: its own, else the object's.
int object_list_slot(const ModelObject &object, const ModelVolume &volume)
{
    if (volume.config.has("extruder"))
        return volume.config.extruder();
    return object.config.has("extruder") ? object.config.extruder() : 1;
}

} // namespace

// A part imported on slot 1 into a new H2D project: step 1 puts the small parts on Fine and the body
// on Coarse, step 2 picks slot 4 (PETG Basic, moved to the left) for fine and slot 6 (PETG HF) for
// coarse, and More options gives the third part the exact slot 3.
TEST_CASE("Wizard Apply gives each part its picked or exact material and keeps High Flow",
          "[TestRebuild][WizardApply][BodySplit]")
{
    WizardProject project(h2d_02_04_high_flow_spec());
    prepare_fresh_high_flow_project(project);
    ModelObject *object = project.add_object("two-part", {{"Body11", Vec3d(40., 30., 8.), Vec3d::Zero(), 1},
                                                        {"Body12", Vec3d(10., 30., 8.), Vec3d(45., 0., 0.), 1},
                                                        {"Body13", Vec3d(10., 30., 8.), Vec3d(-15., 0., 0.), 1}},
                                             Vec2d(100., 100.));
    object->config.set_key_value("extruder", new ConfigOptionInt(1));

    SetupChoice choice = body_choice(0.08, 2);
    choice.fine_slot = 3;
    choice.coarse_slot = 5;
    choice.body_roles = {WizardBodyRole::Coarse, WizardBodyRole::Fine, WizardBodyRole::Fine};
    // Slot 3 sits on the left, the fine nozzle.
    choice.body_slots = {std::nullopt, std::nullopt, std::size_t(2)};
    run_wizard_setup(project, choice);

    CHECK(object_list_slot(*object, *object->volumes[0]) == 6);
    CHECK(object_list_slot(*object, *object->volumes[1]) == 4);
    CHECK(object_list_slot(*object, *object->volumes[2]) == 3);
    const auto *flows = project.bundle.project_config.option<ConfigOptionEnumsGeneric>("nozzle_volume_type");
    REQUIRE(flows != nullptr);
    CHECK(flows->values == kHighFlowRight);
    const PlateSlice slice = require_slice(project);
    CHECK(slice.config.option<ConfigOptionEnumsGeneric>("nozzle_volume_type")->values == kHighFlowRight);
}

TEST_CASE("Setup reopens on the user's own tower settings, also after the project is saved and opened again",
          "[TestRebuild][Wizard][Tower][TowerIntent]")
{
    WizardProject project(h2d_02_06_spec());
    add_cube(project);
    SetupChoice choice = feature_choice(0.10, 4);
    choice.tower = WizardTowerIntent::Preserve;
    run_wizard_setup(project, choice);
    CHECK(tower_intent(project.bundle, *project.plate.config()) == WizardTowerIntent::Preserve);
    project.bundle.prints.get_edited_preset().config.set_key_value("prime_tower_width", new ConfigOptionFloat(47.));

    ScopedTemporaryDir backup("cadence_tower_intent_3mf");
    project.model.set_backup_path(backup.string());
    ScopedTemporaryFile archive(".3mf");
    DynamicPrintConfig saved = project.bundle.full_config(false);
    PlateData plate;
    plate.plate_index = 0;
    plate.config = *project.plate.config();
    save_project_3mf(project.model, saved, plate, archive.string());
    const ReopenedProject loaded(archive.string());
    PresetBundle reopened = installed_bbl_profiles();
    reopened.load_config_model(archive.string(), loaded.config);
    CHECK(tower_intent(reopened, loaded.plate_config) == WizardTowerIntent::Preserve);
    DynamicPrintConfig effective = reopened.full_config();
    effective.apply(loaded.plate_config);
    CHECK_THAT(effective.opt_float("prime_tower_width"), Catch::Matchers::WithinAbs(47., 1e-6));
    // A tower this wide can price the bands all fine, so only the mode and the tower width are checked.
    CHECK(effective.opt_enum<MixedNozzleSlicingMode>("mixed_nozzle_slicing_mode") == MixedNozzleSlicingMode::FeatureSplit);
    const PlateSlice resliced = slice_reopened(loaded, effective);
    CHECK(resliced.gcode.find("; prime_tower_width = 47\n") != std::string::npos);
}
