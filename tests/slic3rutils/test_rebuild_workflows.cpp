
#include <catch2/catch_all.hpp>

#include "wizard_apply_support.hpp"

#include "fff_print/rebuild_harness.hpp"
#include "slic3r/GUI/MixedNozzleAssignmentBindingTransaction.hpp"
#include "slic3r/GUI/FeatureSplitEditorNative.hpp"
#include "slic3r/GUI/MixedNozzleSetupTransaction.hpp"
#include "slic3r/GUI/MixedNozzleTowerPolicy.hpp"
#include "slic3r/GUI/MixedNozzleSidebarController.hpp"
#include "slic3r/GUI/MixedNozzleDecisionReport.hpp"
#include "slic3r/GUI/ProjectNozzleFlowState.hpp"
#include "libslic3r/Format/bbs_3mf.hpp"
#include "libslic3r/MixedNozzleBinding.hpp"
#include "libslic3r/miniz_extension.hpp"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <limits>
#include <map>
#include <set>
#include <sstream>
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

std::string describe(const PlateSlice &slice, const EmittedFacts &facts)
{
    std::ostringstream out;
    for (const char *key : {"mixed_nozzle_slicing_mode", "filament_map", "filament_map_mode", "nozzle_diameter",
                            "outer_wall_filament_id", "sparse_infill_filament_id", "layer_height",
                            "mixed_nozzle_coarse_layer_height", "enable_prime_tower", "print_settings_id",
                            "filament_settings_id", "filament_max_volumetric_speed", "sparse_infill_density",
                            "initial_layer_print_height"})
        out << key << "=" << (slice.config.has(key) ? slice.config.opt_serialize(key) : "absent") << "; ";
    out << "tool changes=" << facts.tool_changes.size() << "; economics: " << slice.economics << "; ";
    for (const char *key : {"; sparse_infill_filament_id =", "; outer_wall_filament_id =", "; filament_diameter =",
                            "; mixed_nozzle_slicing_mode =", "; wall_filament =", "; sparse_infill_filament ="}) {
        const std::size_t at = slice.gcode.find(key);
        out << "gcode" << key << (at == std::string::npos ? std::string(" absent")
                                  : slice.gcode.substr(at + std::strlen(key), slice.gcode.find('\n', at) - at - std::strlen(key)))
            << "; ";
    }
    for (const auto &[reason, count] : slice.band_reasons)
        out << "bands " << reason << " x" << count << "; ";
    {
        std::istringstream lines(slice.gcode);
        std::string line;
        int shown = 0;
        while (shown < 6 && std::getline(lines, line))
            if (line.rfind("; SRL_", 0) == 0) {
                out << line << "; ";
                ++shown;
            }
    }
    for (const auto &[sparse, outer] : slice.region_filaments)
        out << "region sparse " << sparse << " outer " << outer << "; ";
    std::map<int, std::set<int>> logical_by_role;
    for (const auto &fact : facts.role_facts)
        logical_by_role[int(fact.role)].insert(int(fact.logical_filament));
    for (const auto &[role, logicals] : logical_by_role) {
        out << "role " << role << " logical";
        for (int logical : logicals)
            out << " " << logical;
        out << "; ";
    }
    for (const auto &[tool, print] : object_print_by_tool(facts)) {
        out << "tool " << int(tool) << " walls " << print.max_wall_height << " infill " << print.max_infill_height
            << " roles";
        for (ExtrusionRole role : print.roles)
            out << " " << int(role);
        out << "; ";
    }
    return out.str();
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

PlateSlice require_slice(WizardProject &project)
{
    INFO("test: " << Catch::getResultCapture().getCurrentTestName());
    PlateSlice slice = slice_plate(project);
    INFO("Slice refused: " << slice.refusal);
    REQUIRE(slice.refusal.empty());
    REQUIRE_FALSE(slice.gcode.empty());
    return slice;
}

DynamicPrintConfig workflow_print_config(WizardProject &project)
{
    DynamicPrintConfig config = plate_slice_config(project);
    srl_fixtures::fill_per_filament_values(config);
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

void check_feature_split_print(const PlateSlice &slice, std::size_t fine_slot, std::size_t coarse_slot,
                               double fine_height, double coarse_height)
{
    const unsigned char fine = physical_of(slice, fine_slot);
    const unsigned char coarse = physical_of(slice, coarse_slot);
    REQUIRE(fine != coarse);
    const EmittedFacts facts = extract_emitted_facts(slice.gcode);
    INFO("test: " << Catch::getResultCapture().getCurrentTestName());
    INFO(describe(slice, facts));
    const auto tools = object_print_by_tool(facts);
    REQUIRE(tools.count(fine) == 1);
    REQUIRE(tools.count(coarse) == 1);
    CHECK(tools.size() == 2);
    CHECK(tools.at(fine).roles.count(erExternalPerimeter) == 1);
    CHECK(tools.at(coarse).roles.count(erInternalInfill) == 1);
    CHECK_THAT(tools.at(fine).max_wall_height, Catch::Matchers::WithinAbs(fine_height, kHeightTolerance));
    CHECK_THAT(tools.at(coarse).max_infill_height, Catch::Matchers::WithinAbs(coarse_height, kHeightTolerance));
    CHECK_FALSE(facts.tool_changes.empty());
}

void check_body_split_print(const PlateSlice &slice, std::size_t fine_slot, std::size_t coarse_slot,
                            double fine_height, double coarse_height, bool fine_on_left = true)
{
    const unsigned char fine = physical_of(slice, fine_slot);
    const unsigned char coarse = physical_of(slice, coarse_slot);
    REQUIRE(fine != coarse);
    const EmittedFacts facts = extract_emitted_facts(slice.gcode);
    INFO("test: " << Catch::getResultCapture().getCurrentTestName());
    INFO(describe(slice, facts));
    const auto tools = object_print_by_tool(facts);
    REQUIRE(tools.count(fine) == 1);
    REQUIRE(tools.count(coarse) == 1);
    CHECK(tools.size() == 2);
    CHECK_THAT(tools.at(fine).max_wall_height, Catch::Matchers::WithinAbs(fine_height, kHeightTolerance));
    CHECK_THAT(tools.at(coarse).max_wall_height, Catch::Matchers::WithinAbs(coarse_height, kHeightTolerance));
    CHECK_FALSE(facts.tool_changes.empty());

    float fine_right = std::numeric_limits<float>::lowest();
    float coarse_left = std::numeric_limits<float>::max();
    for (const auto &move : slice.moves) {
        if (move.type != EMoveType::Extrude || move.extrusion_role != erExternalPerimeter)
            continue;
        if (move.physical_tool_id == fine)
            fine_right = std::max(fine_right, move.position.x());
        else if (move.physical_tool_id == coarse)
            coarse_left = std::min(coarse_left, move.position.x());
    }
    float fine_left = std::numeric_limits<float>::max();
    float coarse_right = std::numeric_limits<float>::lowest();
    for (const auto &move : slice.moves) {
        if (move.type != EMoveType::Extrude || move.extrusion_role != erExternalPerimeter)
            continue;
        if (move.physical_tool_id == fine)
            fine_left = std::min(fine_left, move.position.x());
        else if (move.physical_tool_id == coarse)
            coarse_right = std::max(coarse_right, move.position.x());
    }
    if (fine_on_left)
        CHECK(fine_right < coarse_left);
    else
        CHECK(coarse_right < fine_left);
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

MixedNozzleAssignmentBindingHooks editor_hooks(int &snapshots)
{
    MixedNozzleAssignmentBindingHooks hooks;
    hooks.take_snapshot = [&snapshots] { ++snapshots; };
    hooks.take_snapshot_with_setup_transition = [&snapshots](const MixedNozzleSetupPlan &) {
        ++snapshots;
        return true;
    };
    hooks.record_cadence_transition = [](std::optional<double>, std::optional<double>) {};
    hooks.notify_batch_changed = [] {};
    return hooks;
}

void commit_feature_editor(WizardProject &project, std::size_t fine_slot, std::size_t coarse_slot,
                           double coarse_height)
{
    PresetBundle &bundle = project.bundle;
    PartPlate &plate = project.plate;
    const auto maps = plate.get_real_filament_maps(bundle.project_config);
    const auto volumes = plate.get_real_filament_volume_maps(bundle.project_config);
    const DynamicPrintConfig full = bundle.full_config(true, maps, volumes);
    PrintConfig resolver = feature_split_resolver_config_from_full(full);
    resolver.filament_map_mode.value = plate.get_real_filament_map_mode(bundle.project_config);
    resolver.filament_map.values = maps;
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
    FeatureSplitEditorDiagnostic staged_diagnostic = FeatureSplitEditorDiagnostic::None;
    request.stage_assignment = [feature_request, mode, nozzle_count, &staged_diagnostic](
                                   MixedNozzleAssignmentBindingStageState &state) {
        const auto staged_maps = state.plate.get_real_filament_maps(state.bundle.project_config);
        const auto staged_volumes = state.plate.get_real_filament_volume_maps(state.bundle.project_config);
        const DynamicPrintConfig staged_full = state.bundle.full_config(true, staged_maps, staged_volumes);
        PrintConfig staged_resolver = feature_split_resolver_config_from_full(staged_full);
        staged_resolver.filament_map_mode.value = state.plate.get_real_filament_map_mode(state.bundle.project_config);
        staged_resolver.filament_map.values = staged_maps;
        const double base = staged_full.has("layer_height") ? staged_full.opt_float("layer_height") : 0.;
        const FeatureSplitApplyResult staged = commit_feature_split_editor_request(
            state.bundle.project_config, state.bundle.prints.get_edited_preset().config, state.plate, mode,
            state.plate.get_index(), feature_request.target.plate_id, nozzle_count, staged_resolver,
            feature_split_coarse_cadence_choices(base, staged_resolver), feature_request);
        staged_diagnostic = staged.diagnostic;
        return MixedNozzleAssignmentStageResult{staged.applied, staged.changed};
    };
    auto plan = stage_mixed_nozzle_assignment_binding(bundle, project.model, plate, request);
    int snapshots = 0;
    MixedNozzleAssignmentBindingDiagnostic diagnostic = MixedNozzleAssignmentBindingDiagnostic::None;
    INFO("stage diagnostic " << int(plan.diagnostic) << ", editor diagnostic " << int(staged_diagnostic));
    const bool committed = commit_mixed_nozzle_assignment_binding(bundle, project.model, project.owners.plates, plan,
                                                                  editor_hooks(snapshots), &diagnostic);
    INFO("commit diagnostic " << int(diagnostic));
    REQUIRE(committed);
    CHECK(snapshots == 1);
}

struct BodyEdit {
    std::size_t slot;
    double height;
};

bool commit_body_editor(WizardProject &project, const std::map<std::string, BodyEdit> &edits)
{
    PresetBundle &bundle = project.bundle;
    PartPlate &plate = project.plate;
    const auto maps = plate.get_real_filament_maps(bundle.project_config);
    const auto volumes = plate.get_real_filament_volume_maps(bundle.project_config);
    const DynamicPrintConfig full = bundle.full_config(true, maps, volumes);
    PrintConfig resolver = mixed_nozzle_resolver_config_from_full(full);
    resolver.filament_map_mode.value = plate.get_real_filament_map_mode(bundle.project_config);
    resolver.filament_map.values = maps;
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
    BodySplitEditorDiagnostic staged_diagnostic = BodySplitEditorDiagnostic::None;
    request.stage_assignment = [body_request, &staged_diagnostic](MixedNozzleAssignmentBindingStageState &state) {
        const auto staged_maps = state.plate.get_real_filament_maps(state.bundle.project_config);
        const auto staged_volumes = state.plate.get_real_filament_volume_maps(state.bundle.project_config);
        PrintConfig staged_resolver = mixed_nozzle_resolver_config_from_full(
            state.bundle.full_config(true, staged_maps, staged_volumes));
        staged_resolver.filament_map_mode.value = state.plate.get_real_filament_map_mode(state.bundle.project_config);
        staged_resolver.filament_map.values = staged_maps;
        const BodySplitApplyResult staged = apply_body_split_editor_request(
            state.model, *state.plate.config(), staged_resolver, body_request);
        staged_diagnostic = staged.diagnostic;
        return MixedNozzleAssignmentStageResult{staged.applied, staged.changed};
    };
    auto plan = stage_mixed_nozzle_assignment_binding(bundle, project.model, plate, request);
    int snapshots = 0;
    MixedNozzleAssignmentBindingDiagnostic diagnostic = MixedNozzleAssignmentBindingDiagnostic::None;
    INFO("stage diagnostic " << int(plan.diagnostic) << ", editor diagnostic " << int(staged_diagnostic));
    const bool committed = commit_mixed_nozzle_assignment_binding(bundle, project.model, project.owners.plates, plan,
                                                                  editor_hooks(snapshots), &diagnostic);
    INFO("commit diagnostic " << int(diagnostic));
    if (committed)
        CHECK(snapshots == 1);
    return committed;
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
          "[TestRebuild][E2E][FeatureSplit]")
{
    WizardProject project(h2d_02_06_spec());
    add_cube(project);
    run_wizard_setup(project, feature_choice(0.10, 4));
    const PlateSlice slice = require_slice(project);
    check_feature_split_print(slice, 0, 1, 0.10, 0.40);
}

TEST_CASE("The wizard's Automatic tower prints at most 35 mm wide with the engine's own brim",
          "[TestRebuild][E2E][FeatureSplit][Tower]")
{
    WizardProject project(h2d_02_06_spec());
    add_cube(project);
    run_wizard_setup(project, feature_choice(0.10, 4));
    const PlateSlice slice = require_slice(project);
    REQUIRE(slice.config.opt_bool("enable_prime_tower"));
    CHECK(slice.config.opt_float("wipe_tower_rotation_angle") == 0.);
    CHECK(slice.config.opt_float("prime_tower_brim_width") < 0.);
    const TowerFootprint tower = tower_footprint(slice.moves);
    REQUIRE(tower.moves > 0);
    CHECK(tower.upper_width <= 35.f + 0.5f);
    CHECK(tower.first_layer_width >= tower.upper_width);
    CHECK_THAT(slice.tower_brim_width, Catch::Matchers::WithinAbs(3., 0.25));
}

TEST_CASE("Body Split set up by the wizard slices each body on its own nozzle at its own layer",
          "[TestRebuild][E2E][BodySplit]")
{
    WizardProject project(h2d_02_06_spec());
    add_two_bodies(project);
    run_wizard_setup(project, body_choice(0.10, 3));
    const PlateSlice slice = require_slice(project);
    check_body_split_print(slice, 0, 1, 0.10, 0.30);
}

TEST_CASE("The sidebar's Sliced line counts a Body Split slice's nozzle changes",
          "[TestRebuild][B33][BodySplit]")
{
    WizardProject project(h2d_02_06_spec());
    add_two_bodies(project);
    run_wizard_setup(project, body_choice(0.10, 3));
    const PlateSlice slice = require_slice(project);
    // Both bodies print on every layer, each on its own nozzle, so the slice swaps nozzles.
    std::set<int> tools;
    for (const auto &move : slice.moves)
        if (move.type == EMoveType::Extrude && move.physical_tool_id != GCodeProcessorResult::UNKNOWN_PHYSICAL_TOOL_ID)
            tools.insert(move.physical_tool_id);
    REQUIRE(tools == std::set<int>{0, 1});

    MixedNozzleSidebarFacts facts;
    mixed_nozzle_sidebar_read_slice_statistics(slice.statistics, facts);
    REQUIRE(facts.nozzle_changes);
    CHECK(*facts.nozzle_changes > 0);
    CHECK(facts.print_seconds.value_or(0.) > 0.);
    facts.tower_readout = "Prime tower: 20.0 x 20.0 mm, 1.0 cm3, about 5 min";
    MixedNozzleSidebarSignature signature;
    signature.target_valid = true;
    signature.effective = MixedNozzleSlicingMode::BodySplit;
    signature.slice_revision = 1;
    const auto texts = mixed_nozzle_sidebar_lines(signature, project.bundle.project_config, {}, facts);
    CHECK(texts.sliced_line.find(std::to_string(*facts.nozzle_changes) + " nozzle change") != std::string::npos);
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

TEST_CASE("A Plate Settings coarse layer change after Feature Split setup slices at the new layer",
          "[TestRebuild][E2E][FeatureSplit][Editor]")
{
    WizardProject project(h2d_02_06_spec());
    add_cube(project);
    run_wizard_setup(project, feature_choice(0.10, 3));
    commit_feature_editor(project, 0, 1, 0.40);
    const PlateSlice slice = require_slice(project);
    check_feature_split_print(slice, 0, 1, 0.10, 0.40);
}

TEST_CASE("A Plate Settings body swap after Body Split setup slices each body on its new nozzle",
          "[TestRebuild][E2E][BodySplit][Editor]")
{
    WizardProject project(h2d_02_06_spec());
    add_two_bodies(project);
    run_wizard_setup(project, body_choice(0.10, 3));
    REQUIRE(commit_body_editor(project, {{"left body", {1, 0.30}}, {"right body", {0, 0.10}}}));
    const PlateSlice slice = require_slice(project);
    check_body_split_print(slice, 0, 1, 0.10, 0.30, false);
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
    int reslices = 0;
    const auto reslice = [&] {
        ++reslices;
        const DynamicPrintConfig config = workflow_print_config(project);
        print.apply(project.model, config);
        const CadenceTest::Facts facts = CadenceTest::slice(print);
        std::ostringstream bodies;
        for (const ModelVolume *volume : project.model.objects.front()->volumes)
            bodies << volume->name << '=' << (volume->config.has("extruder") ? volume->config.opt_int("extruder") : 0) << ' ';
        INFO("slice " << reslices << " map=" << config.opt_serialize("filament_map") << " mode=" << config.opt_serialize("filament_map_mode")
             << " nozzles=" << config.opt_serialize("nozzle_diameter") << " bodies=" << bodies.str()
             << " object extruder=" << (project.model.objects.front()->config.has("extruder")
                                         ? project.model.objects.front()->config.opt_int("extruder") : 0));
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

    // Move only the left body's material from the 0.2 mm head to the first 0.4 mm head. Both can
    // lay 0.12 mm, so its cadence and geometry stay put, and the coarse body stays on the widest
    // head as the engine requires. The second export must rebuild native ownership on this same
    // Print.
    REQUIRE(commit_body_editor(project, { {"left body", {0, .12}},
                                          {"right body", {3, .24}} }));
    const CadenceTest::Facts after = reslice();
    CHECK(wall_tools(after, 99., 111.) == std::set<unsigned>{0});
    CHECK(wall_tools(after, 114., 126.) == std::set<unsigned>{3});
    CHECK_FALSE(after.emitted.tool_changes.empty());
    CHECK(after.gcode != before.gcode);
}

TEST_CASE("Turning Feature Split off clears the decision report on the reused Print",
          "[TestRebuild][E2E][FeatureSplit][Advisor][Reuse]")
{
    WizardProject project(h2d_02_06_spec());
    add_cube(project);
    run_wizard_setup(project, feature_choice(0.10, 4));
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

    const MixedNozzleSetupRequest off = mixed_nozzle_off_request(false);
    const MixedNozzleSetupPlan plan = stage_mixed_nozzle_setup(
        project.bundle, project.owners.plates, off, &project.plate);
    INFO(plan.diagnostic);
    REQUIRE(plan.diagnostic.empty());
    MixedNozzleSetupHooks hooks;
    hooks.take_snapshot = [] {};
    hooks.invalidate = [](PartPlate &) {};
    hooks.notify_batch_changed = [] {};
    REQUIRE(commit_mixed_nozzle_setup_transaction(
        project.bundle, project.owners.plates, plan, off, hooks, &project.plate));
    // The sidebar asks for this report as soon as Apply invalidates the old
    // print, before the replacement Off slice has processed any layers.
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

TEST_CASE("Snapmaker J1 after Feature Split setup slices", "[TestRebuild][E2E][OtherPrinters][J1]")
{
    WizardProject project(j1_spec());
    add_cube(project, Vec2d(30., 30.), Vec3d(80., 80., 30.));
    run_wizard_setup(project, feature_choice(0.10, 3));
    const PlateSlice slice = require_slice(project);
    check_feature_split_print(slice, 0, 1, 0.10, 0.30);
}

TEST_CASE("Snapmaker J1 with its Arachne process after Body Split setup slices",
          "[TestRebuild][E2E][OtherPrinters][J1][Arachne]")
{
    WizardProject project(j1_spec());
    REQUIRE(project.bundle.full_config().opt_enum<PerimeterGeneratorType>("wall_generator") ==
            PerimeterGeneratorType::Arachne);
    add_two_bodies(project, Vec2d(80., 80.));
    run_wizard_setup(project, body_choice(0.10, 2));
    const PlateSlice slice = require_slice(project);
    check_body_split_print(slice, 0, 1, 0.10, 0.20);
}

TEST_CASE("RatRig V-Core 4 IDEX after Feature Split setup slices", "[TestRebuild][E2E][OtherPrinters][RatRig]")
{
    WizardProject project(ratrig_idex_spec());
    add_cube(project);
    run_wizard_setup(project, feature_choice(0.08, 5));
    const PlateSlice slice = require_slice(project);
    check_feature_split_print(slice, 0, 1, 0.08, 0.40);
}

TEST_CASE("RatRig V-Core 4 IDEX after Body Split setup slices", "[TestRebuild][E2E][OtherPrinters][RatRig]")
{
    WizardProject project(ratrig_idex_spec());
    add_two_bodies(project, Vec2d(80., 100.));
    run_wizard_setup(project, body_choice(0.12, 2));
    const PlateSlice slice = require_slice(project);
    check_body_split_print(slice, 0, 1, 0.12, 0.24);
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

TEST_CASE("H2D with Arachne walls after Body Split setup slices", "[TestRebuild][E2E][BodySplit][Arachne]")
{
    WizardProject project(h2d_02_06_spec());
    project.bundle.prints.get_edited_preset().config.set_key_value("wall_generator",
        new ConfigOptionEnum<PerimeterGeneratorType>(PerimeterGeneratorType::Arachne));
    add_two_bodies(project);
    SetupChoice choice = body_choice(0.10, 3);
    choice.keep_current_process = true;
    run_wizard_setup(project, choice);
    const PlateSlice slice = require_slice(project);
    CHECK(slice.config.opt_enum<PerimeterGeneratorType>("wall_generator") == PerimeterGeneratorType::Classic);
    check_body_split_print(slice, 0, 1, 0.10, 0.30);
}

TEST_CASE("H2D elephant foot compensation on no layers survives Body Split setup and slices",
          "[TestRebuild][E2E][BodySplit]")
{
    WizardProject project(h2d_02_06_spec());
    DynamicPrintConfig &process = project.bundle.prints.get_edited_preset().config;
    process.set_key_value("elefant_foot_compensation", new ConfigOptionFloat(0.1));
    process.set_key_value("elefant_foot_compensation_layers", new ConfigOptionInt(0));
    add_two_bodies(project);
    SetupChoice choice = body_choice(0.10, 3);
    choice.keep_current_process = true;
    run_wizard_setup(project, choice);
    const PlateSlice slice = require_slice(project);
    CHECK_THAT(slice.config.opt_float("elefant_foot_compensation"), Catch::Matchers::WithinAbs(0.1, 0.001));
    CHECK(slice.config.opt_int("elefant_foot_compensation_layers") == 0);
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

TEST_CASE("A Plate Settings coarse layer change after Body Split setup keeps the beams sliceable",
          "[TestRebuild][E2E][BodySplit][Editor]")
{
    WizardProject project(h2d_02_06_spec());
    add_two_bodies(project);
    run_wizard_setup(project, body_choice(0.10, 3));
    REQUIRE(commit_body_editor(project, {{"left body", {0, 0.10}}, {"right body", {1, 0.20}}}));
    const PlateSlice slice = require_slice(project);
    check_body_split_print(slice, 0, 1, 0.10, 0.20);
}

TEST_CASE("Feature Split set up by the wizard on a part without its own slot",
          "[TestRebuild][E2E][FeatureSplit]")
{
    WizardProject project(h2d_02_06_spec());
    project.add_object("box", {{"box", Vec3d(180., 180., 12.), Vec3d::Zero(), 0}}, Vec2d(20., 20.));
    run_wizard_setup(project, feature_choice(0.10, 4));
    const PlateSlice slice = require_slice(project);
    check_feature_split_print(slice, 0, 1, 0.10, 0.40);
}

namespace {

DynamicPrintConfig rebuild_ledger_config()
{
    DynamicPrintConfig config = DynamicPrintConfig::full_print_config();
    config.set_key_value("filament_diameter", new ConfigOptionFloats{1.75, 1.75});
    config.set_key_value("filament_map_mode", new ConfigOptionEnum<FilamentMapMode>(fmmManual));
    config.set_key_value("filament_map", new ConfigOptionInts{1, 2});
    config.set_key_value("nozzle_diameter", new ConfigOptionFloats{0.2, 0.6});
    config.set_key_value("mixed_nozzle_process_nozzle_diameters", new ConfigOptionFloats{0.2, 0.6});
    config.set_key_value("print_extruder_id", new ConfigOptionInts{1, 2});
    config.set_key_value("print_extruder_variant",
        new ConfigOptionStrings{"Direct Drive Standard", "Direct Drive High Flow"});
    config.set_key_value("extruder_type",
        new ConfigOptionEnumsGeneric{int(ExtruderType::etDirectDrive), int(ExtruderType::etDirectDrive)});
    config.set_key_value("nozzle_volume_type",
        new ConfigOptionEnumsGeneric{int(NozzleVolumeType::nvtStandard), int(NozzleVolumeType::nvtHighFlow)});
    config.set_key_value("extruder_variant_list",
        new ConfigOptionStrings{"Direct Drive Standard", "Direct Drive High Flow"});
    config.set_key_value("min_layer_height", new ConfigOptionFloats{0.04, 0.08});
    config.set_key_value("max_layer_height", new ConfigOptionFloats{0.20, 0.60});
    config.set_key_value("layer_height", new ConfigOptionFloat(0.10));
    config.set_key_value("mixed_nozzle_coarse_layer_height", new ConfigOptionFloat(0.30));
    config.set_key_value("detect_overhang_wall", new ConfigOptionBool(false));
    for (const char *key : {"outer_wall_filament_id", "inner_wall_filament_id", "internal_solid_filament_id",
                            "top_surface_filament_id", "bottom_surface_filament_id"})
        config.set_key_value(key, new ConfigOptionInt(1));
    config.set_key_value("sparse_infill_filament_id", new ConfigOptionInt(2));
    config.set_key_value("mixed_nozzle_slicing_mode",
        new ConfigOptionEnum<MixedNozzleSlicingMode>(MixedNozzleSlicingMode::Off));
    config.set_key_value("prime_tower_width", new ConfigOptionFloat(50.));
    return config;
}

struct RebuildLedgerFixture {
    PresetBundle bundle;
    Model model;
    PartPlate current_plate;
    PartPlate other_plate;
    WizardNativeOwners owners;

    RebuildLedgerFixture()
        : current_plate(nullptr, Vec3d::Zero(), 200, 200, 200, nullptr, &model),
          other_plate(nullptr, Vec3d(220., 0., 0.), 200, 200, 200, nullptr, &model),
          owners{&bundle, &model, {&current_plate, &other_plate}, &current_plate}
    {
        const DynamicPrintConfig config = rebuild_ledger_config();
        current_plate.set_index(0);
        other_plate.set_index(1);
        DynamicPrintConfig process_config = config;
        for (const char *key : {"filament_map", "filament_map_mode", "filament_volume_map"})
            process_config.erase(key);
        bundle.prints.load_preset("", "process-A", process_config, true);
        bundle.printers.load_preset("", "printer-A", config, true);
        bundle.filaments.load_preset("", "material-0", config, true);
        bundle.filaments.load_preset("", "material-1", config, false);
        bundle.filament_presets = {"material-0", "material-1"};
        bundle.project_config = config;
        *current_plate.config() = config;
        *other_plate.config() = config;

        ModelObject *object = model.add_object();
        object->name = "Ledger cube";
        object->add_volume(make_cube(10., 10., 4.), ModelVolumeType::MODEL_PART, false);
        object->add_instance();
        current_plate.add_instance(0, 0, false);
    }
};

WizardDraft rebuild_feature_split_automatic_draft(const RebuildLedgerFixture &fixture)
{
    WizardDraft draft;
    draft.signature.selected_printer_id = fixture.bundle.printers.get_edited_preset().name;
    draft.signature.selected_process_id = fixture.bundle.prints.get_edited_preset().name;
    for (const PartPlate *plate : fixture.owners.plates)
        draft.signature.plate_ids.push_back(plate->id().id);
    for (const ModelObject *object : fixture.model.objects)
        for (const ModelVolume *volume : object->volumes) {
            WizardVolumeSignature signature;
            signature.object_id = object->id().id;
            signature.volume_id = volume->id().id;
            signature.config_revision = static_cast<const ModelConfig &>(volume->config).timestamp();
            draft.signature.volumes.push_back(signature);
        }
    draft.scope.kind = WizardScopeKind::CurrentPlate;
    draft.scope.current_plate_id = fixture.current_plate.id().id;
    draft.scope.process_affected_plate_ids = {fixture.current_plate.id().id, fixture.other_plate.id().id};
    draft.scope.allow_shared_process_changes = true;
    draft.mode = MixedNozzleSlicingMode::FeatureSplit;
    draft.resolved_physical_roles = {{0, 0, 0.20, "fine"}, {1, 1, 0.60, "coarse"}};
    draft.fine_logical_filament = 0;
    draft.coarse_logical_filament = 1;
    draft.chosen_fine_height = 0.10;
    draft.chosen_coarse_height = 0.40;
    draft.chosen_cadence_ratio = 4;
    draft.tower_intent = WizardTowerIntent::Automatic;
    DynamicPrintConfig proposed = fixture.bundle.prints.get_edited_preset().config;
    const std::string old_coarse = proposed.opt_serialize("mixed_nozzle_coarse_layer_height");
    proposed.set_key_value("mixed_nozzle_coarse_layer_height", new ConfigOptionFloat(0.40));
    draft.approved_key_deltas = {{"mixed_nozzle_coarse_layer_height", old_coarse,
                                  proposed.opt_serialize("mixed_nozzle_coarse_layer_height"),
                                  "Resolver", "Shared process", "mm"}};
    return draft;
}

WizardReview rebuild_approved_review_for(const WizardDraft &draft)
{
    WizardReview review;
    review.can_apply = true;
    review.entries = draft.approved_key_deltas;
    review.affected_plate_ids = draft.scope.process_affected_plate_ids;
    review.preserved_locks = draft.locked_keys;
    return review;
}

void rebuild_apply_feature_split_automatic(RebuildLedgerFixture &fixture)
{
    const WizardDraft draft = rebuild_feature_split_automatic_draft(fixture);
    PreparedWizardApply prepared = prepare_wizard_apply(draft, rebuild_approved_review_for(draft), fixture.owners);
    INFO(prepared.diagnostic);
    REQUIRE(prepared.diagnostic.empty());
    WizardApplyHooks hooks;
    hooks.take_snapshot = [] {};
    hooks.take_snapshot_with_setup_transition = [](const MixedNozzleSetupPlan &) { return true; };
    hooks.invalidate = [](std::size_t) {};
    hooks.notify_batch_changed = [] {};
    const WizardApplyResult applied = commit_wizard_apply(std::move(prepared), fixture.owners, hooks);
    INFO(applied.diagnostic);
    REQUIRE(applied.applied);
}

void rebuild_turn_off_current_plate(RebuildLedgerFixture &fixture)
{
    const MixedNozzleSetupRequest off = mixed_nozzle_off_request(false);
    const MixedNozzleSetupPlan plan = stage_mixed_nozzle_setup(fixture.bundle, fixture.owners.plates, off,
                                                               &fixture.current_plate);
    INFO(plan.diagnostic);
    REQUIRE(plan.diagnostic.empty());
    const MixedNozzleSetupHooks hooks{[] {}, [](PartPlate &) {}, [] {}};
    REQUIRE(commit_mixed_nozzle_setup_transaction(fixture.bundle, fixture.owners.plates, plan, off, hooks,
                                                  &fixture.current_plate));
    REQUIRE(fixture.current_plate.config()->opt_enum<MixedNozzleSlicingMode>("mixed_nozzle_slicing_mode") ==
            MixedNozzleSlicingMode::Off);
}

} // namespace

TEST_CASE("Turning the setup off restores untouched tower settings and preserves a later user reset",
          "[TestRebuild][Wizard][Tower]")
{
    SECTION("untouched Automatic settings return to their earlier values") {
        RebuildLedgerFixture fixture;
        const DynamicPrintConfig before = fixture.bundle.prints.get_edited_preset().config;
        rebuild_apply_feature_split_automatic(fixture);
        const DynamicPrintConfig &applied = fixture.bundle.prints.get_edited_preset().config;
        REQUIRE(applied.opt_float("prime_tower_width") < before.opt_float("prime_tower_width"));
        rebuild_turn_off_current_plate(fixture);
        const DynamicPrintConfig &restored = fixture.bundle.prints.get_edited_preset().config;
        CHECK_THAT(restored.opt_float("prime_tower_width"),
                   Catch::Matchers::WithinAbs(before.opt_float("prime_tower_width"), 0.01));
        CHECK(restored.opt_bool("wipe_tower_no_sparse_layers") == before.opt_bool("wipe_tower_no_sparse_layers"));
    }
    SECTION("a user reset remains after Turn off") {
        RebuildLedgerFixture fixture;
        rebuild_apply_feature_split_automatic(fixture);
        detach_mixed_nozzle_tower_key(fixture.bundle.project_config, MIXED_NOZZLE_TOWER_WIDTH_KEY,
                                      fixture.bundle.prints.get_edited_preset().name);
        fixture.bundle.prints.get_edited_preset().config.set_key_value(
            MIXED_NOZZLE_TOWER_WIDTH_KEY, new ConfigOptionFloat(60.));
        rebuild_turn_off_current_plate(fixture);
        CHECK_THAT(fixture.bundle.prints.get_edited_preset().config.opt_float(MIXED_NOZZLE_TOWER_WIDTH_KEY),
                   Catch::Matchers::WithinAbs(60., 0.01));
    }
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
    ScopedTemporaryFile archive(".3mf");
    const std::string path = archive.string();
    PlateData plate;
    plate.plate_index = 0;
    plate.config = *project.plate.config();
    StoreParams store;
    store.path = path.c_str();
    store.model = &project.model;
    store.config = &original.config;
    store.plate_data_list.push_back(&plate);
    store.strategy = SaveStrategy::Zip64 | SaveStrategy::Silence;
    REQUIRE(store_bbs_3mf(store));
    ScopedTemporaryFile branded(".3mf");
    REQUIRE(mark_native_archive_as_cadence(path, branded.string()));

    Model loaded_model;
    ScopedTemporaryDir loaded_backup("rebuild_workflow_loaded");
    loaded_model.set_backup_path(loaded_backup.string());
    DynamicPrintConfig loaded_config;
    ConfigSubstitutionContext substitutions {ForwardCompatibilitySubstitutionRule::Enable};
    PlateDataPtrs loaded_plates;
    std::vector<Preset *> embedded;
    bool is_bbl = false, is_orca = false;
    Semver version;
    REQUIRE(load_bbs_3mf(branded.string().c_str(), &loaded_config, &substitutions, &loaded_model, &loaded_plates,
                         &embedded, &is_bbl, &is_orca, &version, nullptr,
                         LoadStrategy::LoadModel | LoadStrategy::LoadConfig));
    REQUIRE(loaded_model.objects.size() == 1);
    CHECK(loaded_model.objects.front()->name == "two bodies");
    REQUIRE(loaded_model.objects.front()->instances.size() == 1);
    const Vec3d loaded_position = loaded_model.objects.front()->instances.front()->get_offset();
    CHECK_THAT(loaded_position.x(), Catch::Matchers::WithinAbs(100., .01));
    CHECK_THAT(loaded_position.y(), Catch::Matchers::WithinAbs(100., .01));
    REQUIRE_FALSE(loaded_plates.empty());
    loaded_config.apply(loaded_plates.front()->config);
    const auto *roles = loaded_config.option<ConfigOptionFloats>("mixed_nozzle_process_nozzle_diameters");
    REQUIRE(roles != nullptr);
    CHECK(roles->values.empty());
    Print print;
    print.set_status_silent();
    print.is_BBL_printer() = true;
    print.apply(loaded_model, loaded_config);
    print.apply(loaded_model, loaded_config);
    const CadenceTest::Facts replayed = CadenceTest::slice(print);
    INFO(replayed.refusal.string);
    REQUIRE(replayed.refusal.string.empty());
    PlateSlice reopened;
    reopened.config = loaded_config;
    reopened.gcode = replayed.gcode;
    reopened.moves = replayed.moves;
    check_body_split_print(reopened, 0, 1, 0.10, 0.30);

    // The loaded opaque policy must still own the escaped process identity:
    // the sidebar's real Turn off transaction restores the user's 60 mm
    // tower setting only when that ownership survived both JSON layers.
    const auto *loaded_policy = loaded_config.option<ConfigOptionString>(MIXED_NOZZLE_TOWER_POLICY_OPTION);
    REQUIRE(loaded_policy != nullptr);
    project.bundle.project_config.set_key_value(MIXED_NOZZLE_TOWER_POLICY_OPTION,
        new ConfigOptionString(loaded_policy->value));
    project.bundle.prints.get_edited_preset().config.set_key_value("prime_tower_width",
        new ConfigOptionFloat(loaded_config.opt_float("prime_tower_width")));
    REQUIRE(project.bundle.prints.get_edited_preset().config.opt_float("prime_tower_width") < 60.);
    const MixedNozzleSetupRequest off = mixed_nozzle_off_request(false);
    const MixedNozzleSetupPlan plan = stage_mixed_nozzle_setup(
        project.bundle, project.owners.plates, off, &project.plate);
    INFO(plan.diagnostic);
    REQUIRE(plan.diagnostic.empty());
    const MixedNozzleSetupHooks hooks{[] {}, [](PartPlate &) {}, [] {}};
    REQUIRE(commit_mixed_nozzle_setup_transaction(project.bundle, project.owners.plates,
                                                  plan, off, hooks, &project.plate));
    CHECK_THAT(project.bundle.prints.get_edited_preset().config.opt_float("prime_tower_width"),
               Catch::Matchers::WithinAbs(60., .01));
    release_PlateData_list(loaded_plates);
    for (Preset *preset : embedded)
        delete preset;

    // A copied object in a Cadence project reopens as one object with two
    // instances; a foreign archive would be split into separate objects.
    ModelObject *object = project.model.objects.front();
    ModelInstance *copy = object->add_instance(*object->instances.front());
    copy->set_offset(object->instances.front()->get_offset() + Vec3d(40., 0., 0.));
    ScopedTemporaryFile copies(".3mf");
    const std::string copies_path = copies.string();
    store.path = copies_path.c_str();
    REQUIRE(store_bbs_3mf(store));
    ScopedTemporaryFile branded_copies(".3mf");
    REQUIRE(mark_native_archive_as_cadence(copies_path, branded_copies.string()));
    Model copies_model;
    ScopedTemporaryDir copies_backup("rebuild_workflow_copies");
    copies_model.set_backup_path(copies_backup.string());
    DynamicPrintConfig copies_config;
    PlateDataPtrs copies_plates;
    std::vector<Preset *> copies_embedded;
    REQUIRE(load_bbs_3mf(branded_copies.string().c_str(), &copies_config, &substitutions, &copies_model,
                         &copies_plates, &copies_embedded, &is_bbl, &is_orca, &version, nullptr,
                         LoadStrategy::LoadModel | LoadStrategy::LoadConfig));
    CHECK(copies_model.objects.size() == 1);
    if (!copies_model.objects.empty())
        CHECK(copies_model.objects.front()->instances.size() == 2);
    release_PlateData_list(copies_plates);
    for (Preset *preset : copies_embedded)
        delete preset;
}

TEST_CASE("A wizard Body Split project reports the setting that prevents slicing",
          "[TestRebuild][Wizard][Refusal][BodySplit]")
{
    WizardProject project(h2d_02_06_spec());
    add_two_bodies(project);
    run_wizard_setup(project, body_choice(0.10, 3));
    const PlateSlice admitted = require_slice(project);

    SECTION("Arachne walls identify the wall generator setting") {
        DynamicPrintConfig incompatible = admitted.config;
        incompatible.set_key_value("wall_generator",
            new ConfigOptionEnum<PerimeterGeneratorType>(PerimeterGeneratorType::Arachne));
        Print print;
        print.set_status_silent();
        print.is_BBL_printer() = true;
        print.apply(project.model, incompatible);
        CHECK(print.validate().opt_key == "wall_generator");
    }
    SECTION("beams at the wrong cadence identify the beam layer setting") {
        ModelObject *object = project.model.objects.front();
        object->config.set_key_value("interlocking_beam", new ConfigOptionBool(true));
        object->config.set_key_value("interlocking_beam_layer_count", new ConfigOptionInt(2));
        Print print;
        print.set_status_silent();
        print.is_BBL_printer() = true;
        print.apply(project.model, admitted.config);
        CHECK(print.validate().opt_key == "interlocking_beam_layer_count");
    }
}

TEST_CASE("A plate-local mixed project keeps its saved material values after 3MF import and reslice",
          "[TestRebuild][Wizard][Persistence][Materials]")
{
    WizardProject project(h2d_02_06_spec());
    add_two_bodies(project);
    run_wizard_setup(project, body_choice(0.10, 3));
    REQUIRE(project.plate.config()->has("mixed_nozzle_slicing_mode"));
    const PlateSlice original = require_slice(project);
    check_body_split_print(original, 0, 1, 0.10, 0.30);

    DynamicPrintConfig saved = project.bundle.full_config(false);
    saved.set_key_value("mixed_nozzle_slicing_mode",
        new ConfigOptionEnum<MixedNozzleSlicingMode>(MixedNozzleSlicingMode::Off));
    auto *flow = saved.option<ConfigOptionFloats>("filament_flow_ratio", true);
    auto *temperature = saved.option<ConfigOptionInts>("nozzle_temperature", true);
    auto *initial_temperature = saved.option<ConfigOptionInts>("nozzle_temperature_initial_layer", true);
    REQUIRE(flow != nullptr);
    REQUIRE(temperature != nullptr);
    REQUIRE(initial_temperature != nullptr);
    REQUIRE(flow->values.size() >= 2);
    REQUIRE(temperature->values.size() >= 2);
    REQUIRE(initial_temperature->values.size() >= 2);
    std::fill(flow->values.begin(), flow->values.end(), 0.93);
    std::fill(temperature->values.begin(), temperature->values.end(), 237);
    std::fill(initial_temperature->values.begin(), initial_temperature->values.end(), 237);
    // This producer saved the material vectors without the per-preset dirty-key
    // mask. The project-default mode is Off, while the plate is Body Split. The
    // imported values still belong to the bound materials, not the stock preset.
    saved.erase("different_settings_to_system");
    for (const char *key : {"filament_flow_ratio", "nozzle_temperature",
                            "nozzle_temperature_initial_layer"})
        mixed_nozzle_ledger_add_key(saved, 2, {0, 1}, key);

    ScopedTemporaryDir backup("rebuild_plate_material_3mf");
    project.model.set_backup_path(backup.string());
    ScopedTemporaryFile archive(".3mf");
    const std::string path = archive.string();
    PlateData plate;
    plate.plate_index = 0;
    plate.config = *project.plate.config();
    plate.filament_maps = project.plate.get_real_filament_maps(project.bundle.project_config);
    StoreParams store;
    store.path = path;
    store.model = &project.model;
    store.config = &saved;
    store.plate_data_list.push_back(&plate);
    store.strategy = SaveStrategy::Zip64 | SaveStrategy::Silence;
    REQUIRE(store_bbs_3mf(store));

    Model loaded_model;
    ScopedTemporaryDir loaded_backup("rebuild_plate_material_loaded");
    loaded_model.set_backup_path(loaded_backup.string());
    DynamicPrintConfig loaded_config;
    ConfigSubstitutionContext substitutions {ForwardCompatibilitySubstitutionRule::Enable};
    PlateDataPtrs loaded_plates;
    std::vector<Preset *> embedded;
    bool is_bbl = false, is_orca = false;
    Semver version;
    REQUIRE(load_bbs_3mf(path.c_str(), &loaded_config, &substitutions, &loaded_model, &loaded_plates,
                         &embedded, &is_bbl, &is_orca, &version, nullptr,
                         LoadStrategy::LoadModel | LoadStrategy::LoadConfig));
    REQUIRE_FALSE(loaded_plates.empty());
    REQUIRE(loaded_config.opt_enum<MixedNozzleSlicingMode>("mixed_nozzle_slicing_mode") ==
            MixedNozzleSlicingMode::Off);
    REQUIRE(loaded_plates.front()->config.opt_enum<MixedNozzleSlicingMode>("mixed_nozzle_slicing_mode") ==
            MixedNozzleSlicingMode::BodySplit);

    PresetBundle reopened = installed_bbl_profiles();
    reopened.load_config_model(path, loaded_config);
    DynamicPrintConfig effective = reopened.full_config();
    const auto *reopened_flow = effective.option<ConfigOptionFloats>("filament_flow_ratio");
    const auto *reopened_temperature = effective.option<ConfigOptionInts>("nozzle_temperature");
    REQUIRE(reopened_flow != nullptr);
    REQUIRE(reopened_temperature != nullptr);
    REQUIRE(reopened_flow->values.size() >= 2);
    REQUIRE(reopened_temperature->values.size() >= 2);
    for (std::size_t slot = 0; slot < 2; ++slot) {
        CHECK_THAT(reopened_flow->values[slot], Catch::Matchers::WithinAbs(0.93, 1e-5));
        CHECK(reopened_temperature->values[slot] == 237);
    }

    effective.apply(loaded_plates.front()->config);
    Print print;
    print.set_status_silent();
    print.is_BBL_printer() = true;
    print.apply(loaded_model, effective);
    print.apply(loaded_model, effective);
    const CadenceTest::Facts replayed = CadenceTest::slice(print);
    INFO(replayed.refusal.string);
    REQUIRE(replayed.refusal.string.empty());
    PlateSlice resliced;
    resliced.config = effective;
    resliced.gcode = replayed.gcode;
    resliced.moves = replayed.moves;
    check_body_split_print(resliced, 0, 1, 0.10, 0.30);
    release_PlateData_list(loaded_plates);
    for (Preset *preset : embedded)
        delete preset;
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

        // A native import can rename the sibling's display inheritance while
        // retaining its vendor and material identity. Give that 0.6 mm sibling
        // a distinct flow so the exported coarse roads reveal which one won.
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
        // Keep the remembered process itself. Switching to an installed Body
        // preset would test that preset's availability rather than restored
        // selection, and would replace the selected process's first layer.
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

TEST_CASE("Malformed serialized process changes refuse wizard Apply without writing the project",
          "[TestRebuild][Wizard][Transaction]")
{
    RebuildLedgerFixture fixture;
    WizardDraft malformed = rebuild_feature_split_automatic_draft(fixture);
    REQUIRE_FALSE(malformed.approved_key_deltas.empty());
    malformed.approved_key_deltas.front().new_value = "not-a-layer-height";
    const DynamicPrintConfig original_process = fixture.bundle.prints.get_edited_preset().config;
    PreparedWizardApply refused = prepare_wizard_apply(
        malformed, rebuild_approved_review_for(malformed), fixture.owners);
    REQUIRE_FALSE(refused.diagnostic.empty());
    int snapshots = 0;
    int notifications = 0;
    WizardApplyHooks hooks;
    hooks.take_snapshot = [&] { ++snapshots; };
    hooks.take_snapshot_with_setup_transition = [&](const MixedNozzleSetupPlan &) {
        ++snapshots;
        return true;
    };
    hooks.invalidate = [](std::size_t) {};
    hooks.notify_batch_changed = [&] { ++notifications; };
    const WizardApplyResult rejected = commit_wizard_apply(std::move(refused), fixture.owners, hooks);
    CHECK_FALSE(rejected.applied);
    CHECK(snapshots == 0);
    CHECK(notifications == 0);
    CHECK(fixture.bundle.prints.get_edited_preset().config == original_process);

    const WizardDraft valid = rebuild_feature_split_automatic_draft(fixture);
    PreparedWizardApply prepared = prepare_wizard_apply(valid, rebuild_approved_review_for(valid), fixture.owners);
    INFO(prepared.diagnostic);
    REQUIRE(prepared.diagnostic.empty());
    const WizardApplyResult applied = commit_wizard_apply(std::move(prepared), fixture.owners, hooks);
    INFO(applied.diagnostic);
    REQUIRE(applied.applied);
    CHECK(snapshots == 1);
    CHECK(notifications == 1);
    CHECK(fixture.current_plate.get_real_mixed_nozzle_slicing_mode(fixture.bundle.project_config) ==
          MixedNozzleSlicingMode::FeatureSplit);
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

// Reported: a two-part object imported into a new project has both parts on slot 1. Step 1
// puts the small part on Fine and the body on Coarse, step 2 picks slot 4 (PETG Basic, moved to the
// left) for fine and slot 6 (PETG HF) for coarse. After Apply the small part showed slot 1 and the
// right nozzle's High Flow read Standard.
TEST_CASE("Wizard Apply gives each part its picked material and keeps High Flow",
          "[TestRebuild][WizardApply][BodySplit]")
{
    WizardProject project(h2d_02_04_high_flow_spec());
    prepare_fresh_high_flow_project(project);
    ModelObject *object = project.add_object("two-part", {{"Body11", Vec3d(40., 30., 8.), Vec3d::Zero(), 1},
                                                        {"Body12", Vec3d(10., 30., 8.), Vec3d(45., 0., 0.), 1}},
                                             Vec2d(100., 100.));
    object->config.set_key_value("extruder", new ConfigOptionInt(1));

    SetupChoice choice = body_choice(0.08, 2);
    choice.fine_slot = 3;
    choice.coarse_slot = 5;
    choice.body_roles = {WizardBodyRole::Coarse, WizardBodyRole::Fine};
    run_wizard_setup(project, choice);

    CHECK(object_list_slot(*object, *object->volumes[0]) == 6);
    CHECK(object_list_slot(*object, *object->volumes[1]) == 4);
    const auto *flows = project.bundle.project_config.option<ConfigOptionEnumsGeneric>("nozzle_volume_type");
    REQUIRE(flows != nullptr);
    CHECK(flows->values == kHighFlowRight);
    const PlateSlice slice = require_slice(project);
    CHECK(slice.config.option<ConfigOptionEnumsGeneric>("nozzle_volume_type")->values == kHighFlowRight);
}

// More options' exact slot wins over the role's material, and the other parts still follow step 2.
TEST_CASE("Wizard Apply keeps an exact slot from More options beside the picked materials",
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
}

TEST_CASE("Setup reopens on the tower choice the project has, as the sidebar says",
          "[TestRebuild][B33][Wizard][Tower]")
{
    WizardProject project(h2d_02_06_spec());
    add_cube(project);
    // What Change... opens More options on, and the sidebar's tower line, for the project now.
    const auto reopened = [&project] {
        const DynamicPrintConfig full = wizard_plate_effective_config(project.bundle, project.plate);
        const auto *mask = full.option<ConfigOptionStrings>("different_settings_to_system");
        const std::vector<std::string> values = mask == nullptr ? std::vector<std::string>{} : mask->values;
        const MixedNozzleSlicingMode mode = effective_mixed_nozzle_mode(project.bundle.project_config,
                                                                        project.plate).effective;
        return std::make_pair(mixed_nozzle_default_tower_intent(project.bundle.project_config, values, mode),
                              mixed_nozzle_sidebar_tower_line(project.bundle.project_config, values,
                                                              full.opt_bool("enable_prime_tower")));
    };
    REQUIRE(reopened().first == WizardTowerIntent::Automatic);
    SECTION("kept tower settings") {
        SetupChoice choice = feature_choice(0.10, 4);
        choice.tower = WizardTowerIntent::Preserve;
        run_wizard_setup(project, choice);
        const auto [intent, line] = reopened();
        CHECK(line == "Prime tower: your own settings");
        CHECK(intent == WizardTowerIntent::Preserve);
    }
    SECTION("automatic tower") {
        run_wizard_setup(project, feature_choice(0.10, 4));
        const auto [intent, line] = reopened();
        CHECK(line == "Prime tower: set up automatically");
        CHECK(intent == WizardTowerIntent::Automatic);
    }
}

// Owner, build 31, printer online: an H2D reports its right (main) nozzle first. The sync read each
// flow through the config slot and applied the 0.2 mm Standard rule to the other nozzle, so a right
// 0.4 High Flow came back Standard. Uses the H2D profile's own physical_extruder_map.
TEST_CASE("A printer sync keeps the right High Flow on an H2D with a 0.2 left nozzle",
          "[TestRebuild][WizardApply][DeviceSync]")
{
    WizardProject project(h2d_02_04_high_flow_spec());
    const auto *map_option = project.bundle.printers.get_edited_preset().config.option<ConfigOptionInts>("physical_extruder_map");
    REQUIRE(map_option != nullptr);
    const std::vector<int> map = map_option->values;
    REQUIRE(map == std::vector<int>{1, 0});
    // Device order: extruder 0 is the right 0.4 High Flow, extruder 1 the left 0.2 Standard.
    const std::vector<double> diameters{0.4, 0.2};
    const std::vector<std::optional<NozzleVolumeType>> flows{nvtHighFlow, nvtStandard};
    const std::vector<NozzleVolumeType> types = device_sync_nozzle_volume_types(map, diameters, flows, {{}, {}});
    CHECK(types == std::vector<NozzleVolumeType>{nvtStandard, nvtHighFlow});

    // A 0.2 mm nozzle stays Standard whatever it reports; a stats selection wins.
    CHECK(device_sync_nozzle_volume_types(map, diameters, {nvtHighFlow, nvtHighFlow}, {{}, {}}) ==
          std::vector<NozzleVolumeType>{nvtStandard, nvtHighFlow});
    CHECK(device_sync_nozzle_volume_types(map, diameters, flows, {std::nullopt, nvtHybrid}) ==
          std::vector<NozzleVolumeType>{nvtHybrid, nvtHighFlow});
    // Without flow support every nozzle is Standard, as upstream.
    CHECK(device_sync_nozzle_volume_types(map, diameters, {{}, {}}, {{}, {}}) ==
          std::vector<NozzleVolumeType>{nvtStandard, nvtStandard});
}
