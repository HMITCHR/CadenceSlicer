#include <catch2/catch_all.hpp>

#include "wizard_apply_support.hpp"
#include "fff_print/mixed_nozzle_harness.hpp"
#include "slic3r/GUI/BodySplitEditorModel.hpp"
#include "slic3r/GUI/MixedNozzleWizardModel.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <set>
#include <vector>

using namespace Slic3r;
using namespace Slic3r::GUI;
using namespace Slic3r::GUI::WizardTest;

namespace {

void add_body_pair(WizardProject &project)
{
    project.add_object("two bodies", {{"left body", Vec3d(10., 20., 10.), Vec3d::Zero(), 1},
                                       {"right body", Vec3d(10., 20., 10.), Vec3d(15., 0., 0.), 2}},
                       Vec2d(100., 100.));
}

SetupChoice choice(MixedNozzleSlicingMode mode)
{
    SetupChoice result;
    result.mode = mode;
    result.fine_height = .10;
    result.ratio = 3;
    result.joining = JoiningChoice::Off;
    return result;
}

void check_body_routes(const Emitted::EmittedFacts &facts)
{
    std::set<int> left_tools, left_materials, right_tools, right_materials;
    float left_height = 0.f, right_height = 0.f;
    for (const auto &fact : facts.role_facts) {
        if (fact.role != erExternalPerimeter)
            continue;
        if (fact.max_x < 113.f) {
            left_tools.insert(fact.physical_tool);
            left_materials.insert(fact.logical_filament);
            left_height = std::max(left_height, fact.height);
        } else if (fact.min_x > 112.f) {
            right_tools.insert(fact.physical_tool);
            right_materials.insert(fact.logical_filament);
            right_height = std::max(right_height, fact.height);
        }
    }
    CHECK(left_tools == std::set<int>{0});
    CHECK(left_materials == std::set<int>{0});
    CHECK(right_tools == std::set<int>{1});
    CHECK(right_materials == std::set<int>{2});
    CHECK_THAT(left_height, Catch::Matchers::WithinAbs(.10, .011));
    CHECK_THAT(right_height, Catch::Matchers::WithinAbs(.30, .011));
}

} // namespace

TEST_CASE("Body editor material reassignment survives native 3MF reload and slices on the selected nozzles",
          "[TestRebuild][E2E][BodySplit][Editor][Persistence]")
{
    ProfileSpec spec = h2d_02_06_spec();
    spec.materials.push_back(spec.materials.back());
    WizardProject project(spec);
    add_body_pair(project);
    REQUIRE(run_wizard_setup(project, choice(MixedNozzleSlicingMode::BodySplit)).applied);

    const auto maps = project.plate.get_real_filament_maps(project.bundle.project_config);
    const DynamicPrintConfig full = wizard_plate_effective_config(project.bundle, project.plate);
    PrintConfig resolver = mixed_nozzle_resolver_config_from_full(full);
    resolver.filament_map_mode.value = project.plate.get_real_filament_map_mode(project.bundle.project_config);
    resolver.filament_map.values = maps;
    const double inherited = full.opt_float("layer_height");
    const auto editor = build_body_split_editor_model(project.plate.get_objects_on_this_plate(), resolver, {}, inherited);
    REQUIRE(editor.supported);
    REQUIRE(editor.rows.size() == 2);
    std::vector<BodySplitNativeSelection> selections;
    for (const auto &row : editor.rows) {
        const bool rebind = row.volume_name == "right body";
        const int slot = rebind ? 2 : 0;
        const double height = rebind ? .30 : .10;
        auto found = std::find_if(row.cadence_choices.begin(), row.cadence_choices.end(),
            [&](double candidate) { return std::abs(candidate - height) < 1e-9; });
        REQUIRE(found != row.cadence_choices.end());
        selections.push_back({row.object_id, row.volume_id, slot, int(found - row.cadence_choices.begin())});
    }
    const auto live = build_body_split_live_request(editor.rows, selections, project.bundle.filament_presets.size(),
                                                     resolver, RegionalGridPhaseRule::KeepNominal);
    REQUIRE(live.request.has_value());
    BodySplitApplyRequest request = *live.request;
    request.inherited_layer_height = inherited;
    BodySplitEditorTransaction transaction;
    REQUIRE(transaction.stage(request));
    int snapshots = 0, notifications = 0;
    const auto accepted = accept_body_split_plate_settings(
        {project.plate.get_index(), project.plate.id().id}, project.plate.get_index(), project.plate.id().id,
        MixedNozzleSlicingMode::BodySplit, transaction, project.model, *project.plate.config(), resolver,
        {[&] { ++snapshots; }, [&] { ++notifications; }});
    REQUIRE(accepted.acceptance == BodySplitPlateSettingsAcceptance::Committed);
    REQUIRE(accepted.apply_result.changed);
    CHECK(snapshots == 1);
    CHECK(notifications == 1);

    DynamicPrintConfig config = wizard_plate_effective_config(project.bundle, project.plate);
    config.apply(*project.plate.config());
    ScopedTemporaryDir backup("rebuild_editor_body_3mf");
    project.model.set_backup_path(backup.string());
    ScopedTemporaryFile archive(".3mf");
    PlateData plate;
    plate.plate_index = 0;
    plate.config = *project.plate.config();
    save_project_3mf(project.model, config, plate, archive.string());
    const ReopenedProject loaded(archive.string());
    Print print;
    print.set_status_silent();
    print.is_BBL_printer() = true;
    print.apply(loaded.model, loaded.slice_config());
    const CadenceTest::Facts sliced = CadenceTest::slice(print);
    INFO(sliced.refusal.string);
    REQUIRE(sliced.refusal.string.empty());
    check_body_routes(sliced.emitted);
}

TEST_CASE("A swapped mixed-nozzle process keeps its printed roads after importing exported G-code settings",
          "[TestRebuild][E2E][Persistence][SwappedProcess]")
{
    auto spec = h2d_02_06_spec();
    spec.nozzles = {.6, .2};
    spec.min_heights = {.12, .04};
    spec.max_heights = {.42, .14};
    WizardProject project(spec);
    auto &bundle = project.bundle;
    bundle.project_config.set_key_value("filament_map", new ConfigOptionInts{2, 1});
    bundle.project_config.set_key_value("mixed_nozzle_slicing_mode",
        new ConfigOptionEnum<MixedNozzleSlicingMode>(MixedNozzleSlicingMode::FeatureSplit));
    bundle.project_config.option<ConfigOptionEnumsGeneric>("nozzle_volume_type", true)->values =
        {int(nvtStandard), int(nvtHighFlow)};
    bundle.update_compatible(PresetSelectCompatibleType::Never);
    std::string process;
    for (const Preset &preset : bundle.prints)
        if (preset.name.rfind("MN Feature 0.2-0.6 ", 0) == 0 && preset.is_compatible &&
            std::abs(preset.config.opt_float("layer_height") - .1) < 1e-6 &&
            std::abs(preset.config.opt_float("mixed_nozzle_coarse_layer_height") - .4) < 1e-6)
            process = preset.name;
    REQUIRE_FALSE(process.empty());
    REQUIRE(bundle.prints.select_preset_by_name(process, true));
    auto config = bundle.full_config(false);
    config.set_key_value("filament_map_mode", new ConfigOptionEnum<FilamentMapMode>(fmmManual));
    config.set_key_value("enable_prime_tower", new ConfigOptionBool(true));
    config.set_key_value("wipe_tower_x", new ConfigOptionFloats{150.});
    config.set_key_value("wipe_tower_y", new ConfigOptionFloats{150.});
    for (const char *key : {"outer_wall_filament_id", "inner_wall_filament_id",
                           "internal_solid_filament_id", "top_surface_filament_id", "bottom_surface_filament_id"})
        config.set_key_value(key, new ConfigOptionInt(1));
    config.set_key_value("sparse_infill_filament_id", new ConfigOptionInt(2));
    CadenceTest::Scene scene;
    scene.config = config;
    scene.populate = [](Model &model, Print &print, const DynamicPrintConfig &settings) {
        mixed_nozzle_fixtures::init_feature_flow_fixture(model, print, settings, 4., 80.);
        model.objects.front()->instances.front()->set_offset(Vec3d(40., 40., 0.));
        print.apply(model, settings);
    };
    const auto before = CadenceTest::slice(scene);
    INFO(before.refusal.string);
    REQUIRE(before.refusal.string.empty());
    REQUIRE_FALSE(before.gcode.empty());
    DynamicPrintConfig imported;
    ScopedTemporaryFile saved_gcode(".gcode");
    { std::ofstream output(saved_gcode.string(), std::ios::binary); output << before.gcode; }
    REQUIRE_NOTHROW(imported.load_from_gcode_file(saved_gcode.string(),
                                                ForwardCompatibilitySubstitutionRule::Disable));
    scene.config = imported;
    const auto after = CadenceTest::slice(scene);
    INFO(after.refusal.string);
    REQUIRE(after.refusal.string.empty());
    const auto walls_and_core = [](const CadenceTest::Facts &facts) {
        std::vector<std::array<double, 5>> roads;
        for (const auto &move : facts.moves)
            if (CadenceTest::model_road(move))
                roads.push_back({double(move.physical_tool_id), double(move.extrusion_role),
                                 double(move.height), double(move.width), double(move.feedrate)});
        return roads;
    };
    const auto expected = walls_and_core(before), actual = walls_and_core(after);
    REQUIRE_FALSE(expected.empty());
    CHECK(std::any_of(expected.begin(), expected.end(), [](const auto &r) { return r[0] == 0; }));
    CHECK(std::any_of(expected.begin(), expected.end(), [](const auto &r) { return r[0] == 1; }));
    REQUIRE(actual.size() == expected.size());
    CHECK(actual == expected);
    CHECK(after.emitted.tool_changes.size() == before.emitted.tool_changes.size());
}

TEST_CASE("Setup opens a fresh project on Body Split for a freshly loaded two-part object",
          "[TestRebuild][BodySplit][WizardDefaultMode]")
{
    // A new project has no saved mode, so step 1 follows the live model.
    WizardProject project(h2d_02_06_spec());
    const auto configured = project.bundle.project_config.opt_enum<MixedNozzleSlicingMode>("mixed_nozzle_slicing_mode");
    REQUIRE(configured == MixedNozzleSlicingMode::Off);
    const DynamicPrintConfig full = wizard_plate_effective_config(project.bundle, project.plate);
    const PrintConfig resolver = mixed_nozzle_resolver_config_from_full(full);
    const auto opens_on = [&] {
        const auto editor = build_body_split_editor_model(project.plate.get_objects_on_this_plate(), resolver, {},
                                                          full.opt_float("layer_height"));
        return wizard_default_mode(configured, body_split_rows_have_body_object(editor.rows));
    };

    project.add_object("one body", {{"only body", Vec3d(10., 10., 5.), Vec3d::Zero(), 1}}, Vec2d(60., 60.));
    CHECK(opens_on() == MixedNozzleSlicingMode::FeatureSplit);

    SECTION("both parts on one filament") {
        project.add_object("two bodies", {{"base", Vec3d(20., 20., 5.), Vec3d::Zero(), 1},
                                           {"lid", Vec3d(20., 20., 2.), Vec3d(0., 0., 5.), 1}},
                           Vec2d(120., 120.));
        CHECK(opens_on() == MixedNozzleSlicingMode::BodySplit);
    }
    SECTION("neither part has a filament yet") {
        project.add_object("two bodies", {{"base", Vec3d(20., 20., 5.), Vec3d::Zero(), 0},
                                           {"lid", Vec3d(20., 20., 2.), Vec3d(0., 0., 5.), 0}},
                           Vec2d(120., 120.));
        CHECK(opens_on() == MixedNozzleSlicingMode::BodySplit);
    }
}
