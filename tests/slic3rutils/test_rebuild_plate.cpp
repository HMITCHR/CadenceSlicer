#include <catch2/catch_all.hpp>

#include "wizard_apply_support.hpp"
#include "fff_print/rebuild_harness.hpp"
#include "slic3r/GUI/FeatureSplitEditorNative.hpp"
#include "slic3r/GUI/FeatureSplitEditorSupport.hpp"

#include <algorithm>
#include <map>
#include <utility>

using namespace Slic3r;
using namespace Slic3r::GUI;
using namespace Slic3r::GUI::WizardTest;

namespace {

struct ModelRoads {
    size_t count = 0;
    float tallest = 0.f;
};

using RoadSummary = std::map<unsigned, ModelRoads>;

RoadSummary model_walls(const CadenceTest::Facts &facts)
{
    RoadSummary roads;
    for (const auto &move : facts.moves) {
        if (move.type != EMoveType::Extrude || move.extrusion_role != erExternalPerimeter)
            continue;
        ModelRoads &road = roads[move.physical_tool_id];
        ++road.count;
        road.tallest = std::max(road.tallest, move.height);
    }
    return roads;
}

CadenceTest::Facts slice_one_plate(WizardProject &project, PartPlate &plate)
{
    Model single_plate;
    for (const ModelObject *object : plate.get_objects_on_this_plate())
        single_plate.add_object(*object);
    DynamicPrintConfig config = plate_slice_config(project, plate);
    srl_fixtures::fill_per_filament_values(config);
    Print print;
    print.set_status_silent();
    print.is_BBL_printer() = true;
    print.set_plate_origin(plate.get_origin());
    print.apply(single_plate, config);
    const CadenceTest::Facts facts = CadenceTest::slice(print);
    const std::string &digest = print.feature_economics_whole_print_digest();
    UNSCOPED_INFO("plate " << plate.get_index() << " economics: " << digest.substr(0, digest.find('\n'))
                  << " caps=" << config.opt_serialize("filament_max_volumetric_speed")
                  << " coarse=" << config.opt_serialize("mixed_nozzle_coarse_layer_height"));
    INFO(facts.refusal.string);
    REQUIRE(facts.refusal.string.empty());
    REQUIRE_FALSE(facts.gcode.empty());
    return facts;
}

} // namespace

TEST_CASE("Feature handoff economics leave another plate's Body Split roads on their assigned nozzles",
          "[TestRebuild][PlateIsolation][BodySplit][Economics]")
{
    WizardProject project(h2d_02_06_spec());
    project.add_object("two bodies",
        {{"fine body", Vec3d(10., 20., 10.), Vec3d::Zero(), 1},
         {"coarse body", Vec3d(10., 20., 10.), Vec3d(15., 0., 0.), 2}},
        Vec2d(80., 80.));
    SetupChoice body;
    body.mode = MixedNozzleSlicingMode::BodySplit;
    body.fine_height = .10;
    body.ratio = 3;
    body.joining = JoiningChoice::Off;
    const WizardApplyResult applied = run_wizard_setup(project, body);
    REQUIRE(applied.applied);

    // A second plate uses the same printer, process and materials, but runs Feature Split.
    // Only its own object belongs to it. The two plates are deliberately kept in one project.
    PartPlate feature_plate(nullptr, Vec3d(350., 0., 0.), 350, 320, 325, nullptr, &project.model);
    feature_plate.set_index(1);
    ModelObject *feature = project.model.add_object();
    feature->name = "feature plate box";
    feature->add_volume(make_cube(70., 70., 8.), ModelVolumeType::MODEL_PART, false);
    // At the process's 15% the engine prices this box as faster all fine. A denser core on this
    // plate's own object earns coarse bands without touching the Body plate.
    feature->config.set_key_value("sparse_infill_density", new ConfigOptionPercent(40.));
    feature->add_instance()->set_offset(Vec3d(410., 70., 0.));
    REQUIRE(feature_plate.add_instance(1, 0, false) == 0);
    project.owners.plates.push_back(&feature_plate);
    feature_plate.config()->set_key_value("mixed_nozzle_slicing_mode",
        new ConfigOptionEnum<MixedNozzleSlicingMode>(MixedNozzleSlicingMode::FeatureSplit));
    // Select Manual grouping on the new plate, reusing the project's physical assignments.
    feature_plate.config()->set_key_value("filament_map_mode",
        new ConfigOptionEnum<FilamentMapMode>(fmmManual));
    const auto maps = feature_plate.get_real_filament_maps(project.bundle.project_config);
    const DynamicPrintConfig full = wizard_plate_effective_config(project.bundle, feature_plate);
    PrintConfig resolver = feature_split_resolver_config_from_full(full);
    resolver.filament_map_mode.value = feature_plate.get_real_filament_map_mode(project.bundle.project_config);
    resolver.filament_map.values = maps;
    const auto mode_source = effective_mixed_nozzle_mode(project.bundle.project_config, feature_plate);
    const MixedNozzleSlicingMode mode = mode_source.effective;
    const auto *nozzles = full.option<ConfigOptionFloats>("nozzle_diameter");
    REQUIRE(nozzles != nullptr);
    const FeatureSplitStableTarget target {feature_plate.get_index(), feature_plate.id().id};
    auto live = build_feature_split_native_live_apply(target, {0, 1}, feature_plate.get_index(),
        feature_plate.id().id, mode, nozzles->values.size(), project.bundle.filament_presets.size(), resolver);
    INFO("Feature plate mode=" << int(mode) << " inherited=" << mode_source.inherits_project
         << " project default=" << int(mixed_nozzle_project_default(project.bundle.project_config))
         << " local map count=" << feature_plate.get_filament_maps().size()
         << " effective map count=" << maps.size()
         << " first=" << (maps.empty() ? -1 : maps[0])
         << " second=" << (maps.size() < 2 ? -1 : maps[1])
         << " nozzle count=" << nozzles->values.size() << " logical filament count="
         << project.bundle.filament_presets.size() << " editor diagnostic=" << int(live.state.diagnostic)
         << " status=" << live.state.full_status);
    REQUIRE(mode == MixedNozzleSlicingMode::FeatureSplit);
    REQUIRE(live.request.has_value());
    live.request->coarse_layer_height = .40;
    const FeatureSplitApplyResult editor = commit_feature_split_editor_request(
        project.bundle.project_config, project.bundle.prints.get_edited_preset().config, feature_plate,
        mode, feature_plate.get_index(), target.plate_id, nozzles->values.size(), resolver,
        feature_split_coarse_cadence_choices(full.opt_float("layer_height"), resolver), *live.request);
    REQUIRE(editor.applied);
    REQUIRE(editor.changed);

    const CadenceTest::Facts body_before = slice_one_plate(project, project.plate);
    const RoadSummary before = model_walls(body_before);
    REQUIRE(before.count(0) == 1);
    REQUIRE(before.count(1) == 1);
    REQUIRE(before.at(0).count > 0);
    REQUIRE(before.at(1).count > 0);
    CHECK_THAT(before.at(0).tallest, Catch::Matchers::WithinAbs(.10, .011));
    CHECK_THAT(before.at(1).tallest, Catch::Matchers::WithinAbs(.30, .011));

    const CadenceTest::Facts feature_before = slice_one_plate(project, feature_plate);
    REQUIRE_FALSE(feature_before.emitted.role_facts.empty());
    CHECK(std::any_of(feature_before.emitted.role_facts.begin(), feature_before.emitted.role_facts.end(),
        [](const auto &fact) { return fact.role == erExternalPerimeter && fact.physical_tool == 0; }));
    CHECK(std::any_of(feature_before.emitted.role_facts.begin(), feature_before.emitted.role_facts.end(),
        [](const auto &fact) { return fact.role == erInternalInfill && fact.physical_tool == 1; }));
    REQUIRE_FALSE(feature_before.emitted.plan_malformed);
    // Process Tab economics can change the Feature band's choice. The Body plate has no
    // Feature bands to price; its own wizard assignments and emitted roads must survive.
    project.bundle.prints.get_edited_preset().config.set_key_value(
        "mixed_nozzle_handoff_cost_s", new ConfigOptionFloat(10000.));
    const CadenceTest::Facts feature_after = slice_one_plate(project, feature_plate);
    const CadenceTest::Facts body_after = slice_one_plate(project, project.plate);
    REQUIRE_FALSE(feature_after.emitted.plan_malformed);
    REQUIRE_FALSE(body_after.emitted.plan_malformed);

    const RoadSummary after = model_walls(body_after);
    REQUIRE(after.size() == before.size());
    for (const auto &[tool, roads] : before) {
        REQUIRE(after.count(tool) == 1);
        CHECK(after.at(tool).count == roads.count);
        CHECK_THAT(after.at(tool).tallest, Catch::Matchers::WithinAbs(roads.tallest, .001));
    }
    CHECK(body_after.emitted.tool_changes.size() == body_before.emitted.tool_changes.size());
    // As Slice reads them: the composed config with each plate's own settings on top.
    DynamicPrintConfig body_settings = wizard_plate_effective_config(project.bundle, project.plate);
    body_settings.apply(*project.plate.config());
    DynamicPrintConfig feature_settings = wizard_plate_effective_config(project.bundle, feature_plate);
    feature_settings.apply(*feature_plate.config());
    CHECK(body_settings.opt_enum<MixedNozzleSlicingMode>("mixed_nozzle_slicing_mode") ==
          MixedNozzleSlicingMode::BodySplit);
    CHECK(feature_settings.opt_enum<MixedNozzleSlicingMode>("mixed_nozzle_slicing_mode") ==
          MixedNozzleSlicingMode::FeatureSplit);
}
