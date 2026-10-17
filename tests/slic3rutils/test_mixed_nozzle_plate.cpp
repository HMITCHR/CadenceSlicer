#include <catch2/catch_all.hpp>

#include "wizard_apply_support.hpp"
#include "fff_print/mixed_nozzle_harness.hpp"
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
    mixed_nozzle_fixtures::fill_per_filament_values(config);
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

// The tester's 8-plate project: setup over the whole project put Feature Split on every plate but
// gave only the plate it was opened from its fine and coarse materials. Every other plate then
// sliced with no Feature materials and Slice refused it ("The fine and coarse materials must print
// on different nozzles").
TEST_CASE("Whole-project Feature Split setup gives every plate its Feature materials, so every plate slices",
          "[TestRebuild][PlateIsolation][FeatureScene]")
{
    WizardProject project(h2d_02_06_spec());
    project.add_object("plate 1 box", {{"box", Vec3d(30., 30., 6.), Vec3d::Zero(), 1}}, Vec2d(80., 80.));
    PartPlate second(nullptr, Vec3d(350., 0., 0.), 350, 320, 325, nullptr, &project.model);
    second.set_index(1);
    ModelObject *box = project.model.add_object();
    box->name = "plate 2 box";
    box->add_volume(make_cube(30., 30., 6.), ModelVolumeType::MODEL_PART, false);
    box->add_instance()->set_offset(Vec3d(430., 80., 0.));
    REQUIRE(second.add_instance(1, 0, false) == 0);
    second.config()->set_key_value("filament_map_mode", new ConfigOptionEnum<FilamentMapMode>(fmmManual));
    second.config()->set_key_value("filament_map", new ConfigOptionInts{1, 2});
    project.owners.plates.push_back(&second);

    SetupChoice choice;
    choice.mode = MixedNozzleSlicingMode::FeatureSplit;
    choice.fine_height = .10;
    choice.ratio = 3;
    choice.scope = WizardScopeKind::ProjectDefault;
    REQUIRE(run_wizard_setup(project, choice).applied);

    for (PartPlate *plate : project.owners.plates) {
        CAPTURE(plate->get_index());
        const DynamicPrintConfig config = plate_slice_config(project, *plate);
        CHECK(config.opt_serialize("mixed_nozzle_slicing_mode") == "feature_split");
        CHECK(config.opt_int("outer_wall_filament_id") == 1);
        CHECK(config.opt_int("internal_solid_filament_id") == 1);
        CHECK(config.opt_int("sparse_infill_filament_id") == 2);
    }
    // Plate 2 slices, with its walls on the fine nozzle.
    const CadenceTest::Facts facts = slice_one_plate(project, second);
    CHECK(std::any_of(facts.emitted.role_facts.begin(), facts.emitted.role_facts.end(),
        [](const auto &fact) { return fact.role == erExternalPerimeter && fact.physical_tool == 0; }));
}

// Setup's step 3 times come from a slice of a copied (staged) bundle. On the tester's 0.4 / 0.8
// project the copy composed the 0.8 nozzle's retraction as 0.8 mm, while the project itself (and so
// the real Slice) gives it the 0.8 profile's 3 mm: the copied bundle's edited printer preset kept a
// vendor pointer into the original bundle, so the sibling 0.8 profile was never found. The tower
// time, and with it the whole-print time check, then differed between the row and the real slice.
TEST_CASE("Setup's staged slice config gives the other nozzle size its own retraction, as the real slice does",
          "[TestRebuild][PlateIsolation][FeatureScene]")
{
    // Setup opened from the 0.4 printer, and from the 0.8 printer (setup moves the project to the 0.4
    // one, so the staged copy must select it too, not only take its values).
    const bool from_08 = GENERATE(false, true);
    CAPTURE(from_08);
    ProfileSpec spec;
    spec.vendor = "BBL";
    spec.printer = from_08 ? "Bambu Lab H2D 0.8 nozzle" : "Bambu Lab H2D 0.4 nozzle";
    spec.process = from_08 ? "0.40mm Standard @BBL H2D 0.8 nozzle" : "0.20mm Balanced Strength @BBL H2D";
    spec.materials = {"Generic ABS @BBL H2D", "Generic ABS @BBL H2D"};
    spec.nozzles = {0.4, 0.8};
    spec.min_heights = {0.08, 0.16};
    spec.max_heights = {0.28, 0.56};
    spec.tower = Vec2d(260., 120.);
    WizardProject project(spec);
    project.add_object("box", {{"box", Vec3d(30., 30., 6.), Vec3d::Zero(), 1}}, Vec2d(80., 80.));

    std::vector<int> maps = project.plate.get_real_filament_maps(project.bundle.project_config);
    std::vector<int> volume_maps = project.bundle.get_default_nozzle_volume_types_for_filaments(maps);
    const std::string original = project.bundle.full_config(false, maps, volume_maps).opt_serialize("retraction_length");
    PresetBundle copy(project.bundle);
    const std::string copied = copy.full_config(false, maps, volume_maps).opt_serialize("retraction_length");
    INFO("original " << original << " copy " << copied);
    // The other nozzle's columns come from its own size's profile (0.4 profile 0.8 mm, 0.8 profile 3 mm).
    CHECK(original.find('3') != std::string::npos);
    CHECK(original.find("0.8") != std::string::npos);
    CHECK(copied == original);

    // The step 3 row's config and the plate's config after Apply agree.
    SetupChoice choice;
    choice.mode = MixedNozzleSlicingMode::FeatureSplit;
    choice.fine_slot = 1;
    choice.coarse_slot = 0;
    choice.fine_height = .20;
    choice.ratio = 2;
    choice.scope = WizardScopeKind::ProjectDefault;
    const WizardSetupStage stage = build_wizard_setup_stage(project, choice);
    const MixedNozzleRankingSlice row = wizard_candidate_staged_slice(stage.draft, stage.candidate, stage.body, project.owners);
    REQUIRE(row.model);
    REQUIRE(run_wizard_setup(project, choice).applied);
    const DynamicPrintConfig applied = plate_slice_config(project, project.plate);
    CHECK(row.config.opt_serialize("retraction_length") == applied.opt_serialize("retraction_length"));
}

namespace {

// Model's process-wide automatic-brim inputs, put back when a test that sets them ends.
struct BrimInputsGuard {
    std::map<size_t, ExtruderParams> extruders = Model::extruderParamsMap;
    GlobalSpeedMap speeds = Model::printSpeedMap;
    ~BrimInputsGuard()
    {
        Model::extruderParamsMap = extruders;
        Model::printSpeedMap = speeds;
    }
};

// What the Slice button does first (Plater::priv::on_action_slice_plate): the brim inputs from the
// project's composed config.
void set_brim_inputs_as_slice_does(WizardProject &project, const DynamicPrintConfig &plate_config)
{
    const DynamicPrintConfig full = project.bundle.full_config();
    PrintConfig print_config;
    print_config.apply(plate_config, true);
    Model::setExtruderParams(full, int(project.bundle.filament_presets.size()));
    Model::setPrintSpeedTable(full, print_config);
}

double brim_extrusion(const std::vector<GCodeProcessorResult::MoveVertex> &moves)
{
    double mm = 0.;
    for (const auto &move : moves)
        if (move.type == EMoveType::Extrude && move.extrusion_role == erBrim && move.delta_extruder > 0.f)
            mm += move.delta_extruder;
    return mm;
}

ProfileSpec h2d_04_08_abs_spec()
{
    ProfileSpec spec;
    spec.vendor = "BBL";
    spec.printer = "Bambu Lab H2D 0.4 nozzle";
    spec.process = "0.20mm Balanced Strength @BBL H2D";
    spec.materials = {"Generic ABS @BBL H2D", "Generic ABS @BBL H2D"};
    spec.nozzles = {0.4, 0.8};
    spec.min_heights = {0.08, 0.16};
    spec.max_heights = {0.28, 0.56};
    spec.tower = Vec2d(260., 120.);
    return spec;
}

double normal_time(const PrintEstimatedStatistics &statistics)
{
    return statistics.modes[size_t(PrintEstimatedStatistics::ETimeMode::Normal)].time;
}

} // namespace

// The automatic brim's width comes from Model's process-wide material and speed tables, which the
// plater fills only when Slice is pressed. Step 3 timed its rows with whatever they held (nothing
// in a fresh session), so on the tester's ABS file step 3 said about 2 h 37 min for plate 1 and the
// slice after Apply, with a brim, about 2 h 42 min.
TEST_CASE("Step 3 times the automatic brim the slice after Apply prints",
          "[TestRebuild][PlateIsolation][FeatureScene]")
{
    const BrimInputsGuard guard;
    // As in a fresh session, before any Slice.
    Model::extruderParamsMap = {{0, {"", 0, 0}}};
    Model::printSpeedMap = GlobalSpeedMap{};
    WizardProject project(h2d_04_08_abs_spec());
    // ABS does not go on the default Cool Plate.
    project.bundle.project_config.set_deserialize_strict("curr_bed_type", "Textured PEI Plate");
    project.plate.config()->set_deserialize_strict("curr_bed_type", "Textured PEI Plate");
    project.bundle.prints.get_edited_preset().config.set_key_value("brim_type", new ConfigOptionEnum<BrimType>(btAutoBrim));
    // ABS, 60 x 60 x 40 mm: the automatic brim is about 6.8 mm for ABS, none for an unknown material.
    project.add_object("tall box", {{"box", Vec3d(60., 60., 40.), Vec3d::Zero(), 1}}, Vec2d(120., 120.));
    SetupChoice choice;
    choice.mode = MixedNozzleSlicingMode::FeatureSplit;
    choice.fine_slot = 1;
    choice.coarse_slot = 0;
    choice.fine_height = .20;
    choice.ratio = 2;
    choice.scope = WizardScopeKind::ProjectDefault;
    const WizardSetupStage stage = build_wizard_setup_stage(project, choice);
    const MixedNozzleRankingSlice row = wizard_candidate_staged_slice(stage.draft, stage.candidate, stage.body, project.owners);
    REQUIRE(row.model);
    MixedNozzleRankingCancel cancel;
    const MixedNozzleSliceTime row_time = mixed_nozzle_slice_time(*row.model, row.config, row.is_bbl_printer,
                                                                  row.plate_origin, {}, cancel, row.plate_index);
    INFO(row_time.diagnostic);
    REQUIRE(row_time.status == MixedNozzleSliceTimeStatus::Estimated);
    // The ranking slice leaves the tables as it found them.
    CHECK(Model::printSpeedMap.maxSpeed == 0.);

    REQUIRE(run_wizard_setup(project, choice).applied);
    set_brim_inputs_as_slice_does(project, plate_slice_config(project));
    const PlateSlice applied = slice_plate(project);
    REQUIRE(applied.refusal.empty());
    CHECK(brim_extrusion(applied.moves) > 0.);
    CAPTURE(row_time.seconds, normal_time(applied.statistics));
    CHECK(row_time.seconds == Catch::Approx(normal_time(applied.statistics)).epsilon(1e-4));
}

// Each plate stores its own tower spot (wipe_tower_x / wipe_tower_y by plate index), and the
// plate's own Print reads it by its index. Step 3's slice had no index, so every plate's row was
// timed with plate 1's spot (the tester's plate 2 stores 40, 200 against plate 1's 165, 250).
TEST_CASE("Step 3 times a plate with that plate's own tower spot",
          "[TestRebuild][PlateIsolation][FeatureScene]")
{
    WizardProject project(h2d_02_06_spec());
    project.add_object("plate 1 box", {{"box", Vec3d(30., 30., 6.), Vec3d::Zero(), 1}}, Vec2d(80., 80.));
    PartPlate second(nullptr, Vec3d(350., 0., 0.), 350, 320, 325, nullptr, &project.model);
    second.set_index(1);
    ModelObject *part = project.model.add_object();
    part->name = "plate 2 part";
    part->add_volume(make_cube(85., 85., 16.), ModelVolumeType::MODEL_PART, false)
        ->config.set_key_value("extruder", new ConfigOptionInt(1));
    part->add_instance()->set_offset(Vec3d(350. + 130., 120., 0.));
    REQUIRE(second.add_instance(1, 0, false) == 0);
    second.config()->set_key_value("filament_map_mode", new ConfigOptionEnum<FilamentMapMode>(fmmManual));
    second.config()->set_key_value("filament_map", new ConfigOptionInts{1, 2});
    project.owners.plates.push_back(&second);
    // Plate 1 near the front left corner, plate 2 at the back right.
    project.bundle.project_config.set_key_value("wipe_tower_x", new ConfigOptionFloats{20., 280.});
    project.bundle.project_config.set_key_value("wipe_tower_y", new ConfigOptionFloats{20., 250.});

    SetupChoice choice;
    choice.mode = MixedNozzleSlicingMode::FeatureSplit;
    choice.fine_height = .10;
    choice.ratio = 3;
    choice.scope = WizardScopeKind::ProjectDefault;
    // Step 3 with plate 2 selected.
    WizardSetupStage stage = build_wizard_setup_stage(project, choice);
    stage.draft.scope.current_plate_id = second.id().id;
    WizardNativeOwners owners = project.owners;
    owners.current_plate = &second;
    const MixedNozzleRankingSlice row = wizard_candidate_staged_slice(stage.draft, stage.candidate, stage.body, owners);
    REQUIRE(row.model);
    CHECK(row.plate_index == 1);
    MixedNozzleRankingCancel cancel;
    const MixedNozzleSliceTime row_time = mixed_nozzle_slice_time(*row.model, row.config, row.is_bbl_printer,
                                                                  row.plate_origin, {}, cancel, row.plate_index);
    INFO(row_time.diagnostic);
    REQUIRE(row_time.status == MixedNozzleSliceTimeStatus::Estimated);
    // The plate prints a tower, so where it stands matters.
    REQUIRE(row_time.switches > 0);

    REQUIRE(run_wizard_setup(project, choice).applied);
    // Plate 2 sliced as Slice slices it: its own Print, with its index.
    Model model = project.model;
    for (std::size_t o = 0; o < model.objects.size(); ++o)
        for (std::size_t i = 0; i < model.objects[o]->instances.size(); ++i)
            model.objects[o]->instances[i]->printable = second.contain_instance(int(o), int(i));
    const DynamicPrintConfig config = plate_slice_config(project, second);
    Print print;
    print.set_status_silent();
    print.is_BBL_printer() = true;
    print.set_plate_origin(second.get_origin());
    print.set_plate_index(1);
    print.apply(model, config);
    print.apply(model, config);
    REQUIRE(print.validate().string.empty());
    print.process();
    ScopedTemporaryFile file(".gcode");
    GCodeProcessorResult result;
    print.export_gcode(file.string(), &result, nullptr);
    CAPTURE(row_time.seconds, normal_time(result.print_statistics));
    CHECK(row_time.switches == result.print_statistics.total_physical_tool_changes);
    CHECK(row_time.seconds == Catch::Approx(normal_time(result.print_statistics)).epsilon(1e-4));
}

// A project saved while it printed one material can store a tower spot inside its parts (the
// tester's file, plates 2, 3 and 5 to 8). Setup gives the plate a second tool and so a tower, and it
// stood in the parts: "Conflicts of G-code paths ... (WipeTower <-> ...)" on plate 3. Setup now moves
// such a tower clear of the parts, as the sidebar's Apply does; a clear spot stays where it is, and
// step 3 times the spot Apply stores.
TEST_CASE("Setup moves a prime tower stored inside the parts clear of them, and keeps a clear one",
          "[TestRebuild][PlateIsolation][FeatureScene]")
{
    const bool inside = GENERATE(true, false);
    CAPTURE(inside);
    WizardProject project(h2d_02_06_spec());
    // The bed outline the app gives a plate.
    project.plate.set_shape({Vec2d(0., 0.), Vec2d(350., 0.), Vec2d(350., 320.), Vec2d(0., 320.)}, {}, {}, {},
                            Vec2d::Zero(), 0.f, 0.f);
    // 85 x 85 mm, from 132.5 to 217.5 in X and 117.5 to 202.5 in Y.
    project.add_object("large part", {{"part", Vec3d(85., 85., 16.), Vec3d::Zero(), 1}}, Vec2d(175., 160.));
    const Vec2d stored = inside ? Vec2d(160., 150.) : Vec2d(40., 40.);
    project.bundle.project_config.set_key_value("wipe_tower_x", new ConfigOptionFloats{stored.x()});
    project.bundle.project_config.set_key_value("wipe_tower_y", new ConfigOptionFloats{stored.y()});

    SetupChoice choice;
    choice.mode = MixedNozzleSlicingMode::FeatureSplit;
    choice.fine_height = .10;
    choice.ratio = 3;
    choice.scope = WizardScopeKind::ProjectDefault;
    const WizardSetupStage stage = build_wizard_setup_stage(project, choice);
    const MixedNozzleRankingSlice row = wizard_candidate_staged_slice(stage.draft, stage.candidate, stage.body, project.owners);
    REQUIRE(row.model);
    {
        const WizardSetupStage review_stage = build_wizard_setup_stage(project, choice);
        const PreparedWizardApply prepared = prepare_wizard_apply(review_stage.draft, review_stage.review, project.owners);
        const bool says_so = std::find(prepared.also_changed.begin(), prepared.also_changed.end(),
            "Prime tower: moved clear of the parts on plate 1") != prepared.also_changed.end();
        CHECK(says_so == inside);
    }
    REQUIRE(run_wizard_setup(project, choice).applied);

    const double x = project.bundle.project_config.option<ConfigOptionFloats>("wipe_tower_x")->get_at(0);
    const double y = project.bundle.project_config.option<ConfigOptionFloats>("wipe_tower_y")->get_at(0);
    CAPTURE(x, y);
    if (inside) {
        // Clear of the part's box on at least one side (the tower is at most 35 mm wide).
        CHECK((x >= 217.5 || y >= 202.5 || x + 35. <= 132.5 || y + 35. <= 117.5));
    } else {
        CHECK(x == Catch::Approx(stored.x()));
        CHECK(y == Catch::Approx(stored.y()));
    }
    // Step 3 sliced with the spot Apply stored.
    CHECK(row.config.option<ConfigOptionFloats>("wipe_tower_x")->get_at(0) == Catch::Approx(x));
    CHECK(row.config.option<ConfigOptionFloats>("wipe_tower_y")->get_at(0) == Catch::Approx(y));
}
