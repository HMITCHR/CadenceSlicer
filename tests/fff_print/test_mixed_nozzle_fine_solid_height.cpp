#include <catch2/catch_all.hpp>
#include "mixed_nozzle_harness.hpp"

#include <map>
#include <sstream>

using namespace Slic3r;
using namespace CadenceTest;
using namespace mixed_nozzle_fixtures;

// A Feature Split object with a part (or the whole object) at 100% sparse density, as in a tester's
// 0.4 / 0.8 project where some parts were set to 100% and the rest to 15%. A 100% region has only
// internal solid infill, which is the fine tool's. It must stay at the fine layer height: the fine
// nozzle may never lay anything thicker than its maximum layer height, and coarse-height bands are
// for the coarse tool's sparse infill only.
TEST_CASE("Feature Split keeps a 100% density part's solid infill at the fine layer height",
          "[TestRebuild][FeatureScene]")
{
    // 0: one 100% part next to a 15% part. 1: the whole object at 100%.
    const int variant = GENERATE(0, 1);
    // 0: the fixture's 0.2 / 0.6 pair (fine 0.12, coarse 0.36). 1: a 0.4 / 0.8 pair at fine 0.20 and
    // coarse 0.40, the tester's case, where the fine nozzle can still lay 0.40 without a flow error.
    const int pair = GENERATE(0, 1);
    CAPTURE(variant, pair);
    Scene scene = feature_cube();
    if (pair == 1) {
        scene.config.set_key_value("nozzle_diameter", new ConfigOptionFloats{0.4, 0.8});
        scene.config.set_key_value("min_layer_height", new ConfigOptionFloats{0.08, 0.16});
        scene.config.set_key_value("max_layer_height", new ConfigOptionFloats{0.28, 0.56});
        scene.config.set_key_value("layer_height", new ConfigOptionFloat(0.20));
        scene.config.set_key_value("initial_layer_print_height", new ConfigOptionFloat(0.20));
        scene.config.set_key_value("mixed_nozzle_coarse_layer_height", new ConfigOptionFloat(0.40));
        scene.config.set_key_value("mixed_nozzle_allowed_cadence_ratios", new ConfigOptionInts{2});
    }
    scene.populate = [variant](Model &model, Print &print, const DynamicPrintConfig &config) {
        ModelObject *object = model.add_object();
        object->name = "solid-part-fixture";
        // A sparse part big enough that its coarse bands pay for the tower, so the whole-print time
        // check keeps them, beside a small 100% part.
        ModelVolume *sparse = object->add_volume(make_cube(80., 80., 6.), ModelVolumeType::MODEL_PART, false);
        ModelVolume *solid  = object->add_volume(make_cube(20., 80., 6.), ModelVolumeType::MODEL_PART, false);
        solid->set_offset(Vec3d(90., 0., 0.));
        solid->config.set_key_value("sparse_infill_density", new ConfigOptionPercent(100.));
        if (variant == 1)
            sparse->config.set_key_value("sparse_infill_density", new ConfigOptionPercent(100.));
        object->add_instance();
        object->ensure_on_bed();
        object->instances.front()->set_offset(Vec3d(30., 40., 0.));
        print.apply(model, config);
        print.set_status_silent();
    };
    const DynamicPrintConfig config = scene.config;
    const Facts facts = slice(scene);
    INFO(facts.refusal.string);
    REQUIRE(facts.refusal.string.empty());
    REQUIRE_FALSE(facts.moves.empty());

    const double fine_max = config.option<ConfigOptionFloats>("max_layer_height")->get_at(0);
    const double fine_layer = config.opt_float("layer_height");
    size_t too_thick = 0, thick_solid = 0, solid_moves = 0, coarse_sparse = 0, coarse_other = 0;
    std::string first;
    for (const auto &move : facts.moves) {
        if (move.type != EMoveType::Extrude || move.extrusion_role == erWipeTower ||
            move.extrusion_role == erCustom)
            continue;
        const bool fine_tool = move.physical_tool_id == 0;
        if (move.extrusion_role == erSolidInfill) {
            ++solid_moves;
            if (move.height > fine_layer + 0.005)
                ++thick_solid;
        }
        if (move.physical_tool_id == 1) {
            if (move.extrusion_role == erInternalInfill)
                ++coarse_sparse;
            else
                ++coarse_other;
        }
        if (fine_tool && move.height > fine_max + 0.005) {
            if (first.empty()) {
                std::ostringstream detail;
                detail << "role=" << ExtrusionEntity::role_to_string(move.extrusion_role) << " z=" << move.position.z()
                       << " height=" << move.height << " width=" << move.width;
                first = detail.str();
            }
            ++too_thick;
        }
    }
    CAPTURE(first, solid_moves, thick_solid, coarse_sparse, coarse_other);
    CHECK(solid_moves > 0);
    CHECK(too_thick == 0);
    CHECK(thick_solid == 0);
    // Only sparse infill goes to the coarse tool. With a sparse part there is some; with none, none.
    CHECK(coarse_other == 0);
    if (variant == 0)
        CHECK(coarse_sparse > 0);
    else
        CHECK(coarse_sparse == 0);
}

// The whole-print time check sets the agreed plan against printing every band fine. A 100% part has
// no sparse infill, so it never asks for the coarse tool. When it sat beside a 15% part, the check
// still read it as busy on the coarse tool at every height, kept every switch in the all-fine plan
// and priced it with the plan's own tower. All fine then never looked cheaper, and a print slower
// on two nozzles than on one kept its coarse bands (the tester's 0.4 / 0.8 plate).
TEST_CASE("Feature Split prices all fine without the coarse tool when a 100% part sits beside a sparse part",
          "[TestRebuild][FeatureScene]")
{
    Scene scene = feature_cube();
    // 0: a 100% part beside the 15% part. 1: the 15% part alone, the reference.
    const int variant = GENERATE(0, 1);
    CAPTURE(variant);
    mixed_nozzle_fixtures::fill_per_filament_values(scene.config);
    Model model;
    Print print;
    print.is_BBL_printer() = true;
    ModelObject *object = model.add_object();
    object->name = "solid-beside-sparse";
    object->add_volume(make_cube(30., 60., 3.), ModelVolumeType::MODEL_PART, false);
    if (variant == 0) {
        ModelVolume *solid = object->add_volume(make_cube(30., 60., 3.), ModelVolumeType::MODEL_PART, false);
        solid->set_offset(Vec3d(40., 0., 0.));
        solid->config.set_key_value("sparse_infill_density", new ConfigOptionPercent(100.));
    }
    object->add_instance();
    object->ensure_on_bed();
    object->instances.front()->set_offset(Vec3d(30., 40., 0.));
    print.apply(model, scene.config);
    print.set_status_silent();
    REQUIRE(print.validate().string.empty());
    print.process();
    std::map<std::string, double> digest;
    {
        std::istringstream lines(print.feature_economics_whole_print_digest());
        std::string line;
        std::getline(lines, line);
        std::istringstream items(line);
        for (std::string item; items >> item; )
            if (const size_t equals = item.find('='); equals != std::string::npos && item.rfind("decision", 0) != 0)
                digest[item.substr(0, equals)] = std::stod(item.substr(equals + 1));
    }
    INFO(print.feature_economics_whole_print_digest());
    // The 15% part keeps coarse bands with free switches, so the whole-print check runs.
    REQUIRE(digest.count("plan_tower") == 1);
    REQUIRE(digest["plan_tower"] > 0.);
    // All fine has no coarse work left, so no switch past the first load and no tower.
    CHECK(digest["all_fine_tower"] == 0.);
    CHECK(digest["all_fine_tower"] < digest["plan_tower"]);
    CHECK(digest["all_fine_switches"] <= digest["plan_switches"]);
}
