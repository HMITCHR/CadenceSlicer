#include <catch2/catch_all.hpp>

#include "mixed_nozzle_harness.hpp"
#include "mixed_nozzle_fixtures.hpp"

#include <algorithm>
#include <array>

using namespace Slic3r;
using namespace mixed_nozzle_fixtures;

namespace {

struct PadRun {
    CadenceTest::Facts facts;
    PrintConfig config;
    double nominal_width {0.};
    double generated_footprint_width {0.};
    double emitted_width {0.};
};

PadRun reprime_pad_run(double wall_speed, double width_cap)
{
    CadenceTest::Scene scene = CadenceTest::body_coupon();
    scene.config.set_key_value("prime_tower_width", new ConfigOptionFloat(width_cap));
    if (wall_speed > 0.) {
        scene.config.set_key_value("outer_wall_speed", new ConfigOptionFloatsNullable{wall_speed, wall_speed});
        scene.config.set_key_value("inner_wall_speed", new ConfigOptionFloatsNullable{wall_speed, wall_speed});
    }
    mixed_nozzle_fixtures::fill_per_filament_values(scene.config);
    if (scene.bambu && scene.config.opt_string("machine_start_gcode").find("M1020") == std::string::npos)
        scene.config.set_key_value("machine_start_gcode", new ConfigOptionString(
            scene.config.opt_string("machine_start_gcode") + "\nM1020 S[initial_extruder]\n"));
    Model model;
    Print print;
    print.is_BBL_printer() = scene.bambu;
    scene.populate(model, print, scene.config);
    print.set_status_silent();
    const CadenceTest::Facts facts = CadenceTest::slice(print);
    INFO(facts.refusal.string);
    REQUIRE(facts.refusal.string.empty());
    REQUIRE_FALSE(facts.gcode.empty());
    REQUIRE(print.has_wipe_tower());
    const WipeTowerData &tower = print.wipe_tower_data();
    REQUIRE(tower.width > 0.f);
    BoundingBoxf bounds;
    for (const auto &move : facts.moves)
        if (move.type == EMoveType::Extrude && move.extrusion_role == erWipeTower)
            bounds.merge(Vec2d(move.position.x(), move.position.y()));
    REQUIRE(bounds.defined);
    const double emitted_width = std::max(bounds.size().x(), bounds.size().y());
    REQUIRE(emitted_width > 0.);
    const double generated_footprint_width = std::max(
        double(tower.bbx.max.x() - tower.bbx.min.x()),
        double(tower.bbx.max.y() - tower.bbx.min.y()));
    REQUIRE(generated_footprint_width > 0.);
    // The generator's bbx includes its rib and brim. Check it against the independently measured
    // exported road bounds so this scenario reports the actual printed footprint as well as the
    // nominal width that prime_tower_width caps.
    CHECK_THAT(emitted_width, Catch::Matchers::WithinAbs(generated_footprint_width, .5));
    const PrintConfig config = print.config();
    return {facts, config, double(tower.width), generated_footprint_width, emitted_width};
}

void check_tower_safety(const PadRun &run)
{
    std::array<size_t, 2> roads{};
    for (const auto &fact : run.facts.emitted.role_facts) {
        if (fact.role != erWipeTower)
            continue;
        REQUIRE(fact.physical_tool < roads.size());
        const size_t tool = fact.physical_tool;
        ++roads[tool];
        const double nozzle = run.config.nozzle_diameter.get_at(tool);
        CHECK(fact.height >= run.config.min_layer_height.get_at(tool) - .001);
        CHECK(fact.height <= run.config.max_layer_height.get_at(tool) + .001);
        CHECK(fact.width > 0.f);
        CHECK(fact.width <= 1.6 * nozzle + .01);
        REQUIRE(fact.logical_filament < run.config.filament_map.values.size());
        CHECK(run.config.filament_map.values[fact.logical_filament] == int(tool + 1));
    }
    CHECK(roads[0] > 0);
    CHECK(roads[1] > 0);
    const TowerExitAudit exit = audit_tower_exit(run.facts.gcode, .7);
    CHECK(exit.returns > 0);
    CHECK(exit.unbalanced == 0);
    CHECK(exit.moves_off_rows == 0);
    CHECK(exit.returns_without_wipe == 0);
    CHECK(exit.not_lifted_next == 0);
    CHECK(exit.crossings == 0);
}

} // namespace

TEST_CASE("Reprime: Body Split auto pad converges between short and full re-prime footprints",
          "[TestRebuild][BodySplit][Reprime]")
{
    const PadRun fast = reprime_pad_run(-1., 60.);
    const PadRun slow = reprime_pad_run(2., 60.);
    CAPTURE(fast.nominal_width, slow.nominal_width, fast.emitted_width, slow.emitted_width);
    REQUIRE(slow.nominal_width > fast.nominal_width + 1.);

    // prime_tower_width is the nominal rectangle cap. Exported roads show the whole user-visible
    // print footprint, which also contains the rib and brim outside that rectangle.
    const double cap = .5 * (fast.nominal_width + slow.nominal_width);
    const PadRun tight = reprime_pad_run(2., cap);
    CAPTURE(cap, tight.nominal_width, tight.generated_footprint_width, tight.emitted_width);
    CHECK(tight.nominal_width <= cap + .05);
    CHECK(tight.emitted_width < slow.emitted_width - 1.);
    check_tower_safety(tight);
    // Every warm return lays at least its hand-off prime at the capped tower width.
    CHECK(check_warm_returns(tight.facts.gcode, tight.config, tight.nominal_width, false).count > 0);
}
