#include <catch2/catch_all.hpp>

#include "rebuild_harness.hpp"
#include "srl_fixtures.hpp"
#include "libslic3r/MixedNozzleConfig.hpp"

#include <algorithm>
#include <array>
#include <limits>

using namespace Slic3r;
using namespace srl_fixtures;

namespace {

struct TowerBounds {
    double min_x {std::numeric_limits<double>::max()}, min_y {std::numeric_limits<double>::max()};
    double max_x {std::numeric_limits<double>::lowest()}, max_y {std::numeric_limits<double>::lowest()};
    size_t roads {0};
    void add(const GCodeProcessorResult::MoveVertex &move) {
        min_x = std::min(min_x, double(move.position.x()));
        min_y = std::min(min_y, double(move.position.y()));
        max_x = std::max(max_x, double(move.position.x()));
        max_y = std::max(max_y, double(move.position.y()));
        ++roads;
    }
    double width() const { return std::max(max_x - min_x, max_y - min_y); }
};

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
    srl_fixtures::fill_per_filament_values(scene.config);
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
    TowerBounds bounds;
    for (const auto &move : facts.moves)
        if (move.type == EMoveType::Extrude && move.extrusion_role == erWipeTower)
            bounds.add(move);
    REQUIRE(bounds.roads > 0);
    REQUIRE(bounds.width() > 0.);
    const double generated_footprint_width = std::max(
        double(tower.bbx.max.x() - tower.bbx.min.x()),
        double(tower.bbx.max.y() - tower.bbx.min.y()));
    REQUIRE(generated_footprint_width > 0.);
    // The generator's bbx includes its rib and brim. Check it against the independently measured
    // exported road bounds so this scenario reports the actual printed footprint as well as the
    // nominal width that prime_tower_width caps.
    CHECK_THAT(bounds.width(), Catch::Matchers::WithinAbs(generated_footprint_width, .5));
    const PrintConfig config = print.config();
    return {facts, config, double(tower.width), generated_footprint_width, bounds.width()};
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

void check_warm_return_floors(const PadRun &run)
{
    constexpr double rounding_mm3 = .02;
    const auto purges = scan_v24_arriving_purges(run.facts.gcode, run.config.filament_diameter.values);
    size_t warm_returns = 0;
    for (const auto &purge : purges) {
        if (!purge.warm_return)
            continue;
        ++warm_returns;
        REQUIRE(purge.new_filament >= 0);
        REQUIRE(purge.new_nozzle >= 0);
        const auto floor = mixed_nozzle_handoff_deposits(run.config, size_t(purge.new_filament),
                                                          size_t(purge.new_nozzle), run.nominal_width);
        REQUIRE(floor.has_value());
        CAPTURE(purge.new_filament, purge.new_nozzle, purge.volume_mm3, floor->prime_volume_mm3);
        CHECK(purge.volume_mm3 + rounding_mm3 >= floor->prime_volume_mm3);
    }
    CHECK(warm_returns > 0);
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
    check_warm_return_floors(tight);
}
