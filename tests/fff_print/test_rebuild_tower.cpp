#include <catch2/catch_all.hpp>

#include "rebuild_harness.hpp"
#include "srl_fixtures.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <cstdio>
#include <sstream>
#include <vector>

using namespace Slic3r;
using namespace srl_fixtures;

namespace {

struct TowerBounds {
    double min_x = std::numeric_limits<double>::max();
    double min_y = std::numeric_limits<double>::max();
    double max_x = std::numeric_limits<double>::lowest();
    double max_y = std::numeric_limits<double>::lowest();
    size_t roads = 0;

    void add(const GCodeProcessorResult::MoveVertex &move) {
        min_x = std::min(min_x, double(move.position.x()));
        min_y = std::min(min_y, double(move.position.y()));
        max_x = std::max(max_x, double(move.position.x()));
        max_y = std::max(max_y, double(move.position.y()));
        ++roads;
    }
    double width() const { return max_x - min_x; }
    double depth() const { return max_y - min_y; }
};

struct TowerObservation {
    double top = 0.;
    TowerBounds first;
    TowerBounds next;
};

TowerObservation print_tower(double height, bool auto_brim)
{
    CadenceTest::Scene scene;
    scene.config = v24_prime_tower_config(.20, .60, false);
    scene.config.set_key_value("wipe_tower_wall_type", new ConfigOptionEnum<WipeTowerWallType>(wtwRib));
    scene.config.set_key_value("mixed_nozzle_auto_pad", new ConfigOptionBool(true));
    scene.config.set_key_value("prime_tower_width", new ConfigOptionFloat(60.));
    scene.config.set_key_value("prime_tower_brim_width", new ConfigOptionFloat(auto_brim ? -1. : 0.));
    scene.config.set_key_value("initial_layer_speed", new ConfigOptionFloats{40., 50.});
    scene.config.set_key_value("first_layer_flow_ratio", new ConfigOptionFloat(1.));
    scene.config.set_key_value("enable_arc_fitting", new ConfigOptionBool(true));
    scene.config.set_key_value("sparse_infill_pattern", new ConfigOptionEnum<InfillPattern>(ipGyroid));
    scene.config.set_key_value("change_filament_gcode", new ConfigOptionString(
        scene.config.opt_string("change_filament_gcode") + "\n;VG1 E4 F748\n;VG1 E4 F748\n;VG1 E4 F748\n"));
    // Explicit ram lengths near the automatic total, so the plan stays profitable, but far enough
    // from each automatic dose (about 1 and 5.6 mm3) that the dose check tells them apart.
    scene.config.set_key_value("filament_change_length", new ConfigOptionFloats{1.5, 1.2});
    scene.config.set_key_value("filament_change_length_nc", new ConfigOptionFloats{1.5, 1.2});
    scene.config.set_key_value("mixed_nozzle_filament_explicit_keys",
        new ConfigOptionStrings{"filament_change_length", "filament_change_length"});
    apply_tower_exit_retraction(scene.config);
    fill_per_filament_values(scene.config);
    scene.populate = [height](Model &model, Print &print, const DynamicPrintConfig &config) {
        init_feature_flow_fixture(model, print, config, height, 60.);
    };
    const CadenceTest::Facts facts = CadenceTest::slice(scene);
    INFO(facts.refusal.string);
    REQUIRE(facts.refusal.string.empty());
    REQUIRE_FALSE(facts.gcode.empty());
    CHECK_FALSE(facts.emitted.plan_malformed);
    REQUIRE_FALSE(facts.emitted.tool_changes.empty());

    // GCodeProcessor's gcode_id is the exported one-based line number. The
    // nozzle-change bracket contains outgoing ramming, whose physical width
    // is deliberately wider than a regular tower road (0.2 -> 0.5 mm here).
    std::vector<bool> ramming_line(1, false);
    std::vector<size_t> ramming_block(1, 0);
    size_t block_count = 0;
    {
        std::istringstream lines(facts.gcode);
        std::string line;
        bool ramming = false;
        while (std::getline(lines, line)) {
            if (line.rfind("; NOZZLE_CHANGE_START", 0) == 0) {
                ramming = true;
                ++block_count;
            }
            ramming_line.push_back(ramming);
            ramming_block.push_back(ramming ? block_count : 0);
            if (line.rfind("; NOZZLE_CHANGE_END", 0) == 0)
                ramming = false;
        }
    }

    struct RamDose { double volume = 0., largest_road = 0.; unsigned tool = 0; };
    std::vector<RamDose> doses(block_count + 1);
    double first_z = std::numeric_limits<double>::max();
    double next_z = std::numeric_limits<double>::max();
    TowerObservation out;
    std::array<size_t, 2> tool_roads{};
    size_t bad_flow = 0, bad_envelope = 0, first_at_adhesion_speed = 0;
    size_t ramming_roads = 0, ordinary_roads = 0;
    std::string first_bad;
    for (const auto &move : facts.moves) {
        if (move.type != EMoveType::Extrude || move.extrusion_role != erWipeTower)
            continue;
        const double z = move.position.z();
        if (z < first_z - 1e-4) {
            next_z = first_z;
            first_z = z;
        } else if (z > first_z + 1e-4)
            next_z = std::min(next_z, z);
    }
    REQUIRE(first_z < std::numeric_limits<double>::max());
    REQUIRE(next_z < std::numeric_limits<double>::max());
    for (const auto &move : facts.moves) {
        if (move.type != EMoveType::Extrude || move.extrusion_role != erWipeTower)
            continue;
        const double z = move.position.z();
        out.top = std::max(out.top, z);
        bad_flow += move.mm3_per_mm <= 0.f ? 1 : 0;
        REQUIRE(move.physical_tool_id < 2);
        const unsigned tool = move.physical_tool_id;
        ++tool_roads[tool];
        const double nozzle = tool == 0 ? .20 : .60;
        const double max_h = tool == 0 ? .14 : .40;
        REQUIRE(move.gcode_id < ramming_line.size());
        const bool ramming = ramming_line[move.gcode_id];
        ramming_roads += ramming;
        ordinary_roads += !ramming;
        if (ramming) {
            RamDose &dose = doses[ramming_block[move.gcode_id]];
            const double diameter = scene.config.option<ConfigOptionFloats>("filament_diameter")->get_at(move.extruder_id);
            const double volume = move.delta_extruder * M_PI * diameter * diameter / 4.;
            dose.volume += volume;
            dose.largest_road = std::max(dose.largest_road, volume);
            dose.tool = tool;
        }
        const double width_limit = ramming ? (tool == 0 ? .50 : 1.20) : 1.6 * nozzle;
        const bool outside = move.height <= 0.f || move.height > max_h + 1e-3 ||
                             move.width <= 0.f || move.width > width_limit + .01;
        bad_envelope += outside;
        if (outside && first_bad.empty()) {
            std::ostringstream diagnostic;
            diagnostic << "tool=" << tool << " z=" << z << " height=" << move.height
                       << " width=" << move.width << " ramming=" << ramming
                       << " flow=" << move.mm3_per_mm
                       << " gcode_id=" << move.gcode_id;
            first_bad = diagnostic.str();
        }
        if (std::abs(z - first_z) < 1e-4) {
            out.first.add(move);
            first_at_adhesion_speed += std::abs(move.feedrate - 15.f) < .1f ? 1 : 0;
        } else if (std::abs(z - next_z) < 1e-4)
            out.next.add(move);
    }
    CAPTURE(height, first_z, next_z, out.top, tool_roads[0], tool_roads[1],
            first_at_adhesion_speed, ramming_roads, ordinary_roads, bad_flow, bad_envelope);
    INFO(first_bad);
    REQUIRE(out.first.roads > 0);
    REQUIRE(out.next.roads > 0);
    CHECK(tool_roads[0] > 0);
    CHECK(tool_roads[1] > 0);
    CHECK(first_at_adhesion_speed > 0);
    CHECK(ramming_roads > 0);
    CHECK(ordinary_roads > 0);
    CHECK(bad_flow == 0);
    CHECK(bad_envelope == 0);
    std::array<size_t, 2> rammed_tools{};
    for (const RamDose &dose : doses) {
        if (dose.volume == 0.)
            continue;
        ++rammed_tools[dose.tool];
        const double diameter = scene.config.option<ConfigOptionFloats>("filament_diameter")->get_at(dose.tool);
        const double length = scene.config.option<ConfigOptionFloats>("filament_change_length")->get_at(dose.tool);
        const double target = length * M_PI * diameter * diameter / 4.;
        CAPTURE(dose.tool, dose.volume, target, dose.largest_road);
        // The tower rounds the configured dose to complete extrusion roads.
        CHECK(std::abs(dose.volume - target) <= dose.largest_road + .05);
    }
    CHECK(rammed_tools[0] > 0);
    CHECK(rammed_tools[1] > 0);

    if (height < 10.) {
        std::istringstream lines(facts.gcode);
        std::string line;
        size_t virtual_moves = 0, arcs = 0;
        int final_progress = -1, last_progress = -1;
        while (std::getline(lines, line)) {
            virtual_moves += line.rfind(";VG1", 0) == 0;
            arcs += line.rfind("G2 ", 0) == 0 || line.rfind("G3 ", 0) == 0;
            int percent = -1, minutes = -1;
            if (std::sscanf(line.c_str(), "M73 P%d R%d", &percent, &minutes) == 2) {
                if (percent == 100 && minutes == 0) final_progress = percent;
                else last_progress = percent;
            }
        }
        CAPTURE(virtual_moves, arcs, last_progress);
        REQUIRE(virtual_moves > 0);
        REQUIRE(arcs > 0);
        CHECK(final_progress == 100);
        CHECK(last_progress >= 95);
        const TowerExitAudit exit = audit_tower_exit(facts.gcode, .7);
        CHECK(exit.returns > 0);
        CHECK(exit.unbalanced == 0);
        CHECK(exit.returns_without_wipe == 0);
        CHECK(exit.crossings == 0);
    }
    return out;
}

} // namespace

TEST_CASE("Mixed-nozzle tower exports adhesion, tiered brim, and a tall pad floor",
          "[TestRebuild][TowerDurability]")
{
    const TowerObservation short_tower = print_tower(4., true);
    const TowerObservation short_without_brim = print_tower(4., false);
    const TowerObservation middle_tower = print_tower(34., true);
    const TowerObservation middle_without_brim = print_tower(34., false);
    const TowerObservation tall_tower = print_tower(60.4, false);
    const double short_extension = .5 * std::min(
        short_tower.first.width() - short_without_brim.first.width(),
        short_tower.first.depth() - short_without_brim.first.depth());
    const double middle_extension = .5 * std::min(
        middle_tower.first.width() - middle_without_brim.first.width(),
        middle_tower.first.depth() - middle_without_brim.first.depth());
    CAPTURE(short_tower.top, middle_tower.top, tall_tower.top,
            short_extension, middle_extension,
            tall_tower.first.width(), tall_tower.first.depth());
    CHECK(short_tower.top < 20.);
    CHECK(middle_tower.top >= 20.);
    CHECK(middle_tower.top < 60.);
    CHECK(tall_tower.top >= 60.);
    // Compare matching tower heights: the next-level footprint can change
    // independently of brim. Auto uses 3 and 5 mm radial tiers, rounded to
    // whole bead loops by the actual tower writer.
    CHECK(std::abs(short_extension - 3.) < .8);
    CHECK(std::abs(middle_extension - 5.) < .8);
    CHECK(middle_extension > short_extension + 1.2);
    CHECK(tall_tower.first.width() > 29.);
    CHECK(tall_tower.first.depth() > 29.);
}
