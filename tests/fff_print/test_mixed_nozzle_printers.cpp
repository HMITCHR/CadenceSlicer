#include <catch2/catch_all.hpp>
#include "mixed_nozzle_printers.hpp"
#include <set>
#include <sstream>

using namespace Slic3r;
using namespace CadenceTest;

TEST_CASE("A non-Bambu printer uses its own tool changes and temperatures with mixed nozzle deposition", "[TestRebuild][PrinterScene]")
{
    const bool body = GENERATE(false, true);
    const bool ratrig = GENERATE(false, true);
    Scene scene;
    scene.bambu = false;
    if (ratrig) {
        scene.config = compose_profile({"Ratrig", "RatRig V-Core 4 IDEX 300 0.4 nozzle",
            "0.20mm Quality @RatRig V-Core 4 IDEX 0.4", {"RatRig Generic PLA", "RatRig Generic PLA"}});
        install_nozzles(scene.config, {{0, .4}, {1, .8}}, {{0, {.06, .30}}, {1, {.12, .50}}});
        identity_filament_map(scene.config);
        apply_other_printer_setup(scene.config, Vec2d(240., 150.));
    } else {
        scene.config = j1_config();
    }
    scene.config.set_key_value("filament_start_gcode", new ConfigOptionStrings(2,
        "; material start layer={layer_num} z={layer_z}\n"));
    const double fine = .08;
    const double coarse = ratrig ? .48 : .40;
    const int ratio = ratrig ? 6 : 5;
    const std::vector<double> pair = ratrig ? std::vector<double>{.4, .8} : std::vector<double>{.2, .6};
    scene.config.option<ConfigOptionInts>("nozzle_temperature", true)->values = {220, 250};
    scene.config.option<ConfigOptionInts>("nozzle_temperature_initial_layer", true)->values = {220, 250};
    if (body)
        body_split_heights(scene.config, fine, coarse, ratio, pair);
    else
        feature_split_heights(scene.config, fine, coarse, ratio, 1, 2, pair);
    if (!body)
        scene.config.set_key_value("sparse_infill_density", new ConfigOptionPercent(40.));
    scene.populate = [body, coarse](Model &model, Print &print, const DynamicPrintConfig &config) {
        if (body)
            load_body_pair(model, print, config, Vec2d(145., 100.), 1, 2, coarse);
        else
            load_feature_box(model, print, config, Vec2d(145., 100.));
    };
    const Facts facts = slice(scene);
    INFO(facts.refusal.string);
    REQUIRE(facts.refusal.string.empty());
    REQUIRE_FALSE(facts.gcode.empty());
    CHECK(facts.seconds > 0.);
    const GcodeFacts commands = scan_gcode(facts.gcode, 1);
    CAPTURE(body, ratrig);
    std::istringstream diagnostics(facts.gcode);
    std::string record;
    std::string schedule;
    while (std::getline(diagnostics, record))
        if (record.find("SRL_FEATURE") != std::string::npos)
            schedule += record + "\n";
    INFO(schedule);
    if (ratrig && !body) {
        CHECK(commands.changes.empty());
        size_t roads = 0;
        for (const auto &move : facts.moves)
            if (model_road(move)) {
                ++roads;
                REQUIRE(move.physical_tool_id == 0);
                CHECK(move.height <= fine + .001);
            }
        CHECK(roads > 0);
        return;
    }
    std::regex material_start(R"(; material start layer=([0-9]+) z=([0-9.]+))");
    size_t started_above_bed = 0;
    for (auto it = std::sregex_iterator(facts.gcode.begin(), facts.gcode.end(), material_start);
         it != std::sregex_iterator(); ++it) {
        if (std::stoi((*it)[1]) > 0 && std::stod((*it)[2]) > 0.)
            ++started_above_bed;
    }
    CHECK(started_above_bed > 0);
    REQUIRE_FALSE(commands.changes.empty());
    for (const auto &change : commands.changes) {
        CHECK(change.inside_tower_block);
        CHECK(change.purged_mm > 0.);
    }
    CHECK(commands.bambu_only.empty());
    CHECK_FALSE(facts.emitted.plan_malformed);
    std::set<unsigned> model_tools, tower_tools;
    size_t thick = 0;
    for (const auto &move : facts.moves) {
        if (move.type != EMoveType::Extrude)
            continue;
        if (model_road(move)) {
            model_tools.insert(move.physical_tool_id);
            if (move.physical_tool_id == 1 && move.height > fine + .001)
                ++thick;
        }
        if (move.extrusion_role == erWipeTower)
            tower_tools.insert(move.physical_tool_id);
    }
    CHECK(model_tools == std::set<unsigned>{0, 1});
    CHECK(commands.tools_selected == std::set<int>{0, 1});
    CHECK(thick > 0);
    REQUIRE_FALSE(commands.temperature_lines.empty());
    for (const std::string &line : commands.temperature_lines) {
        std::smatch match;
        REQUIRE(std::regex_search(line, match, std::regex(R"(^M104 T(\d+) S(\d+) N0)")));
        const int tool = std::stoi(match[1]);
        REQUIRE(tool < 2);
        CHECK(std::stoi(match[2]) == (tool == 0 ? 220 : 250));
    }
}
