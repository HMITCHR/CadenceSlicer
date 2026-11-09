#include <catch2/catch_all.hpp>

#include "mixed_nozzle_harness.hpp"
#include "mixed_nozzle_fixtures.hpp"

#include <cmath>
#include <sstream>
#include <string>
#include <vector>

using namespace Slic3r;
using namespace mixed_nozzle_fixtures;

namespace {

// Tower blocks (WIPE_TOWER and NOZZLE_CHANGE brackets): the XY moves made before the block names a
// feedrate, and how many blocks hold a solid grid or open on the feedrate already in force.
struct BracketAudit {
    size_t brackets = 0;
    size_t xy_moves = 0;
    size_t solid_grids = 0;
    size_t equal_first_feedrates = 0;
    std::vector<std::string> moves_before_feedrate;
};

BracketAudit audit_tower_brackets(const std::string &gcode)
{
    struct Bracket {
        double incoming = 0.;
        bool   named = false, first_xy_seen = false, solid_grid = false;
    };
    BracketAudit audit;
    std::vector<Bracket> stack;
    double modal_feedrate = 0.;
    std::istringstream input(gcode);
    std::string raw;
    while (std::getline(input, raw)) {
        if (raw.rfind("; WIPE_TOWER_START", 0) == 0 || raw.rfind("; NOZZLE_CHANGE_START", 0) == 0) {
            stack.push_back({modal_feedrate});
            ++audit.brackets;
            continue;
        }
        if (raw.rfind("; WIPE_TOWER_END", 0) == 0 || raw.rfind("; NOZZLE_CHANGE_END", 0) == 0) {
            if (!stack.empty()) {
                audit.solid_grids += stack.back().solid_grid ? 1 : 0;
                stack.pop_back();
            }
            continue;
        }
        if (raw.find("; CP EMPTY GRID START") != std::string::npos && !stack.empty())
            stack.back().solid_grid = true;

        const std::string code = raw.substr(0, raw.find(';'));
        if (code.size() < 3 || code[0] != 'G' || code[1] < '0' || code[1] > '3' || code[2] != ' ')
            continue;
        bool xy = false, has_f = false;
        double feedrate = 0.;
        std::istringstream words(code.substr(2));
        for (std::string word; words >> word;) {
            xy = xy || word.front() == 'X' || word.front() == 'Y';
            if (word.front() == 'F' && word.size() > 1) {
                feedrate = std::stod(word.substr(1));
                has_f = true;
            }
        }
        if (!stack.empty()) {
            Bracket &bracket = stack.back();
            if (xy) {
                ++audit.xy_moves;
                if (!bracket.named && !has_f)
                    audit.moves_before_feedrate.push_back(raw);
                if (!bracket.first_xy_seen && has_f && std::abs(feedrate - bracket.incoming) < .01)
                    ++audit.equal_first_feedrates;
                bracket.first_xy_seen = true;
            }
            bracket.named = bracket.named || has_f;
        }
        if (has_f)
            modal_feedrate = feedrate;
    }
    return audit;
}

} // namespace

TEST_CASE("Mixed tower blocks name a feedrate before moving the nozzle",
          "[TestRebuild][CoolingBuffer][Tower]")
{
    CadenceTest::Scene scene;
    scene.config = feature_split_tower_config(.20, .60, false);
    scene.config.set_key_value("wipe_tower_wall_type", new ConfigOptionEnum<WipeTowerWallType>(wtwRib));
    scene.config.set_key_value("filament_ramming_volumetric_speed", new ConfigOptionFloatsNullable{0., 0.});
    scene.config.set_key_value("filament_ramming_volumetric_speed_nc", new ConfigOptionFloatsNullable{0., 0.});
    scene.populate = [](Model &model, Print &print, const DynamicPrintConfig &config) {
        init_tower_coupon(model, print, config, make_cube(60., 60., 4.));
    };

    const CadenceTest::Facts facts = CadenceTest::slice(scene);
    INFO(facts.refusal.string);
    REQUIRE(facts.refusal.string.empty());
    REQUIRE_FALSE(facts.emitted.tool_changes.empty());

    const BracketAudit audit = audit_tower_brackets(facts.gcode);
    CAPTURE(audit.brackets, audit.xy_moves, audit.solid_grids, audit.equal_first_feedrates);
    INFO((audit.moves_before_feedrate.empty() ? std::string() : audit.moves_before_feedrate.front()));
    REQUIRE(audit.brackets > 4);
    REQUIRE(audit.xy_moves > 20);
    REQUIRE(audit.solid_grids > 0);
    REQUIRE(audit.equal_first_feedrates > 0);
    CHECK(audit.moves_before_feedrate.empty());
}
