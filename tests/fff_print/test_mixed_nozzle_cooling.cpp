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

struct Bracket {
    bool named_feedrate = false;
    double incoming_feedrate = 0.;
    bool first_xy_seen = false;
    bool has_solid_grid = false;
};

struct BracketAudit {
    size_t brackets = 0;
    size_t xy_moves = 0;
    size_t solid_grids = 0;
    size_t equal_first_feedrates = 0;
    std::vector<std::string> moves_before_feedrate;
};

BracketAudit audit_tower_brackets(const std::string &gcode)
{
    BracketAudit audit;
    std::vector<Bracket> stack;
    double modal_feedrate = 0.;
    std::istringstream input(gcode);
    std::string raw;
    while (std::getline(input, raw)) {
        if (raw.rfind("; WIPE_TOWER_START", 0) == 0 ||
            raw.rfind("; NOZZLE_CHANGE_START", 0) == 0) {
            stack.push_back({false, modal_feedrate, false, false});
            ++audit.brackets;
            continue;
        }
        if (raw.rfind("; WIPE_TOWER_END", 0) == 0 ||
            raw.rfind("; NOZZLE_CHANGE_END", 0) == 0) {
            if (!stack.empty()) {
                if (stack.back().has_solid_grid)
                    ++audit.solid_grids;
                stack.pop_back();
            }
            continue;
        }
        if (raw.find("; CP EMPTY GRID START") != std::string::npos && !stack.empty())
            stack.back().has_solid_grid = true;

        const std::string code = raw.substr(0, raw.find(';'));
        if (code.size() < 2 || code[0] != 'G' ||
            (code[1] != '0' && code[1] != '1' && code[1] != '2' && code[1] != '3') ||
            (code.size() > 2 && code[2] != ' '))
            continue;

        bool xy = false;
        bool has_f = false;
        double feedrate = 0.;
        std::istringstream words(code.substr(2));
        std::string word;
        while (words >> word) {
            if (word.empty())
                continue;
            if (word.front() == 'X' || word.front() == 'Y')
                xy = true;
            if (word.front() == 'F') {
                try {
                    feedrate = std::stod(word.substr(1));
                    has_f = true;
                } catch (...) { }
            }
        }

        if (!stack.empty()) {
            Bracket &bracket = stack.back();
            if (xy) {
                ++audit.xy_moves;
                if (!bracket.named_feedrate && !has_f)
                    audit.moves_before_feedrate.push_back(raw);
                if (!bracket.first_xy_seen && has_f &&
                    std::abs(feedrate - bracket.incoming_feedrate) < .01)
                    ++audit.equal_first_feedrates;
                bracket.first_xy_seen = true;
            }
            if (has_f)
                bracket.named_feedrate = true;
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
    REQUIRE_FALSE(facts.gcode.empty());
    REQUIRE_FALSE(facts.emitted.tool_changes.empty());

    const BracketAudit audit = audit_tower_brackets(facts.gcode);
    CAPTURE(audit.brackets, audit.xy_moves, audit.solid_grids,
            audit.equal_first_feedrates, audit.moves_before_feedrate.size());
    for (size_t i = 0; i < audit.moves_before_feedrate.size() && i < 3; ++i)
        INFO(audit.moves_before_feedrate[i]);
    REQUIRE(audit.brackets > 4);
    REQUIRE(audit.xy_moves > 20);
    REQUIRE(audit.solid_grids > 0);
    REQUIRE(audit.equal_first_feedrates > 0);
    CHECK(audit.moves_before_feedrate.empty());
}
