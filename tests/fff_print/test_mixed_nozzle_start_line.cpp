#include <catch2/catch_all.hpp>

#include "mixed_nozzle_harness.hpp"

#include <algorithm>
#include <optional>
#include <sstream>
#include <string>

using namespace Slic3r;
using namespace CadenceTest;

namespace {
// An H2D-style load line: the start G-code ends at the front right, primed and 1.2 mm up.
const char *k_start_gcode = "G28\nG1 Z5 F1200\nG1 X250 Y1 F30000\nG1 Z0.8\nG1 X290 E10 F300\nG1 Z1.2\n"
                            "; LOAD_LINE_END\nM1020 S[initial_extruder]\n";
constexpr double k_start_z = 1.2;

struct Transition {
    std::string text;
    bool   retracted_before_travel = false;
    bool   first_z_before_retract  = false;
    double lowest_z_before_travel  = k_start_z;
    bool   travelled               = false;
    double first_z_after_travel    = -1.;
};

std::optional<double> word(const std::string &line, char letter)
{
    std::istringstream in(line);
    std::string token;
    while (in >> token) {
        if (token[0] == ';')
            break;
        if (token.size() > 1 && token[0] == letter)
            return std::stod(token.substr(1));
    }
    return std::nullopt;
}

// Reads the slicer's moves from the end of the start G-code to the first Z move after the first XY travel.
Transition first_transition(const std::string &gcode)
{
    Transition t;
    const size_t start = gcode.find("\n; LOAD_LINE_END\n");
    REQUIRE(start != std::string::npos);
    std::istringstream in(gcode.substr(start + 1));
    std::string line;
    bool seen_z = false;
    while (std::getline(in, line)) {
        t.text += line + "\n";
        if (line.rfind("G1 ", 0) != 0 && line.rfind("G0 ", 0) != 0)
            continue;
        const auto x = word(line, 'X'), y = word(line, 'Y'), z = word(line, 'Z'), e = word(line, 'E');
        if (!t.travelled && e && *e < 0. && !x && !y)
            t.retracted_before_travel = true;
        if (!t.travelled && z && !x && !y) {
            if (!seen_z && !t.retracted_before_travel)
                t.first_z_before_retract = true;
            seen_z = true;
            t.lowest_z_before_travel = std::min(t.lowest_z_before_travel, *z);
        }
        if (t.travelled) {
            if (z) {
                t.first_z_after_travel = *z;
                break;
            }
            continue;
        }
        if ((x || y) && !(e && *e > 0.))
            t.travelled = true;
    }
    return t;
}

Transition slice_transition(MixedNozzleSlicingMode mode, bool retract_on_layer_change = true)
{
    Scene scene = feature_cube();
    scene.config.set_key_value("mixed_nozzle_slicing_mode", new ConfigOptionEnum<MixedNozzleSlicingMode>(mode));
    scene.config.set_key_value("machine_start_gcode", new ConfigOptionString(k_start_gcode));
    scene.config.set_key_value("enable_prime_tower", new ConfigOptionBool(true));
    const size_t filaments = scene.config.option<ConfigOptionFloats>("filament_diameter")->values.size();
    scene.config.set_key_value("retract_when_changing_layer", new ConfigOptionBools(filaments, retract_on_layer_change));
    // The H2D profiles' Auto Lift.
    scene.config.set_key_value("z_hop_types", new ConfigOptionEnumsGeneric(print_config_def.get("z_hop_types")->enum_keys_map, filaments, int(ZHopType::zhtAuto)));
    const Facts facts = slice(scene);
    REQUIRE(facts.refusal.string.empty());
    // Both modes print a tower, the case where stock drops to the first layer at the load line.
    REQUIRE_FALSE(facts.emitted.tool_changes.empty());
    return first_transition(facts.gcode);
}
} // namespace

TEST_CASE("The first travel after the start G-code does not drag the load line", "[TestRebuild][StartLine]")
{
    SECTION("Feature Split retracts and stays at the start height until it reaches the tower")
    {
        const Transition t = slice_transition(MixedNozzleSlicingMode::FeatureSplit);
        INFO(t.text);
        REQUIRE(t.travelled);
        CHECK(t.retracted_before_travel);
        CHECK_FALSE(t.first_z_before_retract);
        CHECK(t.lowest_z_before_travel >= k_start_z - 1e-6);
        // The descent happens at the tower, after the travel.
        CHECK(t.first_z_after_travel > 0.);
        CHECK(t.first_z_after_travel < k_start_z);
    }
    SECTION("Feature Split without layer-change retraction still travels before it descends")
    {
        const Transition t = slice_transition(MixedNozzleSlicingMode::FeatureSplit, false);
        INFO(t.text);
        REQUIRE(t.travelled);
        CHECK(t.retracted_before_travel);
        CHECK(t.lowest_z_before_travel >= k_start_z - 1e-6);
        CHECK(t.first_z_after_travel > 0.);
        CHECK(t.first_z_after_travel < k_start_z);
    }
    SECTION("Off keeps the stock drop to the first layer before the retract")
    {
        const Transition t = slice_transition(MixedNozzleSlicingMode::Off);
        INFO(t.text);
        REQUIRE(t.travelled);
        CHECK(t.first_z_before_retract);
        CHECK(t.lowest_z_before_travel < k_start_z);
    }
}
