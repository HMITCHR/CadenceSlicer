#include <catch2/catch_all.hpp>
#include "mixed_nozzle_harness.hpp"
#include <cmath>
#include <memory>
#include <sstream>

using namespace Slic3r;
using namespace CadenceTest;

namespace {
StringObjectException admission(Scene scene)
{
    mixed_nozzle_fixtures::fill_per_filament_values(scene.config);
    Model model;
    Print print;
    print.is_BBL_printer() = scene.bambu;
    scene.populate(model, print, scene.config);
    return print.validate();
}
}

TEST_CASE("Both mixed modes refuse incompatible tower settings and recover when corrected", "[TestRebuild][Admission]")
{
    const bool body = GENERATE(false, true);
    Scene scene = body ? body_coupon() : feature_cube();
    REQUIRE(admission(scene).string.empty());
    const std::vector<std::pair<std::string, std::string>> edits {
        {"wipe_tower_filament", "1"}, {"wipe_tower_rotation_angle", "45"},
        {"prime_tower_flat_ironing", "1"}, {"enable_tower_interface_features", "1"},
        {"use_relative_e_distances", "0"}, {"ooze_prevention", "1"},
        {"prime_volume_mode", "Saving"}, {"purge_in_prime_tower", "1"},
        {"single_extruder_multi_material_priming", "1"}, {"enable_filament_ramming", "1"}
    };
    for (const auto &[key, value] : edits) {
        CAPTURE(body, key);
        Scene rejected = scene;
        rejected.config.set_deserialize_strict(key, value);
        const auto error = admission(rejected);
        INFO(error.string);
        REQUIRE_FALSE(error.string.empty());
        CHECK(error.opt_key == key);
        CHECK(error.string.find("SRL-A32") != std::string::npos);
    }
    const auto corrected = slice(scene);
    REQUIRE(corrected.refusal.string.empty());
    CHECK_FALSE(corrected.gcode.empty());
    CHECK_FALSE(corrected.emitted.tool_changes.empty());
}

TEST_CASE("Feature setup identifies unsupported infill and unresolved support before slicing", "[TestRebuild][Admission]")
{
    Scene scene = feature_cube();
    const std::vector<std::pair<std::string, std::string>> edits {
        {"sparse_infill_pattern", "lightning"}, {"infill_combination", "1"}
    };
    for (const auto &[key, value] : edits) {
        Scene invalid = scene;
        invalid.config.set_deserialize_strict(key, value);
        const auto error = admission(invalid);
        CAPTURE(key);
        INFO(error.string);
        REQUIRE_FALSE(error.string.empty());
        CHECK(error.opt_key == key);
    }
    scene.config.set_key_value("enable_support", new ConfigOptionBool(true));
    scene.config.set_key_value("support_filament", new ConfigOptionInt(0));
    const auto unresolved = admission(scene);
    REQUIRE_FALSE(unresolved.string.empty());
    CHECK(unresolved.opt_key == "support_filament");
    CHECK(unresolved.string.find("SRL-F01") != std::string::npos);
}

TEST_CASE("A surface material uses its mapped nozzle only when that nozzle can print the surface height", "[TestRebuild][Admission]")
{
    Scene scene = feature_cube();
    scene.config.set_key_value("top_shell_layers", new ConfigOptionInt(3));
    scene.config.set_key_value("dont_filter_internal_bridges", new ConfigOptionEnum<InternalBridgeFilter>(ibfNofilter));
    scene.config.set_key_value("top_surface_filament_id", new ConfigOptionInt(7));
    scene.config.set_key_value("min_layer_height", new ConfigOptionFloats{.04, .16});
    const auto rejected = admission(scene);
    INFO(rejected.string);
    REQUIRE_FALSE(rejected.string.empty());
    CHECK(rejected.opt_key == "top_surface_filament_id");
    CHECK(rejected.string.find("SRL-F12") != std::string::npos);

    scene.config.set_key_value("min_layer_height", new ConfigOptionFloats{.04, .12});
    scene.config.set_key_value("internal_solid_filament_id", new ConfigOptionInt(7));
    const auto corrected = slice(scene);
    INFO(corrected.refusal.string);
    REQUIRE(corrected.refusal.string.empty());
    size_t top_roads = 0, wall_roads = 0, bridge_roads = 0;
    size_t bad_top_tool = 0, bad_top_height = 0, bad_top_flow = 0;
    size_t bad_bridge_tool = 0, bad_bridge_flow = 0, bad_wall_tool = 0;
    std::string first_bad;
    const auto count_bad = [&](bool good, size_t &bad, const char *rule, const auto &move) {
        if (good)
            return;
        ++bad;
        if (first_bad.empty()) {
            std::ostringstream detail;
            detail << rule << " role=" << int(move.extrusion_role) << " physical=" << int(move.physical_tool_id)
                   << " z=" << move.position.z() << " height=" << move.height
                   << " flow=" << move.mm3_per_mm;
            first_bad = detail.str();
        }
    };
    for (const auto &move : corrected.moves) {
        if (!model_road(move))
            continue;
        if (move.extrusion_role == erTopSolidInfill) {
            ++top_roads;
            count_bad(move.physical_tool_id == 1, bad_top_tool, "top tool", move);
            count_bad(std::abs(move.height - .12f) <= .001f, bad_top_height, "top height", move);
            count_bad(move.mm3_per_mm > 0., bad_top_flow, "top flow", move);
        } else if (move.extrusion_role == erInternalBridgeInfill) {
            ++bridge_roads;
            count_bad(move.physical_tool_id == 1, bad_bridge_tool, "bridge tool", move);
            count_bad(move.mm3_per_mm > 0., bad_bridge_flow, "bridge flow", move);
        } else if (move.extrusion_role == erExternalPerimeter) {
            ++wall_roads;
            count_bad(move.physical_tool_id == 0, bad_wall_tool, "wall tool", move);
        }
    }
    CAPTURE(bad_top_tool, bad_top_height, bad_top_flow, bad_bridge_tool, bad_bridge_flow, bad_wall_tool, first_bad);
    CHECK(bad_top_tool == 0);
    CHECK(bad_top_height == 0);
    CHECK(bad_top_flow == 0);
    CHECK(bad_bridge_tool == 0);
    CHECK(bad_bridge_flow == 0);
    CHECK(bad_wall_tool == 0);
    CHECK(top_roads > 0);
    CHECK(bridge_roads > 0);
    CHECK(wall_roads > 0);
}

TEST_CASE("A layer height a nozzle cannot lay is refused naming the nozzle, the heights it lays and the setting to change",
          "[TestRebuild][Admission]")
{
    // The combo sweep's Body Split refusal for a first layer outside the fine nozzle's range read "A body's layer
    // height or the shared first layer is outside what its nozzle can print ...", naming neither the nozzle nor the
    // value to change. Feature Split named setting keys ("top_surface_filament_id names filament 7 ...").
    SECTION("Body Split, a first layer too thick for the fine nozzle") {
        Scene scene = body_coupon();
        scene.config.set_key_value("initial_layer_print_height", new ConfigOptionFloat(0.3));
        const StringObjectException error = admission(scene);
        INFO(error.string);
        REQUIRE(error.string.find("SRL-A38") != std::string::npos);
        CHECK(error.string.find("The first layer is 0.3 mm") != std::string::npos);
        CHECK(error.string.find("0.2 mm nozzle") != std::string::npos);
        CHECK(error.string.find("Set First layer height to at most") != std::string::npos);
        CHECK(error.string.find("_height") == std::string::npos);
    }
    SECTION("Feature Split, top surfaces on a nozzle that cannot lay the layer height") {
        Scene scene = feature_cube();
        scene.config.set_key_value("top_shell_layers", new ConfigOptionInt(3));
        scene.config.set_key_value("dont_filter_internal_bridges", new ConfigOptionEnum<InternalBridgeFilter>(ibfNofilter));
        scene.config.set_key_value("top_surface_filament_id", new ConfigOptionInt(7));
        scene.config.set_key_value("min_layer_height", new ConfigOptionFloats{.04, .16});
        const StringObjectException error = admission(scene);
        INFO(error.string);
        REQUIRE(error.string.find("SRL-F12") != std::string::npos);
        CHECK(error.opt_key == "top_surface_filament_id");
        CHECK(error.string.find("prints its top surfaces with filament 7 on the") != std::string::npos);
        CHECK(error.string.find("which lays layers from 0.16 mm to") != std::string::npos);
        CHECK(error.string.find("Change Layer height to fit that nozzle, or put the top surfaces on a filament on the other nozzle.") != std::string::npos);
        CHECK(error.string.find("_filament_id") == std::string::npos);
    }
}
