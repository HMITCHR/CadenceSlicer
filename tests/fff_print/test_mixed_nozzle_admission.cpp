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

TEST_CASE("Feature Split with the outer walls and sparse infill on one nozzle says so and what to set back",
          "[TestRebuild][Admission]")
{
    // MNS-CAD-A05 used to read "Feature Split sparse infill needs a coarse layer height that both nozzles can
    // print. [MNS-CAD-A05] The fine and coarse materials must print on different nozzles.", which names no
    // setting. Filaments 1 to 6 are on the 0.2 mm nozzle and filament 7 on the 0.6 mm nozzle here.
    const auto refusal = [](const std::vector<std::pair<std::string, int>> &object_keys,
                            const std::vector<std::pair<std::string, int>> &plate_keys) {
        Scene scene = feature_cube();
        for (const auto &[key, filament] : plate_keys)
            scene.config.set_key_value(key, new ConfigOptionInt(filament));
        mixed_nozzle_fixtures::fill_per_filament_values(scene.config);
        Model model;
        Print print;
        print.is_BBL_printer() = true;
        scene.populate(model, print, scene.config);
        for (const auto &[key, filament] : object_keys)
            model.objects.front()->config.set_key_value(key, new ConfigOptionInt(filament));
        print.apply(model, scene.config);
        print.set_status_silent();
        const StringObjectException validation = print.validate();
        if (!validation.string.empty())
            return validation.string;
        try {
            print.process();
        } catch (const SlicingError &error) {
            return std::string(error.what());
        }
        return std::string();
    };
    SECTION("Outer walls moved to the coarse nozzle on one object") {
        const std::string text = refusal({{"outer_wall_filament_id", 7}}, {});
        INFO(text);
        CHECK(text == "Feature Split prints the outer walls on the fine nozzle and the sparse infill on the coarse "
                      "nozzle, but here both are on the 0.6 mm nozzle. On this object, set Outer walls back to "
                      "filament 1. [MNS-CAD-A05]");
    }
    SECTION("Sparse infill moved to the fine nozzle on one object") {
        const std::string text = refusal({{"sparse_infill_filament_id", 1}}, {});
        INFO(text);
        CHECK(text == "Feature Split prints the outer walls on the fine nozzle and the sparse infill on the coarse "
                      "nozzle, but here both are on the 0.2 mm nozzle. On this object, set Infill back to "
                      "filament 7. [MNS-CAD-A05]");
    }
    SECTION("Everything on one nozzle for the whole plate") {
        const std::string text = refusal({}, {{"sparse_infill_filament_id", 1}});
        INFO(text);
        CHECK(text == "Feature Split prints the outer walls on the fine nozzle and the sparse infill on the coarse "
                      "nozzle, but here both are on the 0.2 mm nozzle. Click Change... in the sidebar and apply "
                      "setup again to restore them. [MNS-CAD-A05]");
    }
}

TEST_CASE("The support interface on a larger nozzle than the base gets advice that fits the support type",
          "[TestRebuild][Admission][Support]")
{
    // SRL-F01 / SRL-A29 said "set Support/raft interface to Default" for every support type. Organic trees with a
    // coarse base are refused with the interface at Default (SRL-F14 / SRL-A53), so trees are told to put the
    // interface on the fine nozzle.
    PrintObjectConfig config;
    config.support_type.value = stNormalAuto;
    const std::string normal = mixed_nozzle_interface_on_larger_nozzle_message(config, true);
    CHECK(normal == "The support interface is on a larger nozzle than the support base. The interface is what touches "
                    "the model, so set Support/raft interface to Default (it then follows the base) or to a filament "
                    "on the base's nozzle or a finer one.");
    // A raft with no support is laid like Normal support.
    config.support_type.value = stTreeAuto;
    CHECK(mixed_nozzle_interface_on_larger_nozzle_message(config, false) == normal);
    const std::string tree = mixed_nozzle_interface_on_larger_nozzle_message(config, true);
    CHECK(tree == "The support interface is on a larger nozzle than the support base. The interface is what touches "
                  "the model, so put Support/raft interface on a filament on the fine nozzle. Tree support needs its "
                  "interface on the fine nozzle even when its base is on the coarse one.");
    CHECK(tree.find("Default") == std::string::npos);
}

TEST_CASE("Body Split with a feature on another filament points to Fine skins", "[TestRebuild][Admission][Body]")
{
    // SRL-A11 said only "Every feature of each body must use one filament.", with no supported way to get fine
    // tops and bottoms on a coarse part.
    Scene scene = body_coupon();
    const auto populate = scene.populate;
    scene.populate = [populate](Model &model, Print &print, const DynamicPrintConfig &config) {
        populate(model, print, config);
        // The coarse part (filament 2) with its solid infill on the fine filament.
        model.objects.front()->volumes.back()->config.set_key_value("internal_solid_filament_id", new ConfigOptionInt(1));
        print.apply(model, config);
    };
    const StringObjectException error = admission(scene);
    INFO(error.string);
    REQUIRE(error.string.find("SRL-A11") != std::string::npos);
    CHECK(error.string.find("In Body Split each part prints all its features with one filament. To print a coarse "
                            "part's top and bottom with the fine nozzle, tick Fine skins for that part in the Mixed "
                            "Nozzle section of the sidebar and click Apply.") != std::string::npos);
}

TEST_CASE("Body Split infill refusals name the setting in plain words", "[TestRebuild][Admission][Body]")
{
    // SRL-A13 and SRL-A16 used engine words ("committing the pair", "re-types small sparse islands").
    SECTION("A sparse infill pattern that changes from layer to layer") {
        Scene scene = body_coupon();
        scene.config.set_key_value("sparse_infill_pattern", new ConfigOptionEnum<InfillPattern>(ipGyroid));
        const StringObjectException error = admission(scene);
        INFO(error.string);
        REQUIRE(error.string.find("SRL-A13") != std::string::npos);
        CHECK(error.string.find("Body Split needs a sparse infill pattern that is the same on every layer. Use grid, "
                                "triangles, stars or aligned rectilinear.") != std::string::npos);
        CHECK(error.string.find("committing") == std::string::npos);
    }
    SECTION("A nonzero minimum sparse infill threshold") {
        Scene scene = body_coupon();
        scene.config.set_key_value("minimum_sparse_infill_area", new ConfigOptionFloat(15.));
        const StringObjectException error = admission(scene);
        INFO(error.string);
        REQUIRE(error.string.find("SRL-A16") != std::string::npos);
        CHECK(error.string.find("Body Split needs Minimum sparse infill threshold set to 0 mm2.") != std::string::npos);
        CHECK(error.string.find("re-types") == std::string::npos);
    }
}
