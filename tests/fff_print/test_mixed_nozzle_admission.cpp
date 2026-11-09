#include <catch2/catch_all.hpp>
#include "mixed_nozzle_harness.hpp"
#include <cmath>
#include <functional>
#include <sstream>

using namespace Slic3r;
using namespace CadenceTest;

namespace {
// What the scene is refused with: validation alone, or with `sliced` the whole slice and export, where a
// refusal thrown while slicing counts too. `object` edits the model after the scene has built it.
StringObjectException refusal(Scene scene, const std::function<void(Model &)> &object = {}, bool sliced = false)
{
    mixed_nozzle_fixtures::fill_per_filament_values(scene.config);
    if (scene.bambu && scene.config.opt_string("machine_start_gcode").find("M1020") == std::string::npos)
        scene.config.set_key_value("machine_start_gcode", new ConfigOptionString(scene.config.opt_string("machine_start_gcode") + "\nM1020 S[initial_extruder]\n"));
    Model model;
    Print print;
    print.is_BBL_printer() = scene.bambu;
    scene.populate(model, print, scene.config);
    // The Body Split coupon marks the print as Bambu; the scene decides.
    print.is_BBL_printer() = scene.bambu;
    if (object) {
        object(model);
        print.apply(model, scene.config);
    }
    print.set_status_silent();
    if (!sliced)
        return print.validate();
    StringObjectException error;
    try {
        const Facts facts = slice(print);
        error = facts.refusal;
        if (error.string.empty() && facts.gcode.empty())
            error.string = "no G-code";
    } catch (const std::exception &thrown) {
        error.string = thrown.what();
    }
    return error;
}

StringObjectException admission(const Scene &scene) { return refusal(scene); }

// The plain two-body coupon on a 0.2/0.4 pair, not a Bambu printer.
Scene coupon()
{
    Scene scene;
    scene.bambu = false;
    scene.config = mixed_nozzle_fixtures::coupon_config(true);
    scene.populate = [](Model &model, Print &print, const DynamicPrintConfig &config) {
        mixed_nozzle_fixtures::init_coupon(model, print, config);
    };
    return scene;
}

// The Feature Split tower coupon on a 0.2 and a `coarse` nozzle, with a 30 mm square 2 mm tall beside the fine body.
Scene timelapse_tower_coupon(double coarse)
{
    Scene scene;
    scene.config = mixed_nozzle_fixtures::feature_split_tower_config(.20, coarse, false);
    scene.populate = [](Model &model, Print &print, const DynamicPrintConfig &config) {
        mixed_nozzle_fixtures::init_tower_coupon(model, print, config, make_cube(30., 30., 2.));
    };
    return scene;
}

// The Body Split pad coupon on a 0.2/0.6 pair.
Scene body_pad_coupon_06()
{
    Scene scene;
    scene.config = mixed_nozzle_fixtures::body_split_pad_config(.2, .6, .1, .2, false, true, 60., wtwRib);
    scene.populate = [](Model &model, Print &print, const DynamicPrintConfig &config) {
        mixed_nozzle_fixtures::init_body_split_pad_coupon(model, print, config, false, 4., .1, .2);
    };
    return scene;
}

// An edit that sets one key.
template<typename T> std::function<void(DynamicPrintConfig &)> edit_key(const char *key, T *value)
{
    std::shared_ptr<T> owned(value);
    return [key, owned](DynamicPrintConfig &config) { config.set_key_value(key, owned->clone()); };
}
} // namespace

TEST_CASE("Both mixed modes refuse incompatible tower settings and recover when corrected", "[TestRebuild][Admission]")
{
    const bool body = GENERATE(false, true);
    Scene scene = body ? body_coupon() : feature_cube();
    REQUIRE(admission(scene).string.empty());
    const std::vector<std::pair<std::string, std::string>> edits {
        {"wipe_tower_filament", "1"}, {"wipe_tower_rotation_angle", "45"},
        {"prime_tower_flat_ironing", "1"}, {"enable_tower_interface_features", "1"},
        {"use_relative_e_distances", "0"}, {"ooze_prevention", "1"},
        {"prime_volume_mode", "Saving"}, {"single_extruder_multi_material_priming", "1"}
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

    // Purge in prime tower and filament ramming act only on the Type 2 tower. The Bambu profiles
    // leave both on, so the printer's own Type 1 tower takes them and prints the same moves.
    Scene purging = scene;
    purging.config.set_deserialize_strict("purge_in_prime_tower", "1");
    purging.config.set_deserialize_strict("enable_filament_ramming", "1");
    const auto purged = slice(purging);
    INFO(purged.refusal.string);
    REQUIRE(purged.refusal.string.empty());
    const auto moves = [](const std::string &gcode) {
        std::string out;
        std::istringstream lines(gcode);
        for (std::string line; std::getline(lines, line);)
            if (!line.empty() && line.front() != ';')
                out += line + '\n';
        return out;
    };
    CHECK(moves(purged.gcode) == moves(corrected.gcode));
    CHECK(purged.seconds == Catch::Approx(corrected.seconds));

    // A tower that is not Type 1 is still refused, with both keys on.
    Scene type2 = purging;
    type2.bambu = false;
    type2.config.set_deserialize_strict("wipe_tower_type", "type2");
    const auto type2_error = admission(type2);
    INFO(type2_error.string);
    CHECK(type2_error.string.find("SRL-A32") != std::string::npos);
    CHECK(type2_error.opt_key == "wipe_tower_type");
}

// Each row edits an admitted scene into one the modes cannot slice. The refusal is checked by its code and
// the setting it names, and the scene without the edit is admitted.
TEST_CASE("Both mixed modes refuse what they cannot slice by code and setting, and admit the plate when it is put back",
          "[TestRebuild][Admission]")
{
    struct Row {
        std::string                               name;
        std::function<Scene()>                    scene;
        std::function<void(DynamicPrintConfig &)> plate = {};
        std::function<void(Model &)>              object = {};
        std::string                               code = {};    // empty: admitted
        std::string                               opt_key = {}; // empty: a refusal while slicing, naming no setting
        bool                                      sliced = false;
        bool                                      reverts = true;
    };
    const auto feature = [] { return feature_cube(); };
    const auto body = [] { return body_coupon(); };
    const auto both = [&](const std::string &name, const std::function<void(DynamicPrintConfig &)> &plate,
                          const std::string &code, const std::string &opt_key) {
        return std::vector<Row>{{"Feature Split, " + name, feature, plate, {}, code, opt_key},
                                {"Body Split, " + name, body, plate, {}, code, opt_key}};
    };
    std::vector<Row> rows {
        {"lightning infill", feature, edit_key("sparse_infill_pattern", new ConfigOptionEnum<InfillPattern>(ipLightning)), {}, "", "sparse_infill_pattern"},
        {"infill combination", feature, edit_key("infill_combination", new ConfigOptionBool(true)), {}, "", "infill_combination"},
        {"unresolved support filament", feature, [](DynamicPrintConfig &config) {
             config.set_key_value("enable_support", new ConfigOptionBool(true));
             config.set_key_value("support_filament", new ConfigOptionInt(0));
         }, {}, "SRL-F01", "support_filament"},
        {"inner walls on another filament with one wall loop", feature, [](DynamicPrintConfig &config) {
             config.set_key_value("inner_wall_filament_id", new ConfigOptionInt(2));
             config.set_key_value("wall_loops", new ConfigOptionInt(1));
             config.set_key_value("wall_sequence", new ConfigOptionEnum<WallSequence>(WallSequence::InnerOuter));
         }, {}, "SRL-F10", "wall_loops"},
        {"inner walls on another filament in inner/outer/inner order", feature, [](DynamicPrintConfig &config) {
             config.set_key_value("inner_wall_filament_id", new ConfigOptionInt(2));
             config.set_key_value("wall_loops", new ConfigOptionInt(3));
             config.set_key_value("wall_sequence", new ConfigOptionEnum<WallSequence>(WallSequence::InnerOuterInner));
         }, {}, "SRL-F10", "wall_sequence"},
        {"inner walls on another filament with two wall loops", feature, [](DynamicPrintConfig &config) {
             config.set_key_value("inner_wall_filament_id", new ConfigOptionInt(2));
             config.set_key_value("wall_loops", new ConfigOptionInt(2));
             config.set_key_value("wall_sequence", new ConfigOptionEnum<WallSequence>(WallSequence::InnerOuter));
         }},
        // Filaments 1 to 6 are on the 0.2 mm nozzle and filament 7 on the 0.6 mm nozzle.
        {"outer walls moved to the coarse nozzle on one object", feature, {}, [](Model &model) {
             model.objects.front()->config.set_key_value("outer_wall_filament_id", new ConfigOptionInt(7));
         }, "MNS-CAD-A05", "", true},
        {"everything on the fine nozzle for the whole plate", feature, edit_key("sparse_infill_filament_id", new ConfigOptionInt(1)),
         {}, "MNS-CAD-A05", "", true},
        {"two nozzles of the same size", feature, edit_key("nozzle_diameter", new ConfigOptionFloats{0.4, 0.4}), {}, "MNS-CAD-A06", "", true},
        {"Smooth timelapse with a coarse nozzle that cannot lay the layer height", [] { return timelapse_tower_coupon(.60); },
         edit_key("timelapse_type", new ConfigOptionEnum<TimelapseType>(tlSmooth)), {}, "SRL-F17", "timelapse_type"},
        {"Traditional timelapse on 0.2/0.6", [] { return timelapse_tower_coupon(.60); },
         edit_key("timelapse_type", new ConfigOptionEnum<TimelapseType>(tlTraditional)), {}, "", "", true},
        {"Smooth timelapse on 0.2/0.4, whose coarse nozzle lays the tower layers", [] { return timelapse_tower_coupon(.40); },
         edit_key("timelapse_type", new ConfigOptionEnum<TimelapseType>(tlSmooth)), {}, "", "", true},
        {"Body Split, Smooth timelapse with a nozzle that cannot lay the base layer", body_pad_coupon_06,
         edit_key("timelapse_type", new ConfigOptionEnum<TimelapseType>(tlSmooth)), {}, "SRL-A38", "timelapse_type", false, false},
        {"Body Split, a first layer too thick for the fine nozzle", body,
         edit_key("initial_layer_print_height", new ConfigOptionFloat(0.3)), {}, "SRL-A38", "max_layer_height"},
        {"Body Split, a plain object its nozzle cannot lay", [] { return project_scene("body-split-plate-plain-coarse-object.3mf"); },
         {}, {}, "SRL-A38", "outer_wall_filament_id", false, false},
        {"Body Split, a coarse part with its solid infill on the fine filament", body, {}, [](Model &model) {
             model.objects.front()->volumes.back()->config.set_key_value("internal_solid_filament_id", new ConfigOptionInt(1));
         }, "SRL-A11", "outer_wall_filament_id"},
        {"Body Split, a sparse infill pattern that changes from layer to layer", body,
         edit_key("sparse_infill_pattern", new ConfigOptionEnum<InfillPattern>(ipGyroid)), {}, "SRL-A13", "sparse_infill_pattern"},
        {"Body Split, a minimum sparse infill threshold", body,
         edit_key("minimum_sparse_infill_area", new ConfigOptionFloat(15.)), {}, "SRL-A16", "minimum_sparse_infill_area"},
        {"Body Split, a raft", body, [](DynamicPrintConfig &config) {
             config.set_key_value("raft_layers", new ConfigOptionInt(2));
             // The raft's base on the fine nozzle.
             config.set_key_value("support_filament", new ConfigOptionInt(1));
             config.set_key_value("support_interface_filament", new ConfigOptionInt(1));
         }, {}, "SRL-A54", "raft_layers"},
        {"Body Split, a raft under two copies of the object", body, [](DynamicPrintConfig &config) {
             config.set_key_value("raft_layers", new ConfigOptionInt(2));
             config.set_key_value("support_filament", new ConfigOptionInt(1));
             config.set_key_value("support_interface_filament", new ConfigOptionInt(1));
         }, [](Model &model) {
             ModelObject *copy = model.add_object(*model.objects.front());
             for (ModelInstance *instance : copy->instances)
                 instance->set_offset(instance->get_offset() + Vec3d(60., 0., 0.));
         }, "SRL-A54", "raft_layers"},
        {"Body Split, hole conversion", coupon, edit_key("hole_to_polyhole", new ConfigOptionBool(true)), {}, "SRL-A43", "hole_to_polyhole", false, false},
        {"Body Split, separated infill", coupon, edit_key("separated_infills", new ConfigOptionBool(true)), {}, "SRL-A44", "separated_infills", false, false},
        {"Body Split, a bad manual nozzle map", coupon, [](DynamicPrintConfig &config) {
             config.set_key_value("filament_map_mode", new ConfigOptionEnum<FilamentMapMode>(fmmNozzleManual));
             config.set_key_value("filament_nozzle_map", new ConfigOptionInts{0});
         }, {}, "SRL-A39", "filament_nozzle_map", false, false},
    };
    // A single part with a modifier on the other nozzle's filament is two bodies, so the coarse one needs the
    // coarse layer height.
    for (const bool part_coarse : {false, true})
        rows.push_back({std::string("Body Split, a modifier on the other nozzle in a part on the ") + (part_coarse ? "coarse" : "fine") + " nozzle",
                        [part_coarse] {
                            return body_parts_scene({{"part", Vec3d(20., 20., 4.), Vec3d::Zero(), part_coarse ? 2 : 1},
                                                     {"logo", Vec3d(8., 3., 0.8), Vec3d(6., 8., 3.2), part_coarse ? 1 : 2, 0.,
                                                      ModelVolumeType::PARAMETER_MODIFIER}}, 0.14);
                        }, {}, {}, "SRL-A23", "regional_layer_height", false, false});
    for (Row &row : both("a belt printer", edit_key("belt_printer", new ConfigOptionBool(true)), "SRL-A56", "belt_printer"))
        rows.push_back(row);
    for (Row &row : both("IDEX/IQEX parallel printing", edit_key("is_imex", new ConfigOptionBool(true)), "SRL-A57", "is_imex"))
        rows.push_back(row);
    // Every slot marked mixed, so whichever slot the plate prints is a mixed filament.
    for (Row &row : both("a mixed filament", [](DynamicPrintConfig &config) {
             const size_t filaments = config.option<ConfigOptionFloats>("filament_diameter")->values.size();
             config.set_key_value("filament_is_mixed", new ConfigOptionBools(std::vector<unsigned char>(filaments, 1)));
         }, "SRL-A58", "filament_is_mixed"))
        rows.push_back(row);
    const auto cyclic = edit_key("toolchange_ordering", new ConfigOptionEnum<ToolChangeOrderingType>(ToolChangeOrderingType::Cyclic));
    rows.push_back({"Feature Split, cyclic tool ordering", feature, cyclic, {}, "SRL-F18", "toolchange_ordering"});
    rows.push_back({"Body Split, cyclic tool ordering", body, cyclic, {}, "SRL-A34", "toolchange_ordering"});

    for (const Row &row : rows) {
        CAPTURE(row.name);
        Scene scene = row.scene();
        if (row.reverts && (row.plate || row.object)) {
            const StringObjectException base = admission(scene);
            INFO("before the edit: " << base.string);
            CHECK(base.string.empty());
        }
        if (row.plate)
            row.plate(scene.config);
        const bool admitted = row.code.empty() && row.opt_key.empty();
        const StringObjectException error = refusal(scene, row.object, row.sliced || admitted);
        INFO(error.string);
        if (admitted) {
            CHECK(error.string.empty());
            continue;
        }
        CHECK_FALSE(error.string.empty());
        if (!row.code.empty())
            CHECK(error.string.find(row.code) != std::string::npos);
        if (!row.opt_key.empty())
            CHECK(error.opt_key == row.opt_key);
    }
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
    size_t top_roads = 0, wall_roads = 0, bridge_roads = 0, bad_top = 0, bad_bridge = 0, bad_wall = 0;
    for (const auto &move : corrected.moves) {
        if (!model_road(move))
            continue;
        if (move.extrusion_role == erTopSolidInfill) {
            ++top_roads;
            bad_top += move.physical_tool_id != 1 || std::abs(move.height - .12f) > .001f || move.mm3_per_mm <= 0.;
        } else if (move.extrusion_role == erInternalBridgeInfill) {
            ++bridge_roads;
            bad_bridge += move.physical_tool_id != 1 || move.mm3_per_mm <= 0.;
        } else if (move.extrusion_role == erExternalPerimeter) {
            ++wall_roads;
            bad_wall += move.physical_tool_id != 0;
        }
    }
    CHECK(bad_top == 0);
    CHECK(bad_bridge == 0);
    CHECK(bad_wall == 0);
    CHECK(top_roads > 0);
    CHECK(bridge_roads > 0);
    CHECK(wall_roads > 0);
}
