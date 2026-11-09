#include <catch2/catch_all.hpp>
#include "mixed_nozzle_harness.hpp"
#include "libslic3r/MixedNozzleRanking.hpp"
#include "libslic3r/libslic3r.h"
#include <algorithm>
#include <array>
#include <cmath>
#include <fstream>
#include <functional>
#include <limits>
#include <map>
#include <set>
#include <sstream>

using namespace Slic3r;
using namespace CadenceTest;
using namespace mixed_nozzle_fixtures;

namespace {
void require_export(const Facts &facts)
{
    INFO(facts.refusal.string);
    REQUIRE(facts.refusal.string.empty());
    REQUIRE_FALSE(facts.gcode.empty());
    REQUIRE(facts.seconds > 0.);
    REQUIRE_FALSE(facts.emitted.role_facts.empty());
    CHECK_FALSE(facts.emitted.plan_malformed);
}

void count_bad(bool good, size_t &bad, std::string &first, const char *rule,
               const GCodeProcessorResult::MoveVertex &move)
{
    if (good)
        return;
    ++bad;
    if (first.empty()) {
        std::ostringstream detail;
        detail << rule << " role=" << int(move.extrusion_role) << " physical=" << int(move.physical_tool_id)
               << " logical=" << int(move.extruder_id) << " z=" << move.position.z()
               << " width=" << move.width << " height=" << move.height << " flow=" << move.mm3_per_mm;
        first = detail.str();
    }
}

// The facets of the volume's mesh whose normal passes `facing`.
std::vector<int> facets_where(const ModelVolume &volume, const std::function<bool(const Vec3f &)> &facing)
{
    const indexed_triangle_set &its = volume.mesh().its;
    std::vector<int> out;
    for (size_t facet = 0; facet < its.indices.size(); ++facet) {
        const Vec3f &a = its.vertices[its.indices[facet](0)];
        const Vec3f &b = its.vertices[its.indices[facet](1)];
        const Vec3f &c = its.vertices[its.indices[facet](2)];
        if (facing((b - a).cross(c - a).normalized()))
            out.push_back(int(facet));
    }
    return out;
}
}

TEST_CASE("A profitable coarse core preserves fine walls and exports a stable priced schedule", "[TestRebuild][FeatureScene]")
{
    const int fine_slot = GENERATE(1, 3);
    const bool percent_core = GENERATE(false, true);
    CAPTURE(fine_slot, percent_core);
    Scene scene = feature_cube();
    for (const char *key : {"outer_wall_filament_id", "inner_wall_filament_id",
                            "internal_solid_filament_id", "top_surface_filament_id", "bottom_surface_filament_id"})
        scene.config.set_key_value(key, new ConfigOptionInt(fine_slot));
    scene.config.set_key_value("outer_wall_line_width", new ConfigOptionFloatOrPercent(.22, false));
    scene.config.set_key_value("inner_wall_line_width", new ConfigOptionFloatOrPercent(110., true));
    scene.config.set_key_value("initial_layer_line_width", new ConfigOptionFloatOrPercent(.22, false));
    scene.config.set_key_value("sparse_infill_line_width",
        new ConfigOptionFloatOrPercent(percent_core ? 110. : .22, percent_core));
    const Facts mixed = slice(scene);
    require_export(mixed);
    REQUIRE_FALSE(mixed.emitted.tool_changes.empty());
    std::array<size_t, 2> model_roads{};
    size_t coarse_core = 0, tower_roads = 0;
    std::array<size_t, 11> bad{};
    std::string first_bad;
    for (const auto &move : mixed.moves) {
        if (move.type != EMoveType::Extrude)
            continue;
        if (move.extrusion_role == erWipeTower) {
            ++tower_roads;
            continue;
        }
        if (!model_road(move))
            continue;
        count_bad(move.physical_tool_id < 2, bad[0], first_bad, "model tool", move);
        if (move.physical_tool_id >= 2)
            continue;
        count_bad(move.mm3_per_mm > 0., bad[1], first_bad, "model flow", move);
        ++model_roads[move.physical_tool_id];
        if (move.extrusion_role == erExternalPerimeter || move.extrusion_role == erPerimeter) {
            count_bad(move.physical_tool_id == 0, bad[2], first_bad, "wall tool", move);
            count_bad(move.extruder_id == fine_slot - 1, bad[3], first_bad, "wall slot", move);
        }
        if (move.extrusion_role == erPerimeter)
            count_bad(std::abs(move.width - .22f) <= .002f, bad[10], first_bad, "percent inner width", move);
        if (move.extrusion_role == erExternalPerimeter)
            count_bad(std::abs(move.width - .22f) <= .002f, bad[4], first_bad, "outer width", move);
        if (move.physical_tool_id == 1) {
            count_bad(move.extruder_id == 6, bad[5], first_bad, "core slot", move);
            count_bad(move.extrusion_role == erInternalInfill, bad[6], first_bad, "core role", move);
            count_bad(move.height <= .361f, bad[7], first_bad, "core max height", move);
            count_bad(std::abs(move.width - .66f) <= .002f, bad[8], first_bad, "core width", move);
            count_bad(move.height > .12f, bad[9], first_bad, "core min height", move);
            ++coarse_core;
        }
    }
    CAPTURE(bad, first_bad);
    for (size_t count : bad)
        CHECK(count == 0);
    CHECK(model_roads[0] > 0);
    CHECK(model_roads[1] > 0);
    CHECK(coarse_core > 0);
    CHECK(tower_roads > 0);
    CHECK(mixed.emitted.plan_start_markers == mixed.emitted.plan_end_markers);
    // The header names the Cadence version, and the settings still load from the file, as G-code import does.
    CHECK(mixed.gcode.find(std::string("; generated by ") + SLIC3R_APP_NAME + " " + CADENCE_PRODUCT_VERSION + " on ") != std::string::npos);
    CHECK(mixed.gcode.find(std::string("; generated by ") + SLIC3R_APP_NAME + " " + SoftFever_VERSION + " ") == std::string::npos);
    ScopedTemporaryFile saved(".gcode");
    { std::ofstream output(saved.string(), std::ios::binary); output << mixed.gcode; }
    DynamicPrintConfig restored;
    // The test printer leaves some printer lists empty, which loading would otherwise refuse.
    REQUIRE_NOTHROW(restored.load_from_gcode_file(saved.string(), ForwardCompatibilitySubstitutionRule::EnableSilent));
    CHECK(restored.has("layer_height"));

    const Facts repeated = slice(scene);
    require_export(repeated);
    CHECK(Test::strip_nondeterministic_lines(mixed.gcode) == Test::strip_nondeterministic_lines(repeated.gcode));

    auto populate = scene.populate;
    scene.populate = [populate, fine_slot](Model &model, Print &print, const DynamicPrintConfig &config) {
        populate(model, print, config);
        DynamicPrintConfig fine = config;
        mixed_nozzle_single_nozzle_baseline(model, fine, fine_slot - 1, config.opt_float("layer_height"));
        print.apply(model, fine);
    };
    const Facts baseline = slice(scene);
    require_export(baseline);
    CHECK(mixed.seconds < baseline.seconds);
}

TEST_CASE("A saved single-material print gains a tower when switched to Feature Split",
          "[TestRebuild][FeatureScene]")
{
    DynamicPrintConfig mixed = feature_cube().config;
    fill_per_filament_values(mixed);
    for (const char *key : {"outer_wall_filament_id", "inner_wall_filament_id",
                            "internal_solid_filament_id", "top_surface_filament_id", "bottom_surface_filament_id"})
        mixed.set_key_value(key, new ConfigOptionInt(1));
    mixed.set_key_value("outer_wall_line_width", new ConfigOptionFloatOrPercent(.22, false));
    mixed.set_key_value("inner_wall_line_width", new ConfigOptionFloatOrPercent(.22, false));
    mixed.set_key_value("initial_layer_line_width", new ConfigOptionFloatOrPercent(.22, false));

    DynamicPrintConfig single = mixed;
    single.set_key_value("mixed_nozzle_slicing_mode",
                         new ConfigOptionEnum<MixedNozzleSlicingMode>(MixedNozzleSlicingMode::Off));
    single.set_key_value("sparse_infill_filament_id", new ConfigOptionInt(1));
    single.set_key_value("enable_prime_tower", new ConfigOptionBool(false));

    Model model;
    Print print;
    print.is_BBL_printer() = true;
    ModelObject *object = model.add_object();
    object->add_volume(make_cube(80., 80., 2.16), ModelVolumeType::MODEL_PART, false);
    object->add_instance();
    object->ensure_on_bed();
    object->instances.front()->set_offset(Vec3d(40., 40., 0.));
    print.apply(model, single);
    const Facts before = slice(print);
    require_export(before);
    print.apply(model, mixed);
    const Facts after = slice(print);
    require_export(after);
    std::array<size_t, 2> model_roads{};
    size_t tower_roads = 0;
    size_t invalid_model_roads = 0;
    for (const auto &move : after.moves) {
        if (move.type != EMoveType::Extrude)
            continue;
        if (move.extrusion_role == erWipeTower) {
            ++tower_roads;
            continue;
        }
        if (!model_road(move))
            continue;
        if (move.physical_tool_id >= 2) {
            ++invalid_model_roads;
            continue;
        }
        ++model_roads[move.physical_tool_id];
        if (move.width <= 0. || move.height <= 0. ||
            move.height > (move.physical_tool_id == 0 ? .141f : .421f) || move.mm3_per_mm <= 0.)
            ++invalid_model_roads;
    }
    CHECK(invalid_model_roads == 0);
    CHECK(model_roads[0] > 0);
    CHECK(model_roads[1] > 0);
    CHECK(tower_roads > 0);
}

TEST_CASE("Expensive nozzle changes keep a small core fine without paying for a tower", "[TestRebuild][FeatureScene]")
{
    Scene scene = feature_cube(1.08, 20.);
    scene.config = feature_economics_baseline_config();
    const Facts fine = slice(scene);
    require_export(fine);
    size_t roads = 0, towers = 0, bad_tool = 0, bad_height = 0;
    std::string first_bad;
    for (const auto &move : fine.moves) {
        if (model_road(move)) {
            ++roads;
            count_bad(move.physical_tool_id == 0, bad_tool, first_bad, "fallback tool", move);
            count_bad(move.height <= .121f, bad_height, first_bad, "fallback height", move);
        }
        if (move.type == EMoveType::Extrude && move.extrusion_role == erWipeTower)
            ++towers;
    }
    CAPTURE(bad_tool, bad_height, first_bad);
    REQUIRE(roads > 0);
    CHECK(bad_tool == 0);
    CHECK(bad_height == 0);
    CHECK(towers == 0);
    CHECK(fine.emitted.tool_changes.empty());

    Scene all_fine = scene;
    const auto populate = all_fine.populate;
    all_fine.populate = [populate](Model &model, Print &print, const DynamicPrintConfig &config) {
        populate(model, print, config);
        DynamicPrintConfig baseline = config;
        mixed_nozzle_single_nozzle_baseline(model, baseline, 0, config.opt_float("layer_height"));
        print.apply(model, baseline);
    };
    const Facts exported_baseline = slice(all_fine);
    require_export(exported_baseline);
    CAPTURE(fine.seconds, exported_baseline.seconds);
    CHECK(fine.seconds <= exported_baseline.seconds * 1.005 + 1.);
}

TEST_CASE("A tower coupon primes both materials and leaves balanced fine nozzle returns", "[TestRebuild][TowerScene]")
{
    Scene scene = body_coupon();
    const std::string changed = escape_strings_cstyle({"filament_prime_volume"});
    scene.config.set_key_value("different_settings_to_system", new ConfigOptionStrings{"", changed, changed, ""});
    scene.config.set_key_value("enable_arc_fitting", new ConfigOptionEnum<ArcFittingMode>(ArcFittingMode::Enabled));
    scene.config.set_key_value("support_object_skip_flush", new ConfigOptionBool(true));
    const Facts facts = slice(scene);
    require_export(facts);
    CHECK(facts.gcode.find("M624 ") != std::string::npos);
    std::map<unsigned int, std::array<size_t, 3>> labels_by_layer;
    for (const auto &move : facts.moves) {
        if (move.type != EMoveType::Extrude)
            continue;
        auto &counts = labels_by_layer[move.layer_id];
        if (move.extrusion_role == erWipeTower && move.physical_tool_id == 1)
            ++counts[0];
        else if (model_road(move) && move.physical_tool_id == 1)
            ++counts[1];
        else if (model_road(move) && move.physical_tool_id == 0)
            ++counts[2];
    }
    size_t idle_tower_layers = 0;
    for (const auto &[layer, counts] : labels_by_layer)
        idle_tower_layers += counts[0] > 0 && counts[1] == 0 && counts[2] > 0;
    CAPTURE(idle_tower_layers);
    CHECK(idle_tower_layers > 0);
    // M624 carries only object labels; an idle scheduled tower tool has none to encode.
    size_t empty_labels = 0;
    std::istringstream gcode_lines(facts.gcode);
    for (std::string line; std::getline(gcode_lines, line); )
        empty_labels += line.rfind("M624 ", 0) == 0 && line.find_first_not_of(" \r", 5) == std::string::npos;
    CHECK(empty_labels == 0);
    const auto exits = audit_tower_exit(facts.gcode, .7);
    REQUIRE(exits.returns > 0);
    CHECK(exits.unbalanced == 0);
    CHECK(exits.moves_off_rows == 0);
    CHECK(exits.returns_without_wipe == 0);
    CHECK(exits.not_lifted_next == 0);
    CHECK(exits.crossings == 0);
    const auto grid = audit_tower_grid_speed(facts.gcode);
    REQUIRE(grid.grid_roads > 0);
    CHECK(grid.faster_roads == 0);
    PrintConfig resolved;
    resolved.apply(scene.config, true);
    std::array<size_t, 2> warm{};
    for (const auto &purge : scan_arriving_purges(facts.gcode, {1.75, 1.75})) {
        if (!purge.warm_return)
            continue;
        REQUIRE(purge.new_nozzle >= 0);
        REQUIRE(purge.new_nozzle < 2);
        ++warm[purge.new_nozzle];
        CHECK(purge.positive_moves > 0);
        REQUIRE(purge.new_filament >= 0);
        const double target = resolved.filament_prime_volume.get_at(size_t(purge.new_filament));
        const double allowance = prime_row_allowance_mm3(resolved, purge.new_nozzle,
                                                            std::max(purge.max_emitted_height, .20));
        CAPTURE(purge.volume_mm3, target, allowance);
        CHECK(purge.volume_mm3 >= target - allowance - .02);
        CHECK(purge.volume_mm3 <= target + allowance + .02);
        CHECK(purge.volume_mm3 > 15. + allowance);
    }
    CHECK(warm[0] > 0);
    CHECK(warm[1] > 0);
    const Facts repeated = slice(scene);
    require_export(repeated);
    CHECK(Test::strip_nondeterministic_lines(facts.gcode) ==
          Test::strip_nondeterministic_lines(repeated.gcode));
}

TEST_CASE("Changing three dimensional infill preserves coarse deposition and legal tower steps", "[TestRebuild][TowerScene]")
{
    const auto pattern = GENERATE(ipGyroid, ipCubic, ip3DHoneycomb);
    Scene scene = feature_cube(4.32, 80.);
    scene.config.set_key_value("sparse_infill_pattern", new ConfigOptionEnum<InfillPattern>(pattern));
    const Facts facts = slice(scene);
    require_export(facts);
    size_t sparse = 0, tower = 0;
    std::array<double, 2> tower_top{};
    std::array<size_t, 6> bad{};
    std::string first_bad;
    for (const auto &move : facts.moves) {
        if (move.type != EMoveType::Extrude)
            continue;
        if (model_road(move) && move.physical_tool_id == 1) {
            ++sparse;
            count_bad(move.extrusion_role == erInternalInfill, bad[0], first_bad, "coarse role", move);
            count_bad(move.height <= .361f, bad[1], first_bad, "coarse height", move);
        }
        if (move.extrusion_role != erWipeTower)
            continue;
        count_bad(move.physical_tool_id < 2, bad[2], first_bad, "tower tool", move);
        if (move.physical_tool_id >= 2)
            continue;
        const auto tool = move.physical_tool_id;
        ++tower;
        count_bad(move.height > 0., bad[3], first_bad, "tower positive height", move);
        count_bad(move.height <= (tool == 0 ? .141 : .421), bad[4], first_bad, "tower max height", move);
        count_bad(move.mm3_per_mm > 0., bad[5], first_bad, "tower flow", move);
        tower_top[tool] = std::max(tower_top[tool], double(move.position.z()));
    }
    CAPTURE(bad, first_bad);
    for (size_t count : bad)
        CHECK(count == 0);
    CHECK(sparse > 0);
    CHECK(tower > 0);
    CHECK(tower_top[0] > .12);
    CHECK(tower_top[1] > .12);
}

TEST_CASE("A tapered feature print preserves its fine exterior and finishes above its last coarse band", "[TestRebuild][FeatureScene]")
{
    Scene scene = feature_cube();
    scene.populate = [](Model &model, Print &print, const DynamicPrintConfig &config) {
        auto *object = model.add_object();
        object->add_volume(make_cone(30., 30.));
        object->add_instance();
        object->instances.front()->set_offset(Vec3d(65., 65., 0.));
        object->ensure_on_bed();
        print.apply(model, config);
    };
    const Facts facts = slice(scene);
    require_export(facts);
    double fine_top = 0., coarse_top = 0.;
    size_t fine_walls = 0, coarse_roads = 0, bad_tool = 0, bad_role = 0;
    std::string first_bad;
    for (const auto &move : facts.moves) {
        if (!model_road(move))
            continue;
        if (move.physical_tool_id == 0) {
            fine_top = std::max(fine_top, double(move.position.z()));
            if (move.extrusion_role == erExternalPerimeter)
                ++fine_walls;
        } else {
            count_bad(move.physical_tool_id == 1, bad_tool, first_bad, "taper tool", move);
            count_bad(move.extrusion_role == erInternalInfill, bad_role, first_bad, "taper coarse role", move);
            coarse_top = std::max(coarse_top, double(move.position.z()));
            ++coarse_roads;
        }
    }
    CAPTURE(bad_tool, bad_role, first_bad);
    CHECK(bad_tool == 0);
    CHECK(bad_role == 0);
    CHECK(fine_walls > 0);
    CHECK(coarse_roads > 0);
    CHECK(fine_top > coarse_top);
    CHECK(fine_top >= 29.8);

    Scene wedge = feature_cube();
    wedge.config.set_key_value("gap_fill_target", new ConfigOptionEnum<GapFillTarget>(gftEverywhere));
    wedge.config.set_key_value("filter_out_gap_fill", new ConfigOptionFloat(0.));
    wedge.config.set_key_value("wall_loops", new ConfigOptionInt(2));
    wedge.populate = [](Model &model, Print &print, const DynamicPrintConfig &config) {
        ModelObject *object = model.add_object();
        // Rotate a triangular roof prism so its triangle is an XY wedge on every Z layer.
        TriangleMesh triangle = make_prism(30.f, 4.f, 6.f);
        triangle.rotate_x(float(M_PI_2));
        triangle.translate(0.f, 6.f, 2.f);
        object->add_volume(std::move(triangle));
        object->add_instance();
        object->instances.front()->set_offset(Vec3d(65., 65., 0.));
        object->ensure_on_bed();
        print.apply(model, config);
    };
    const Facts wedge_facts = slice(wedge);
    require_export(wedge_facts);
    size_t gap_roads = 0, wrong_gap_tool = 0;
    for (const auto &move : wedge_facts.moves)
        if (model_road(move) && move.extrusion_role == erGapFill) {
            ++gap_roads;
            wrong_gap_tool += move.physical_tool_id != 0;
        }
    CAPTURE(gap_roads, wrong_gap_tool);
    CHECK(gap_roads > 0);
    CHECK(wrong_gap_tool == 0);
}

// Under Feature Split a painted color replaces the filament only on what the fine nozzle prints: the coarse
// nozzle's sparse infill keeps the part's own coarse filament. Stock paint with no maximum width
// (mmu_segmented_region_max_width 0) reaches across the interior; on a part painted on one side Feature Split
// stops the paint at the depth of the walls, so the sparse infill behind it stays coarse. A layer painted on
// every side keeps the paint all the way in, so the part's own filament adds no color change there.
TEST_CASE("Paint on a Feature Split part reaches only as deep as the walls and keeps the coarse sparse infill",
          "[TestRebuild][FeatureScene]")
{
    const int paint_slot = 2; // filament_map {1,1,1,1,1,1,2,1}: slot 2 is on the fine 0.2 nozzle.
    // Top: only the top is painted, with top shells 3. Its paint colors as many layers as the top paint
    // depth; a project saved before that option existed loads it as 0, which colors every top shell layer.
    enum class Pattern { All, OneSide, EverySide, Top };
    const auto row = GENERATE(table<Pattern, int>(
        {{Pattern::All, 0}, {Pattern::OneSide, 0}, {Pattern::EverySide, 0}, {Pattern::Top, 0}, {Pattern::Top, 2}}));
    const Pattern pattern = std::get<0>(row);
    const int top_depth = std::get<1>(row);
    CAPTURE(int(pattern), top_depth);
    Scene scene = feature_cube(pattern == Pattern::All || pattern == Pattern::Top ? 2.16 : 4.);
    REQUIRE(scene.config.opt_float("mmu_segmented_region_max_width") == 0.);
    if (pattern == Pattern::All || pattern == Pattern::Top) {
        scene.config.set_key_value("top_shell_layers", new ConfigOptionInt(3));
        scene.config.set_key_value("top_shell_thickness", new ConfigOptionFloat(0.));
    }
    if (pattern == Pattern::Top)
        scene.config.set_key_value("top_color_penetration_layers", new ConfigOptionInt(top_depth));
    if (pattern == Pattern::OneSide) {
        // A filament change on one nozzle loads and unloads, and this paint changes color on the fine nozzle on
        // every layer. Those changes are in both plans the whole-print check compares, so they must not decide it.
        scene.config.set_key_value("machine_load_filament_time", new ConfigOptionFloat(500.));
        scene.config.set_key_value("machine_unload_filament_time", new ConfigOptionFloat(500.));
    }
    const auto populate = scene.populate;
    scene.populate = [populate, pattern](Model &model, Print &print, const DynamicPrintConfig &config) {
        populate(model, print, config);
        ModelVolume &volume = *model.objects.front()->volumes.front();
        const std::vector<int> facets = facets_where(volume, [pattern](const Vec3f &normal) {
            if (pattern == Pattern::Top)
                return normal.z() > .9f;
            return pattern == Pattern::All || (pattern == Pattern::OneSide ? normal.x() > .9f : std::abs(normal.z()) < .1f);
        });
        REQUIRE_FALSE(facets.empty());
        paint_cadence_facets(volume, facets, EnforcerBlockerType(paint_slot));
        print.apply(model, config);
    };
    const Facts painted = slice(scene);
    require_export(painted);
    if (pattern == Pattern::Top) {
        std::set<int> painted_layers;
        for (const auto &move : painted.moves)
            if (model_road(move) && move.extruder_id == paint_slot - 1)
                painted_layers.insert(int(std::lround(move.position.z() * 1000.f)));
        CAPTURE(painted_layers);
        CHECK(painted_layers.size() == size_t(top_depth == 0 ? 3 : top_depth));
        return;
    }

    const auto is_paint = [](const GCodeProcessorResult::MoveVertex &move) { return move.extruder_id == paint_slot - 1; };
    const auto is_wall = [](const GCodeProcessorResult::MoveVertex &move) {
        return move.extrusion_role == erExternalPerimeter || move.extrusion_role == erPerimeter;
    };
    size_t coarse_core = 0, painted_walls = 0, painted_skins = 0;
    std::array<size_t, 5> bad{};
    std::string first_bad;
    for (const auto &move : painted.moves) {
        if (!model_road(move))
            continue;
        if (move.physical_tool_id == 1 && move.extrusion_role == erInternalInfill)
            ++coarse_core;
        if (is_wall(move) && is_paint(move))
            ++painted_walls;
        // The coarse nozzle lays only sparse infill, in the part's own coarse filament (slot 7).
        if (move.physical_tool_id == 1 && pattern != Pattern::EverySide)
            count_bad(move.extrusion_role == erInternalInfill && move.extruder_id == 6, bad[0], first_bad, "coarse core", move);
        if (pattern == Pattern::All) {
            if (is_wall(move))
                count_bad(move.physical_tool_id == 0 && is_paint(move), bad[1], first_bad, "painted wall", move);
            if (move.extrusion_role == erTopSolidInfill || move.extrusion_role == erSolidInfill ||
                move.extrusion_role == erBottomSurface) {
                ++painted_skins;
                count_bad(move.physical_tool_id == 0 && is_paint(move), bad[2], first_bad, "painted skin", move);
            }
            // A sparse layer the hand-off leaves fine-owned falls back to the fine wall filament.
            if (move.extrusion_role == erInternalInfill && move.physical_tool_id == 0)
                count_bad(is_paint(move), bad[3], first_bad, "fine-owned sparse", move);
        }
        // The unpainted bottom shells keep the part's own filament, as in stock painting.
        if (pattern == Pattern::EverySide && is_wall(move) && !is_paint(move) && move.position.z() > .5f)
            count_bad(false, bad[4], first_bad, "wall in the part's own filament", move);
    }
    CAPTURE(bad, first_bad, coarse_core, painted_walls);
    for (size_t count : bad)
        CHECK(count == 0);
    CHECK(coarse_core > 0);
    CHECK(painted_walls > 0);
    if (pattern == Pattern::All)
        CHECK(painted_skins > 0);
    if (pattern != Pattern::OneSide)
        return;

    float face_x = std::numeric_limits<float>::lowest(), min_x = std::numeric_limits<float>::max();
    std::map<unsigned int, std::set<unsigned int>> wall_slots_by_layer;
    size_t plain_walls = 0, fine_tool_walls = 0;
    for (const auto &move : painted.moves)
        if (model_road(move) && move.extrusion_role == erExternalPerimeter) {
            face_x = std::max(face_x, move.position.x());
            min_x  = std::min(min_x, move.position.x());
            wall_slots_by_layer[move.layer_id].insert(move.extruder_id);
            plain_walls += is_paint(move) ? 0 : 1;
            fine_tool_walls += move.physical_tool_id == 0 ? 1 : 0;
        }
    REQUIRE(face_x > min_x + 40.f);
    size_t layers_with_both = 0;
    for (const auto &[layer, slots] : wall_slots_by_layer)
        layers_with_both += slots.size() > 1 ? 1 : 0;
    // The paint really changes color on the fine nozzle layer after layer.
    CHECK(plain_walls > 0);
    CHECK(layers_with_both > 3);
    const float mid_x = .5f * (face_x + min_x);
    // The walls of a 0.2 mm nozzle are about 1 mm deep; 2 mm leaves room for the paint's inner loop.
    const float paint_depth_limit = 2.f;
    size_t painted_face_walls = 0, deep_paint = 0, painted_half_coarse = 0, painted_half_fine_sparse = 0, outer_walls = 0;
    std::string first_deep;
    for (const auto &move : painted.moves) {
        if (!model_road(move))
            continue;
        const bool paint = move.physical_tool_id == 0 && is_paint(move);
        if (move.extrusion_role == erExternalPerimeter)
            ++outer_walls;
        if (paint && move.extrusion_role == erExternalPerimeter && move.position.x() > face_x - .5f)
            ++painted_face_walls;
        if (paint)
            count_bad(move.position.x() > face_x - paint_depth_limit, deep_paint, first_deep, "paint deeper than the walls", move);
        if (move.extrusion_role == erInternalInfill && move.position.x() > mid_x)
            ++(move.physical_tool_id == 1 ? painted_half_coarse : painted_half_fine_sparse);
    }
    CAPTURE(face_x, min_x, painted_face_walls, deep_paint, first_deep, painted_half_coarse, painted_half_fine_sparse);
    // Every outer wall is on the fine nozzle, and the paint shows on the painted face.
    CHECK(fine_tool_walls == outer_walls);
    CHECK(painted_face_walls > 0);
    // Nothing deeper than the walls prints in the paint color, and the sparse infill behind it is coarse.
    CHECK(deep_paint == 0);
    CHECK(painted_half_coarse > 0);
    CHECK(painted_half_fine_sparse == 0);
}
