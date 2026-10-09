#include <catch2/catch_all.hpp>

#include "libslic3r/GCode/GCodeProcessor.hpp"
#include "libslic3r/Model.hpp"
#include "libslic3r/Print.hpp"
#include "mixed_nozzle_facts.hpp"
#include "mixed_nozzle_harness.hpp"
#include "mixed_nozzle_fixtures.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <map>
#include <limits>
#include <sstream>
#include <set>
#include <string>
#include <tuple>
#include <utility>
#include <vector>

using namespace Slic3r;
using namespace mixed_nozzle_fixtures;
using Slic3r::Emitted::EmittedFacts;
using Slic3r::Emitted::EmittedRoleFact;
using Slic3r::Emitted::extract_emitted_facts;

namespace {

using Move = GCodeProcessorResult::MoveVertex;

EmittedFacts body_facts(Print &print)
{
    const CadenceTest::Facts facts = CadenceTest::slice(print);
    INFO(facts.refusal.string);
    REQUIRE(facts.refusal.string.empty());
    REQUIRE_FALSE(facts.gcode.empty());
    return facts.emitted;
}

std::string body_gcode(Print &print)
{
    const CadenceTest::Facts facts = CadenceTest::slice(print);
    INFO(facts.refusal.string);
    REQUIRE(facts.refusal.string.empty());
    REQUIRE_FALSE(facts.gcode.empty());
    return facts.gcode;
}

DynamicPrintConfig full_coupon_config()
{
    DynamicPrintConfig config = coupon_config(true);
    fill_per_filament_values(config);
    return config;
}

DynamicPrintConfig body_support_config()
{
    DynamicPrintConfig config = full_coupon_config();
    config.set_key_value("nozzle_diameter", new ConfigOptionFloats{0.6, 0.2});
    config.set_key_value("filament_map", new ConfigOptionInts{2, 1});
    config.set_key_value("enable_support", new ConfigOptionBool(true));
    config.set_key_value("support_filament", new ConfigOptionInt(2));
    config.set_key_value("support_interface_filament", new ConfigOptionInt(1));
    config.set_key_value("support_type", new ConfigOptionEnum<SupportType>(stNormalAuto));
    config.set_key_value("support_line_width", new ConfigOptionFloatOrPercent(0., false));
    config.set_key_value("line_width", new ConfigOptionFloatOrPercent(0., false));
    config.set_key_value("independent_support_layer_height", new ConfigOptionBool(false));
    config.set_key_value("filament_start_gcode", new ConfigOptionStrings(2, ""));
    config.set_key_value("filament_end_gcode", new ConfigOptionStrings(2, ""));
    fill_per_filament_values(config);
    return config;
}

DynamicPrintConfig body_raft_config()
{
    DynamicPrintConfig config = coarse_base_support_config(MixedNozzleSlicingMode::BodySplit);
    config.set_key_value("raft_layers", new ConfigOptionInt(4));
    // A raft's base is on the fine nozzle: the coarse one may not lay the print's first layer (SRL-A52).
    config.set_key_value("support_filament", new ConfigOptionInt(1));
    config.set_key_value("raft_contact_distance", new ConfigOptionFloat(0.35));
    config.set_key_value("support_top_z_distance", new ConfigOptionFloat(0.25));
    for (const char *key : {"outer_wall_filament_id", "inner_wall_filament_id",
                            "sparse_infill_filament_id", "internal_solid_filament_id",
                            "top_surface_filament_id", "bottom_surface_filament_id"})
        config.set_key_value(key, new ConfigOptionInt(0));
    config.set_key_value("min_layer_height", new ConfigOptionFloats{0.04, 0.08});
    config.set_key_value("wipe_tower_type", new ConfigOptionEnum<WipeTowerType>(WipeTowerType::Type1));
    config.set_key_value("before_layer_change_gcode", new ConfigOptionString("G92 E0\n"));
    fill_per_filament_values(config);
    return config;
}

std::pair<double, double> support_gap(const std::vector<Move> &moves, double x_lo, double x_hi)
{
    double object_bottom = std::numeric_limits<double>::max();
    for (const Move &move : moves)
        if (CadenceTest::model_road(move) && move.position.x() > x_lo && move.position.x() < x_hi)
            object_bottom = std::min(object_bottom, double(move.position.z()) - double(move.height));
    double support_top = 0.;
    for (const Move &move : moves)
        if (move.type == EMoveType::Extrude &&
            (move.extrusion_role == erSupportMaterial || move.extrusion_role == erSupportMaterialInterface ||
             move.extrusion_role == erSupportTransition) &&
            move.position.x() > x_lo && move.position.x() < x_hi &&
            double(move.position.z()) < object_bottom + 1e-4)
            support_top = std::max(support_top, double(move.position.z()));
    return {support_top, object_bottom};
}

} // namespace

TEST_CASE("Body Split emits separate fine and coarse grids on their physical nozzles", "[TestRebuild][BodySplit]")
{
    Model model;
    Print print;
    const DynamicPrintConfig config = full_coupon_config();
    init_coupon(model, print, config);
    declare_native_cadences(model, print, config);
    const EmittedFacts facts = body_facts(print);

    REQUIRE_FALSE(facts.plan_malformed);
    CHECK(facts.plan_start_markers == 1);
    CHECK(facts.plan_end_markers == 1);
    REQUIRE(facts.plan_grids.size() == 2);
    std::set<unsigned> grid_tools;
    std::set<int> grid_heights_um;
    for (const auto &grid : facts.plan_grids) {
        grid_tools.insert(grid.tool);
        grid_heights_um.insert(int(std::lround(grid.cadence * 1000.)));
        CHECK(grid.cells > 0);
        CHECK(grid.top_z >= 3.9);
    }
    CHECK(grid_tools == std::set<unsigned>{0, 1});
    CHECK(grid_heights_um == std::set<int>{100, 200});

    size_t fine_roads = 0, coarse_roads = 0, wrong_height = 0, empty_flow = 0;
    std::set<unsigned> coarse_layers;
    for (const EmittedRoleFact &fact : facts.role_facts) {
        if (fact.role == erWipeTower || fact.role == erSupportMaterial ||
            fact.role == erSupportMaterialInterface || fact.role == erSkirt || fact.role == erBrim)
            continue;
        if (fact.physical_tool == 0) {
            fine_roads += fact.move_count;
            wrong_height += std::abs(fact.height - 0.10f) > 1e-3f ? fact.move_count : 0;
        } else if (fact.physical_tool == 1) {
            coarse_roads += fact.move_count;
            coarse_layers.insert(fact.layer_id);
            // Both bodies share a 0.10 mm first layer.
            const bool legal = std::abs(fact.height - 0.20f) <= 1e-3f ||
                               (fact.layer_id == 0 && std::abs(fact.height - 0.10f) <= 1e-3f);
            wrong_height += legal ? 0 : fact.move_count;
        }
        empty_flow += fact.mm3_per_mm <= 0.f ? fact.move_count : 0;
    }
    CAPTURE(fine_roads, coarse_roads, wrong_height, empty_flow, coarse_layers.size());
    CHECK(fine_roads > 0);
    CHECK(coarse_roads > 0);
    CHECK(coarse_layers.size() == 20);
    const bool ends_on_coarse = !facts.role_facts.empty() && facts.role_facts.back().physical_tool == 1;
    CHECK(facts.tool_changes.size() == 2 * coarse_layers.size() - (ends_on_coarse ? 1 : 0));
    CHECK(wrong_height == 0);
    CHECK(empty_flow == 0);
    REQUIRE(facts.plan_toolchanges_total.has_value());
    CHECK(*facts.plan_toolchanges_total == int(facts.tool_changes.size()));
    CHECK_FALSE(facts.tool_changes.empty());
}

TEST_CASE("Body Split keeps physical ownership when the nozzle slots are reversed", "[TestRebuild][BodySplit]")
{
    DynamicPrintConfig config = full_coupon_config();
    config.set_key_value("nozzle_diameter", new ConfigOptionFloats{0.4, 0.2});
    config.set_key_value("filament_map", new ConfigOptionInts{2, 1});
    fill_per_filament_values(config);
    Model model;
    Print print;
    init_mapped_coupon(model, print, config, 1, 2);
    declare_native_cadences(model, print, config);
    const EmittedFacts facts = body_facts(print);

    std::set<unsigned> fine_tools, coarse_tools;
    size_t fine_roads = 0, coarse_roads = 0;
    for (const EmittedRoleFact &fact : facts.role_facts) {
        if (fact.role == erWipeTower || fact.role == erSupportMaterial ||
            fact.role == erSupportMaterialInterface || fact.role == erSkirt || fact.role == erBrim)
            continue;
        if (fact.logical_filament == 0) {
            fine_tools.insert(fact.physical_tool);
            fine_roads += fact.move_count;
        } else if (fact.logical_filament == 1) {
            coarse_tools.insert(fact.physical_tool);
            coarse_roads += fact.move_count;
        }
    }
    CHECK(fine_roads > 0);
    CHECK(coarse_roads > 0);
    CHECK(fine_tools == std::set<unsigned>{1});
    CHECK(coarse_tools == std::set<unsigned>{0});
}

TEST_CASE("Body Split painting preserves a shared-nozzle colour without extra physical changes", "[TestRebuild][BodySplit]")
{
    DynamicPrintConfig config = coupon_config_with_filaments(3);
    config.set_key_value("nozzle_diameter", new ConfigOptionFloats{0.2, 0.6});
    config.set_key_value("filament_map", new ConfigOptionInts{1, 2, 1});
    config.set_key_value("filament_colour", new ConfigOptionStrings{"#FF0000", "#0000FF", "#00FF00"});
    config.set_key_value("flush_multiplier", new ConfigOptionFloats{1., 1.});
    fill_per_filament_values(config);
    const std::vector<int> side{4, 5};

    Model control_model;
    Print control_print;
    init_paint_cadence_coupon(control_model, control_print, config, 2, std::nullopt, std::nullopt, side);
    const EmittedFacts control = body_facts(control_print);
    Model painted_model;
    Print painted_print;
    init_paint_cadence_coupon(painted_model, painted_print, config, 2,
                              EnforcerBlockerType::Extruder3, std::nullopt, side);
    const EmittedFacts painted = body_facts(painted_print);

    size_t painted_roads = 0, coarse_roads = 0;
    for (const EmittedRoleFact &fact : painted.role_facts) {
        if (fact.logical_filament == 2) {
            painted_roads += fact.move_count;
            CHECK(fact.physical_tool == 0);
        }
        if (fact.logical_filament == 1 && fact.role != erWipeTower) {
            coarse_roads += fact.move_count;
            CHECK(fact.physical_tool == 1);
        }
    }
    CHECK(painted_roads > 0);
    CHECK(coarse_roads > 0);
    REQUIRE_FALSE(control.tool_changes.empty());
    CHECK(painted.tool_changes.size() == control.tool_changes.size());
    CHECK_FALSE(painted.plan_malformed);
}

TEST_CASE("Body Split deposits paint across both nozzles on one two-body object", "[TestRebuild][BodySplit]")
{
    const bool top_paint = GENERATE(false, true);
    const std::vector<int> facets = top_paint ? std::vector<int>{2, 3} : std::vector<int>{4, 5};
    DynamicPrintConfig config = coupon_config_with_filaments(3);
    config.set_key_value("nozzle_diameter", new ConfigOptionFloats{0.2, 0.6});
    config.set_key_value("filament_map", new ConfigOptionInts{1, 2, 1});
    config.set_key_value("filament_colour", new ConfigOptionStrings{"#FF0000", "#0000FF", "#00FF00"});
    fill_per_filament_values(config);
    Model model;
    Print print;
    init_paint_cadence_coupon(model, print, config, 2, EnforcerBlockerType::Extruder2,
                              EnforcerBlockerType::Extruder1, facets);
    const EmittedFacts facts = body_facts(print);
    const double left = model.objects.front()->instance_bounding_box(0).min.x();
    size_t coarse_on_fine = 0, fine_on_coarse = 0;
    for (const EmittedRoleFact &fact : facts.role_facts) {
        if (fact.role == erWipeTower || fact.role == erSkirt || fact.role == erBrim)
            continue;
        if (fact.physical_tool == 1 && fact.min_x < left + 9.)
            coarse_on_fine += fact.move_count;
        if (fact.physical_tool == 0 && fact.max_x > left + 13.)
            fine_on_coarse += fact.move_count;
    }
    CAPTURE(top_paint, coarse_on_fine, fine_on_coarse);
    CHECK(coarse_on_fine > 0);
    CHECK(fine_on_coarse > 0);
    CHECK_FALSE(facts.plan_malformed);
    CHECK(facts.plan_grids.size() >= 4);
}

TEST_CASE("An untouched Body Split destination keeps incoming paint and self-targeted bodies",
          "[TestRebuild][BodySplit]")
{
    DynamicPrintConfig config = coupon_config_with_filaments(3);
    config.set_key_value("nozzle_diameter", new ConfigOptionFloats{0.2, 0.6});
    config.set_key_value("filament_map", new ConfigOptionInts{1, 2, 1});
    fill_per_filament_values(config);

    const auto count_regions = [](const CadenceTest::Facts &facts, double left) {
        std::array<size_t, 3> roads{}; // fine on fine, coarse on fine, coarse on coarse
        for (const Move &move : facts.moves) {
            if (!CadenceTest::model_road(move))
                continue;
            const double x = double(move.position.x()) - left;
            if (x > 1. && x < 9.) {
                if (move.physical_tool_id == 0) ++roads[0];
                if (move.physical_tool_id == 1) ++roads[1];
            } else if (x > 13. && x < 21. && move.physical_tool_id == 1)
                ++roads[2];
        }
        return roads;
    };

    Model control_model;
    Print control_print;
    init_paint_cadence_coupon(control_model, control_print, config, 2,
                              std::nullopt, std::nullopt, {4, 5});
    const CadenceTest::Facts control = CadenceTest::slice(control_print);
    INFO(control.refusal.string);
    REQUIRE(control.refusal.string.empty());
    const auto base = count_regions(control, control_model.objects.front()->instance_bounding_box(0).min.x());
    REQUIRE(base[0] > 0);
    REQUIRE(base[2] > 0);

    for (const auto target : {EnforcerBlockerType::Extruder2, EnforcerBlockerType::Extruder1}) {
        Model model;
        Print print;
        init_paint_cadence_coupon(model, print, config, 2, target, std::nullopt, {4, 5});
        const CadenceTest::Facts facts = CadenceTest::slice(print);
        INFO(facts.refusal.string);
        REQUIRE(facts.refusal.string.empty());
        const auto roads = count_regions(facts, model.objects.front()->instance_bounding_box(0).min.x());
        CAPTURE(int(target), roads[0], roads[1], roads[2], base[2]);
        CHECK(roads[0] > 0);
        CHECK(roads[2] >= base[2] * 9 / 10);
        if (target == EnforcerBlockerType::Extruder2)
            CHECK(roads[1] > 0);
        else
            CHECK(roads[1] == 0);
        CHECK_FALSE(facts.emitted.plan_malformed);
    }
}

TEST_CASE("Side paint on a coarse body adds fine shell planes while coarse roads remain", "[TestRebuild][BodySplit]")
{
    DynamicPrintConfig config = coupon_config_with_filaments(3);
    config.set_key_value("nozzle_diameter", new ConfigOptionFloats{0.2, 0.6});
    config.set_key_value("filament_map", new ConfigOptionInts{1, 1, 2});
    fill_per_filament_values(config);
    const auto make_single_body = [&](bool paint, Model &model, Print &print) {
        ModelObject *object = model.add_object();
        object->name = "painted-coarse-body";
        ModelVolume *coarse = object->add_volume(make_cube(20., 20., 4.), ModelVolumeType::MODEL_PART, false);
        coarse->config.set_key_value("extruder", new ConfigOptionInt(3));
        coarse->config.set_key_value("regional_layer_height", new ConfigOptionFloat(0.20));
        if (paint)
            paint_cadence_facets(*coarse, {4, 5}, EnforcerBlockerType::Extruder1);
        ModelVolume *fine = object->add_volume(make_cube(10., 20., 4.), ModelVolumeType::MODEL_PART, false);
        fine->set_offset(Vec3d(25., 0., 0.));
        fine->config.set_key_value("extruder", new ConfigOptionInt(1));
        fine->config.set_key_value("regional_layer_height", new ConfigOptionFloat(0.10));
        object->add_instance();
        object->ensure_on_bed();
        print.apply(model, config);
        print.set_status_silent();
    };

    Model control_model;
    Print control_print;
    make_single_body(false, control_model, control_print);
    const CadenceTest::Facts control = CadenceTest::slice(control_print);
    INFO(control.refusal.string);
    REQUIRE(control.refusal.string.empty());
    Model painted_model;
    Print painted_print;
    make_single_body(true, painted_model, painted_print);
    const CadenceTest::Facts painted = CadenceTest::slice(painted_print);
    INFO(painted.refusal.string);
    REQUIRE(painted.refusal.string.empty());

    std::set<int> control_planes, painted_planes, added_fine_planes;
    size_t fine_roads = 0, coarse_roads = 0;
    const double left = painted_model.objects.front()->instance_bounding_box(0).min.x();
    for (const Move &move : control.moves)
        if (CadenceTest::model_road(move) && move.position.x() > left + 1. && move.position.x() < left + 19.)
            control_planes.insert(int(std::lround(double(move.position.z()) * 1000.)));
    for (const Move &move : painted.moves) {
        if (!CadenceTest::model_road(move) || move.position.x() <= left + 1. ||
            move.position.x() >= left + 19.)
            continue;
        const int z = int(std::lround(double(move.position.z()) * 1000.));
        painted_planes.insert(z);
        if (move.physical_tool_id == 0) {
            ++fine_roads;
            if (control_planes.count(z) == 0)
                added_fine_planes.insert(z);
        } else if (move.physical_tool_id == 1)
            ++coarse_roads;
    }
    CAPTURE(fine_roads, coarse_roads, control_planes.size(), painted_planes.size(), added_fine_planes.size());
    CHECK(fine_roads > 0);
    CHECK(coarse_roads > 0);
    CHECK(painted_planes.size() > control_planes.size());
    CHECK_FALSE(added_fine_planes.empty());
}

// Fine filament painted onto a coarse body that is too thick for the fine nozzle (0.30 mm against the
// 0.2 nozzle's 0.14 mm limit). Admission took the painted area's height from the body and refused it;
// slicing prints it at the fine height. Alone, and beside a separate fine part.
TEST_CASE("Fine paint on a coarse body is admitted and printed at the fine layer height", "[TestRebuild][BodySplit]")
{
    const bool with_fine_part = GENERATE(false, true);
    CAPTURE(with_fine_part);
    DynamicPrintConfig config = coupon_config_with_filaments(2);
    config.set_key_value("nozzle_diameter", new ConfigOptionFloats{0.2, 0.6});
    config.set_key_value("max_layer_height", new ConfigOptionFloats{0.14, 0.42});
    config.set_key_value("mixed_nozzle_allowed_cadence_ratios", new ConfigOptionInts{3});
    fill_per_filament_values(config);
    Model model;
    Print print;
    ModelObject *object = model.add_object();
    object->name = "fine-paint-on-coarse-body";
    ModelVolume *coarse = object->add_volume(make_cube(20., 20., 4.), ModelVolumeType::MODEL_PART, false);
    coarse->config.set_key_value("extruder", new ConfigOptionInt(2));
    coarse->config.set_key_value("regional_layer_height", new ConfigOptionFloat(0.30));
    paint_cadence_facets(*coarse, {4, 5}, EnforcerBlockerType::Extruder1);
    if (with_fine_part) {
        ModelVolume *fine = object->add_volume(make_cube(6., 20., 4.), ModelVolumeType::MODEL_PART, false);
        fine->set_offset(Vec3d(25., 0., 0.));
        fine->config.set_key_value("extruder", new ConfigOptionInt(1));
        fine->config.set_key_value("regional_layer_height", new ConfigOptionFloat(0.10));
    }
    object->add_instance();
    object->ensure_on_bed();
    print.apply(model, config);
    print.set_status_silent();
    const CadenceTest::Facts facts = CadenceTest::slice(print);
    INFO(facts.refusal.string);
    REQUIRE(facts.refusal.string.empty());
    REQUIRE_FALSE(facts.gcode.empty());

    // Over the painted body: fine roads, at most 0.10 mm thick, and coarse roads.
    const double left = model.objects.front()->instance_bounding_box(0).min.x();
    size_t fine_roads = 0, coarse_roads = 0;
    double thickest_fine = 0.;
    for (const Move &move : facts.moves) {
        if (!CadenceTest::model_road(move) || move.position.x() <= left + 0.5 || move.position.x() >= left + 19.5)
            continue;
        if (move.physical_tool_id == 0) {
            ++fine_roads;
            thickest_fine = std::max(thickest_fine, double(move.height));
        } else if (move.physical_tool_id == 1)
            ++coarse_roads;
    }
    CAPTURE(fine_roads, coarse_roads, thickest_fine);
    CHECK(fine_roads > 0);
    CHECK(coarse_roads > 0);
    CHECK(thickest_fine <= 0.10 + 1e-3);
    CHECK_FALSE(facts.emitted.plan_malformed);
}

// A text-shaped modifier on the fine filament inside a single coarse part (a flush inlay) was refused
// unless the object had another fine part. Its area is now a second body, printed at
// the fine layer height like fine paint. A modifier that names no filament keeps the part plain.
TEST_CASE("A fine-filament modifier in a single coarse part is a Body Split inlay", "[TestRebuild][BodySplit]")
{
    DynamicPrintConfig config = coupon_config_with_filaments(2);
    config.set_key_value("nozzle_diameter", new ConfigOptionFloats{0.2, 0.6});
    config.set_key_value("max_layer_height", new ConfigOptionFloats{0.14, 0.42});
    config.set_key_value("mixed_nozzle_allowed_cadence_ratios", new ConfigOptionInts{3});
    fill_per_filament_values(config);
    const auto make = [&](bool modifier_filament, Model &model, Print &print) {
        ModelObject *object = model.add_object();
        object->name = "modifier-inlay";
        ModelVolume *coarse = object->add_volume(make_cube(20., 20., 4.), ModelVolumeType::MODEL_PART, false);
        coarse->config.set_key_value("extruder", new ConfigOptionInt(2));
        coarse->config.set_key_value("regional_layer_height", new ConfigOptionFloat(0.30));
        ModelVolume *inlay = object->add_volume(make_cube(8., 3., 0.8), ModelVolumeType::PARAMETER_MODIFIER, false);
        inlay->set_offset(Vec3d(6., 8., 3.2));
        if (modifier_filament)
            inlay->config.set_key_value("extruder", new ConfigOptionInt(1));
        else
            inlay->config.set_key_value("sparse_infill_density", new ConfigOptionPercent(40.));
        object->add_instance();
        object->ensure_on_bed();
        print.apply(model, config);
        print.set_status_silent();
    };

    Model model;
    Print print;
    make(true, model, print);
    const CadenceTest::Facts facts = CadenceTest::slice(print);
    INFO(facts.refusal.string);
    REQUIRE(facts.refusal.string.empty());
    REQUIRE_FALSE(facts.gcode.empty());
    const BoundingBoxf3 box = model.objects.front()->instance_bounding_box(0);
    size_t fine_in_inlay = 0, fine_elsewhere = 0, coarse_roads = 0;
    double thickest_fine = 0., fine_top = 0.;
    Vec3f previous = Vec3f::Zero();
    for (const Move &move : facts.moves) {
        const Vec3f from = previous;
        previous = move.position;
        if (!CadenceTest::model_road(move))
            continue;
        const double x = 0.5 * double(move.position.x() + from.x()) - box.min.x();
        const double y = 0.5 * double(move.position.y() + from.y()) - box.min.y();
        if (move.physical_tool_id == 0) {
            const bool inside = x > 5.5 && x < 14.5 && y > 7.5 && y < 11.5;
            (inside ? fine_in_inlay : fine_elsewhere)++;
            thickest_fine = std::max(thickest_fine, double(move.height));
            fine_top = std::max(fine_top, double(move.position.z()));
        } else if (move.physical_tool_id == 1)
            ++coarse_roads;
    }
    CAPTURE(fine_in_inlay, fine_elsewhere, coarse_roads, thickest_fine, fine_top);
    CHECK(fine_in_inlay > 0);
    CHECK(fine_elsewhere == 0);
    CHECK(coarse_roads > 0);
    CHECK(thickest_fine <= 0.10 + 1e-3);
    CHECK(std::abs(fine_top - 4.) < 1e-3);

    Model plain_model;
    Print plain_print;
    make(false, plain_model, plain_print);
    CHECK_FALSE(is_body_split_object(plain_print.config(), *plain_model.objects.front()));
}

TEST_CASE("Body Split fine skins put a coarse body's roof and pocket floor on the fine nozzle", "[TestRebuild][BodySplit]")
{
    DynamicPrintConfig config = fine_skin_ratio3_config();
    fill_per_filament_values(config);
    Model model;
    Print print;
    init_native_skin_coupon(model, print, config, make_cube(10., 20., 3.),
        box_with_internal_cavity(Vec3d(20., 20., 3.), Vec3d(8.5, 8.5, 1.7), Vec3d(3., 3., 1.)),
        0.10, 0.30);
    const EmittedFacts facts = body_facts(print);

    float coarse_min_x = 1e9f, coarse_top = 0.f;
    size_t coarse_roads = 0;
    for (const EmittedRoleFact &fact : facts.role_facts)
        if (fact.physical_tool == 1 && fact.role != erWipeTower) {
            coarse_min_x = std::min(coarse_min_x, fact.min_x);
            coarse_top = std::max(coarse_top, fact.z);
            coarse_roads += fact.move_count;
        }
    REQUIRE(coarse_roads > 0);
    size_t roof = 0, floor = 0, wrong_height = 0;
    for (const EmittedRoleFact &fact : facts.role_facts) {
        if (fact.physical_tool != 0 || fact.max_x <= coarse_min_x + 1.f)
            continue;
        wrong_height += std::abs(fact.height - 0.10f) > 1e-3f ? fact.move_count : 0;
        if (fact.z > 2.7f + 1e-3f)
            roof += fact.move_count;
        else if (fact.z > 1.4f + 1e-3f && fact.z <= 1.7f + 1e-3f)
            floor += fact.move_count;
    }
    CAPTURE(roof, floor, coarse_top, wrong_height);
    CHECK(coarse_top <= 2.5f + 1e-3f);
    CHECK(roof > 0);
    CHECK(floor > 0);
    CHECK(wrong_height == 0);

    // A separate stacked coupon exercises the exposed half of a coarse body's vertical-shell
    // window. The top at 1.10 is three 0.20 mm coarse cells above the cell ending at 0.70;
    // counting fine 0.10 mm event rows would leave that exposed half sparse.
    DynamicPrintConfig shell_config = coupon_config(true);
    shell_config.set_key_value("ensure_vertical_shell_thickness",
                               new ConfigOptionEnum<EnsureVerticalShellThickness>(evstAll));
    shell_config.set_key_value("bottom_shell_layers", new ConfigOptionInt(1));
    fill_per_filament_values(shell_config);
    Model shell_model;
    ModelObject *stack = shell_model.add_object();
    stack->name = "coarse-shell-window";
    ModelVolume *base = stack->add_volume(make_cube(20., 10., 1.10), ModelVolumeType::MODEL_PART, false);
    base->config.set_key_value("extruder", new ConfigOptionInt(2));
    base->config.set_key_value("regional_layer_height", new ConfigOptionFloat(0.20));
    ModelVolume *upper = stack->add_volume(make_cube(10., 10., 0.90), ModelVolumeType::MODEL_PART, false);
    upper->set_offset(Vec3d(0., 0., 1.10));
    upper->config.set_key_value("extruder", new ConfigOptionInt(1));
    upper->config.set_key_value("regional_layer_height", new ConfigOptionFloat(0.10));
    stack->add_instance();
    stack->ensure_on_bed();
    Print shell_print;
    shell_print.apply(shell_model, shell_config);
    shell_print.set_status_silent();
    const CadenceTest::Facts shell = CadenceTest::slice(shell_print);
    INFO(shell.refusal.string);
    REQUIRE(shell.refusal.string.empty());
    const double shell_left = stack->instance_bounding_box(0).min.x();
    const double shell_front = stack->instance_bounding_box(0).min.y();
    size_t solid_exposed = 0, sparse_exposed = 0, solid_under_upper = 0, lower_sparse = 0;
    size_t shell_coarse_total = 0;
    double shell_min_y = 1e9, shell_max_y = -1e9;
    std::map<int, size_t> shell_z_levels;
    std::map<int, size_t> shell_model_tools;
    double shell_model_min_y = 1e9, shell_model_max_y = -1e9;
    // Infill roads run edge to edge, so their end points sit on the outline. Test the whole
    // road, from the previous position, against each window.
    const auto crosses = [&](size_t index, double x_min, double x_max) {
        const Vec3f &end = shell.moves[index].position;
        const Vec3f &start = index > 0 ? shell.moves[index - 1].position : end;
        for (int step = 0; step <= 40; ++step) {
            const double t = step / 40.;
            const double x = double(start.x()) + t * (double(end.x()) - double(start.x())) - shell_left;
            const double y = double(start.y()) + t * (double(end.y()) - double(start.y())) - shell_front;
            if (x > x_min && x < x_max && y > 2. && y < 8.)
                return true;
        }
        return false;
    };
    for (size_t index = 0; index < shell.moves.size(); ++index) {
        const Move &move = shell.moves[index];
        if (!CadenceTest::model_road(move))
            continue;
        ++shell_model_tools[int(move.physical_tool_id)];
        shell_model_min_y = std::min(shell_model_min_y, double(move.position.y()));
        shell_model_max_y = std::max(shell_model_max_y, double(move.position.y()));
        if (move.physical_tool_id != 1)
            continue;
        ++shell_coarse_total;
        shell_min_y = std::min(shell_min_y, double(move.position.y()));
        shell_max_y = std::max(shell_max_y, double(move.position.y()));
        ++shell_z_levels[int(std::lround(double(move.position.z()) * 1000.))];
        // The first solid cell over sparse infill prints as an internal bridge.
        const bool solid = move.extrusion_role == erSolidInfill || move.extrusion_role == erInternalBridgeInfill;
        if (std::abs(double(move.position.z()) - 0.70) < 1e-3) {
            if (crosses(index, 12., 18.)) {
                solid_exposed += solid;
                sparse_exposed += move.extrusion_role == erInternalInfill;
            }
            if (crosses(index, 2., 8.))
                solid_under_upper += solid;
        } else if (std::abs(double(move.position.z()) - 0.30) < 1e-3 && crosses(index, 12., 18.))
            lower_sparse += move.extrusion_role == erInternalInfill;
    }
    // Per coarse level: x range, then role counts in the exposed and covered halves.
    std::ostringstream shell_levels;
    for (const auto &[z, count] : shell_z_levels) {
        double lo = 1e9, hi = -1e9;
        std::map<int, size_t> exposed_roles, covered_roles;
        for (size_t index = 0; index < shell.moves.size(); ++index) {
            const Move &move = shell.moves[index];
            if (!CadenceTest::model_road(move) || move.physical_tool_id != 1 ||
                int(std::lround(double(move.position.z()) * 1000.)) != z)
                continue;
            const double x = double(move.position.x()) - shell_left;
            lo = std::min(lo, x);
            hi = std::max(hi, x);
            if (crosses(index, 12., 18.))
                ++exposed_roles[int(move.extrusion_role)];
            if (crosses(index, 2., 8.))
                ++covered_roles[int(move.extrusion_role)];
        }
        shell_levels << "z=" << z << " n=" << count << " x=[" << lo << "," << hi << "] exposed{";
        for (const auto &[role, n] : exposed_roles)
            shell_levels << ' ' << role << ':' << n;
        shell_levels << " } covered{";
        for (const auto &[role, n] : covered_roles)
            shell_levels << ' ' << role << ':' << n;
        shell_levels << " }\n";
    }
    const std::string shell_histogram = shell_levels.str();
    CAPTURE(solid_exposed, sparse_exposed, solid_under_upper, lower_sparse,
            shell_coarse_total, shell_min_y, shell_max_y, shell_histogram,
            shell_model_tools.size(), shell_model_min_y, shell_model_max_y, shell_left, shell_front);
    CHECK(solid_exposed > 0);
    CHECK(sparse_exposed == 0);
    CHECK(solid_under_upper > 0);
    CHECK(lower_sparse > 0);
}

TEST_CASE("Body Split beams cross a synthetic raised joint in both directions", "[TestRebuild][BodySplit]")
{
    constexpr double joint_x = 20.;
    constexpr double strip_floor = 1.30;
    constexpr double top = 2.18;
    Model model;
    ModelObject *object = model.add_object();
    object->name = "synthetic-rebated-joint";
    ModelVolume *plate = object->add_volume(extruded_xz_outline(
        {{0., 0.}, {26., 0.}, {26., strip_floor}, {joint_x, strip_floor},
         {joint_x, top}, {top, top}}, 16.), ModelVolumeType::MODEL_PART, false);
    plate->config.set_key_value("extruder", new ConfigOptionInt(2));
    plate->config.set_key_value("regional_layer_height", new ConfigOptionFloat(0.24));
    ModelVolume *strip = object->add_volume(extruded_xz_outline(
        {{joint_x, strip_floor}, {26., strip_floor}, {25.3, top}, {joint_x, top}}, 16.),
        ModelVolumeType::MODEL_PART, false);
    strip->config.set_key_value("extruder", new ConfigOptionInt(1));
    strip->config.set_key_value("regional_layer_height", new ConfigOptionFloat(0.08));
    object->config.set_key_value("interlocking_beam", new ConfigOptionBool(true));
    object->config.set_key_value("interlocking_beam_layer_count", new ConfigOptionInt(3));
    object->config.set_key_value("interlocking_beam_width", new ConfigOptionFloat(0.8));
    object->config.set_key_value("interlocking_depth", new ConfigOptionInt(2));
    object->config.set_key_value("interlocking_boundary_avoidance", new ConfigOptionInt(2));
    object->config.set_key_value("interlocking_orientation", new ConfigOptionFloat(22.5));
    object->add_instance();
    object->ensure_on_bed();

    DynamicPrintConfig config = full_coupon_config();
    config.set_key_value("layer_height", new ConfigOptionFloat(0.08));
    config.set_key_value("initial_layer_print_height", new ConfigOptionFloat(0.10));
    config.set_key_value("max_layer_height", new ConfigOptionFloats{0.14, 0.28});
    config.set_key_value("mixed_nozzle_allowed_cadence_ratios", new ConfigOptionInts{2, 3});
    fill_per_filament_values(config);
    Print print;
    print.apply(model, config);
    print.set_status_silent();
    const std::string gcode = body_gcode(print);
    const EmittedFacts facts = extract_emitted_facts(gcode);
    CHECK_FALSE(facts.plan_malformed);
    REQUIRE_FALSE(facts.tool_changes.empty());

    const double left = model.objects.front()->instance_bounding_box(0).min.x();
    size_t fine_into_plate = 0, coarse_into_strip = 0;
    for (const Move &move : Slic3r::Emitted::emitted_moves(gcode)) {
        if (!CadenceTest::model_road(move))
            continue;
        const double z = double(move.position.z());
        const double x = double(move.position.x()) - left;
        if (z <= strip_floor + 1e-3 || z > top + 1e-3)
            continue;
        fine_into_plate += move.physical_tool_id == 0 && x < joint_x - 0.3 ? 1 : 0;
        coarse_into_strip += move.physical_tool_id == 1 && x > joint_x + 0.3 ? 1 : 0;
    }
    CAPTURE(fine_into_plate, coarse_into_strip);
    CHECK(fine_into_plate > 0);
    CHECK(coarse_into_strip > 0);
}

// Two fine parts on one filament, on top of and beside a coarse base. Only the last part's slices
// used to reach the shared fine region, so with the side part last the base's top was planned as if
// nothing sat on it: under the top part the base stopped one coarse layer short, the base rose past
// its top elsewhere, and the top part started over a gap. Both part orders must print the same.
TEST_CASE("Body Split prints several fine parts on one filament the same in any order, on top of the base",
          "[TestRebuild][BodySplit]")
{
    DynamicPrintConfig config = coupon_config_with_filaments(2);
    config.set_key_value("nozzle_diameter", new ConfigOptionFloats{0.2, 0.6});
    config.set_key_value("max_layer_height", new ConfigOptionFloats{0.20, 0.42});
    config.set_key_value("mixed_nozzle_allowed_cadence_ratios", new ConfigOptionInts{3});
    fill_per_filament_values(config);
    // The coarse layers end at 2.8 and 3.1, so the base top at 3.0 has to come from the parts above it.
    constexpr double base_top = 3.;
    struct Result { CadenceTest::Facts facts; Vec2d base_origin; };
    const auto slice_order = [&](bool side_last) {
        Model model;
        Print print;
        ModelObject *object = model.add_object();
        object->name = side_last ? "side-part-last" : "top-part-last";
        ModelVolume *base = object->add_volume(make_cube(20., 20., base_top), ModelVolumeType::MODEL_PART, false);
        base->config.set_key_value("extruder", new ConfigOptionInt(2));
        base->config.set_key_value("regional_layer_height", new ConfigOptionFloat(0.30));
        const auto add_fine = [&](const char *name, const Vec3d &size, const Vec3d &offset) {
            ModelVolume *part = object->add_volume(make_cube(size.x(), size.y(), size.z()), ModelVolumeType::MODEL_PART, false);
            part->name = name;
            part->set_offset(offset);
            part->config.set_key_value("extruder", new ConfigOptionInt(1));
            part->config.set_key_value("regional_layer_height", new ConfigOptionFloat(0.10));
        };
        const auto add_top  = [&] { add_fine("top", Vec3d(8., 8., 1.), Vec3d(4., 4., base_top)); };
        const auto add_side = [&] { add_fine("side", Vec3d(6., 1., 1.6), Vec3d(7., -1., 0.6)); };
        if (side_last) { add_top(); add_side(); } else { add_side(); add_top(); }
        object->add_instance();
        object->ensure_on_bed();
        print.apply(model, config);
        print.set_status_silent();
        Result result{CadenceTest::slice(print), Vec2d::Zero()};
        INFO(result.facts.refusal.string);
        REQUIRE(result.facts.refusal.string.empty());
        REQUIRE_FALSE(result.facts.gcode.empty());
        // The side part sticks out 1 mm in front of the base.
        const BoundingBoxf3 box = model.objects.front()->instance_bounding_box(0);
        result.base_origin = Vec2d(box.min.x(), box.min.y() + 1.);
        return result;
    };

    struct Summary {
        std::map<std::tuple<int, int, int>, long> lengths; // per layer, tool and role, tenths of a mm
        double coarse_top = 0.;                            // highest coarse road anywhere
        double coarse_top_under = 0.;                      // highest coarse road under the top part
        double fine_bottom_over = 1e9;                     // lowest fine road over the top part
    };
    const auto summarize = [](const Result &result) {
        Summary out;
        Vec3f previous = Vec3f::Zero();
        for (const Move &move : result.facts.moves) {
            const Vec3f from = previous;
            previous = move.position;
            if (!CadenceTest::model_road(move))
                continue;
            const double z = double(move.position.z());
            // The road's midpoint, against the base's own corner.
            const double x = 0.5 * double(move.position.x() + from.x()) - result.base_origin.x();
            const double y = 0.5 * double(move.position.y() + from.y()) - result.base_origin.y();
            const bool under_top = x > 5. && x < 11. && y > 5. && y < 11.;
            if (move.physical_tool_id == 1) {
                out.coarse_top = std::max(out.coarse_top, z);
                if (under_top)
                    out.coarse_top_under = std::max(out.coarse_top_under, z);
            } else if (under_top && z > 2.)
                out.fine_bottom_over = std::min(out.fine_bottom_over, z);
            out.lengths[{int(std::lround(z * 1000.)), int(move.physical_tool_id), int(move.extrusion_role)}] +=
                std::lround(10. * double((move.position.head<2>() - from.head<2>()).norm()));
        }
        return out;
    };
    const Summary side_last = summarize(slice_order(true));
    const Summary top_last  = summarize(slice_order(false));
    for (const Summary *summary : {&side_last, &top_last}) {
        CAPTURE(summary == &side_last, summary->coarse_top, summary->coarse_top_under, summary->fine_bottom_over);
        // The base ends at its own top everywhere, and the top part starts right on it.
        CHECK(std::abs(summary->coarse_top - base_top) < 1e-3);
        CHECK(std::abs(summary->coarse_top_under - base_top) < 1e-3);
        CHECK(std::abs(summary->fine_bottom_over - (base_top + 0.1)) < 1e-3);
    }
    // The same layers, tools and features in both orders, with the same path lengths to within 1%.
    std::set<std::tuple<int, int, int>> keys;
    for (const auto &entry : side_last.lengths) keys.insert(entry.first);
    for (const auto &entry : top_last.lengths) keys.insert(entry.first);
    size_t differing = 0;
    for (const auto &key : keys) {
        const long a = side_last.lengths.count(key) ? side_last.lengths.at(key) : 0;
        const long b = top_last.lengths.count(key) ? top_last.lengths.at(key) : 0;
        if (std::abs(a - b) > std::max(2L, std::max(a, b) / 100))
            ++differing;
    }
    CAPTURE(keys.size(), side_last.lengths.size(), top_last.lengths.size());
    CHECK(differing == 0);
}

TEST_CASE("Body Split tower deposits at the planned footprint and stays on its physical tools", "[TestRebuild][BodySplit]")
{
    DynamicPrintConfig config = body_split_pad_config(.20, .40, .10, .20, false, true, 60., wtwRib);
    apply_tower_exit_retraction(config);
    fill_per_filament_values(config);
    Model model;
    Print print;
    config.set_key_value("enable_arc_fitting", new ConfigOptionBool(true));
    init_body_split_pad_coupon(model, print, config, false, 10., .10, .20);
    REQUIRE(print.has_wipe_tower());
    const std::string gcode = body_gcode(print);
    const EmittedFacts facts = extract_emitted_facts(gcode);
    REQUIRE(facts.plan_toolchanges_total.has_value());
    CHECK(*facts.plan_toolchanges_total == int(facts.tool_changes.size()));
    CHECK_FALSE(facts.tool_changes.empty());
    const TowerFootprintAudit footprint = audit_tower_footprint(gcode, print);
    CHECK(footprint.tower_roads > 0);
    CHECK(footprint.outside == 0);
    const TowerFlowAudit flow = audit_coarse_tower_flow(gcode, print.config());
    CAPTURE(flow.coarse_grid_roads, flow.coarse_wall_arcs, flow.thinnest_ratio);
    REQUIRE(flow.coarse_grid_roads + flow.coarse_wall_arcs > 0);
    CHECK(flow.thin_grid == 0);
    CHECK(flow.thin_wall == 0);
    const TowerGridSpeedAudit speed = audit_tower_grid_speed(gcode);
    CHECK(speed.grid_roads > 0);
    CHECK(speed.faster_roads == 0);
    const TowerExitAudit exit = audit_tower_exit(gcode, 0.7);
    CHECK(exit.returns > 0);
    CHECK(exit.lifted_before_pull == 0);
    CHECK(exit.moves_off_rows == 0);
    CHECK(exit.returns_without_wipe == 0);
    CHECK(exit.unbalanced == 0);
    CHECK(exit.not_lifted_next == 0);
    CHECK(exit.crossings == 0);
}

TEST_CASE("Two coarse colours build Body Split tower levels at the coarse height after fine roads", "[TestRebuild][BodySplit]")
{
    DynamicPrintConfig config = paint_cadence_tower_config();
    config.set_key_value("nozzle_diameter", new ConfigOptionFloats{0.2, 0.6});
    config.set_key_value("min_layer_height", new ConfigOptionFloats{0.04, 0.12});
    config.set_key_value("max_layer_height", new ConfigOptionFloats{0.14, 0.42});
    fill_per_filament_values(config);
    Model model;
    Print print;
    init_paint_cadence_coupon(model, print, config, 2, std::nullopt, std::nullopt,
                              {}, 3, true);
    REQUIRE(print.has_wipe_tower());
    const std::string gcode = body_gcode(print);
    const EmittedFacts facts = extract_emitted_facts(gcode);
    CHECK_FALSE(facts.plan_malformed);

    const std::vector<Move> moves = Slic3r::Emitted::emitted_moves(gcode);
    std::set<unsigned> grid_lines;
    std::istringstream tower_stream(gcode);
    std::string tower_line;
    bool grid = false;
    unsigned line_number = 0;
    while (std::getline(tower_stream, tower_line)) {
        ++line_number;
        if (tower_line.find("CP EMPTY GRID START") != std::string::npos) grid = true;
        if (tower_line.find("CP EMPTY GRID END") != std::string::npos) grid = false;
        if (grid) grid_lines.insert(line_number);
    }
    std::map<int, std::set<unsigned>> model_tools_at_z;
    std::map<int, std::set<unsigned>> coarse_tower_filaments_at_z;
    size_t coarse_tower_roads = 0, wrong_coarse_height = 0, empty_coarse_flow = 0;
    for (const Move &move : moves) {
        if (move.type != EMoveType::Extrude)
            continue;
        const int z = int(std::lround(double(move.position.z()) * 1000.));
        if (CadenceTest::model_road(move) && move.physical_tool_id <= 1)
            model_tools_at_z[z].insert(move.physical_tool_id);
        if (move.extrusion_role != erWipeTower || move.physical_tool_id != 1 || z <= 100 || grid_lines.count(move.gcode_id) == 0)
            continue;
        ++coarse_tower_roads;
        coarse_tower_filaments_at_z[z].insert(move.extruder_id);
        // The fine tool lays the tower's bed level on layer 1, so the coarse steps up to the second
        // coarse layer (0.4 mm) cannot be the 0.2 mm grid: the first is the coarse minimum.
        if (z > 400)
            wrong_coarse_height += std::abs(move.height - .20f) > 1e-3f ? 1 : 0;
        else
            wrong_coarse_height += move.height < .12f - 1e-3f || move.height > .20f + 1e-3f ? 1 : 0;
        empty_coarse_flow += move.mm3_per_mm <= 0.f ? 1 : 0;
    }
    REQUIRE(coarse_tower_roads > 0);
    REQUIRE_FALSE(coarse_tower_filaments_at_z.empty());
    std::set<unsigned> coarse_tower_filaments;
    size_t tower_only_levels = 0;
    for (const auto &[z, filaments] : coarse_tower_filaments_at_z) {
        coarse_tower_filaments.insert(filaments.begin(), filaments.end());
        tower_only_levels += model_tools_at_z[z].count(1) == 0 ? 1 : 0;
    }
    CAPTURE(coarse_tower_roads, wrong_coarse_height, empty_coarse_flow,
            tower_only_levels, coarse_tower_filaments_at_z.size());
    CHECK_FALSE(coarse_tower_filaments.empty());
    for (unsigned filament : coarse_tower_filaments)
        CHECK(std::set<unsigned>{1, 2}.count(filament) == 1);
    CHECK(wrong_coarse_height == 0);
    CHECK(empty_coarse_flow == 0);
    // Every coarse tower level corresponds to a real coarse-body deposition level.
    CHECK(tower_only_levels == 0);

    std::set<unsigned> tools_by_filament[3];
    for (const EmittedRoleFact &fact : facts.role_facts)
        if (fact.logical_filament < 3 && fact.physical_tool <= 1)
            tools_by_filament[fact.logical_filament].insert(fact.physical_tool);
    CHECK(tools_by_filament[0] == std::set<unsigned>{0});
    CHECK(tools_by_filament[1] == std::set<unsigned>{1});
    CHECK(tools_by_filament[2] == std::set<unsigned>{1});

    std::map<int, std::vector<unsigned>> model_order;
    for (const Move &move : moves)
        if (CadenceTest::model_road(move) && move.physical_tool_id <= 1)
            model_order[int(std::lround(double(move.position.z()) * 1000.))].push_back(move.physical_tool_id);
    size_t shared_levels = 0;
    for (const auto &[z, tools] : model_order) {
        if (z <= 100 || std::find(tools.begin(), tools.end(), 0u) == tools.end() ||
            std::find(tools.begin(), tools.end(), 1u) == tools.end())
            continue;
        ++shared_levels;
        CHECK(tools.front() == 0);
    }
    CHECK(shared_levels > 0);
}

TEST_CASE("Body Split support roads use their assigned tools and leave an object-grid Z gap", "[TestRebuild][BodySplit]")
{
    DynamicPrintConfig config = body_support_config();
    config.set_key_value("support_top_z_distance", new ConfigOptionFloat(0.25));
    config.set_key_value("support_bottom_z_distance", new ConfigOptionFloat(0.25));
    fill_per_filament_values(config);
    Model model;
    Print print;
    init_native_coupon(model, print, config, make_cube(10., 20., 1.8),
                       overhang_shelf_mesh(12., 20., 0.6, 1.8));
    const StringObjectException validation = print.validate();
    INFO(validation.string);
    REQUIRE(validation.string.empty());
    const std::string gcode = Test::gcode(print);
    REQUIRE_FALSE(gcode.empty());
    const EmittedFacts facts = extract_emitted_facts(gcode);
    size_t base = 0, interface_roads = 0, wrong = 0;
    for (const EmittedRoleFact &fact : facts.role_facts) {
        if (fact.role == erSupportMaterial) {
            base += fact.move_count;
            // The dense base-material layer against the interface is one object layer tall, so the 0.2
            // lays it at its own width.
            wrong += (fact.physical_tool != 0 || std::abs(fact.width - 0.6f) > 1e-3f) &&
                     (fact.physical_tool != 1 || std::abs(fact.width - 0.2f) > 1e-3f) ? fact.move_count : 0;
        } else if (fact.role == erSupportMaterialInterface) {
            interface_roads += fact.move_count;
            wrong += fact.physical_tool != 1 || std::abs(fact.width - 0.2f) > 1e-3f ? fact.move_count : 0;
        }
    }
    CAPTURE(base, interface_roads, wrong);
    CHECK(base > 0);
    CHECK(interface_roads > 0);
    CHECK(wrong == 0);
    REQUIRE(facts.support.has_value());
    CHECK(facts.support->base_tool == 0);
    CHECK(facts.support->interface_tool == 1);
    const double left = model.objects.front()->instance_bounding_box(0).min.x();
    const auto [support_top, object_bottom] = support_gap(Slic3r::Emitted::emitted_moves(gcode), left + 33., left + 37.);
    CAPTURE(support_top, object_bottom);
    REQUIRE(support_top > 0.);
    CHECK_THAT(object_bottom, Catch::Matchers::WithinAbs(0.70, 1e-3));
    CHECK_THAT(object_bottom - support_top, Catch::Matchers::WithinAbs(0.30, 1e-3));
}

// Body Split on a raft used to be planned from the bed while the object stood on the raft. With the
// coarse nozzle's minimum above the fine layer that failed late with internal messages, so every
// Body Split object on a raft is refused at validation (SRL-A54), whatever its nozzles.
TEST_CASE("A rafted Body Split object is refused before slicing", "[TestRebuild][BodySplit][Raft]")
{
    const bool duplicated = GENERATE(false, true);
    CAPTURE(duplicated);
    const DynamicPrintConfig config = body_raft_config();
    Model model;
    Print print;
    print.is_BBL_printer() = false;
    init_native_coupon(model, print, config, make_cube(10., 20., 1.7), make_cube(20., 20., 1.7), 0.10, 0.40);
    if (duplicated) {
        ModelObject *copy = model.add_object(*model.objects.front());
        for (ModelInstance *instance : copy->instances)
            instance->set_offset(instance->get_offset() + Vec3d(60., 0., 0.));
        print.apply(model, config);
    }
    const StringObjectException refusal = print.validate();
    INFO(refusal.string);
    REQUIRE_FALSE(refusal.string.empty());
    CHECK(refusal.string.find("[SRL-A54]") != std::string::npos);
    CHECK(refusal.opt_key == "raft_layers");
}

TEST_CASE("Body Split refuses unsafe settings with stable admission codes", "[TestRebuild][BodySplit]")
{
    struct Refusal {
        const char *name;
        const char *key;
        const char *code;
        void (*change)(DynamicPrintConfig &);
    };
    const Refusal refusals[] = {
        {"hole conversion", "hole_to_polyhole", "SRL-A43", [](DynamicPrintConfig &c) {
             c.set_key_value("hole_to_polyhole", new ConfigOptionBool(true)); }},
        {"separated infill", "separated_infills", "SRL-A44", [](DynamicPrintConfig &c) {
             c.set_key_value("separated_infills", new ConfigOptionBool(true)); }},
        {"bad manual map", "filament_nozzle_map", "SRL-A39", [](DynamicPrintConfig &c) {
             c.set_key_value("filament_map_mode", new ConfigOptionEnum<FilamentMapMode>(fmmNozzleManual));
             c.set_key_value("filament_nozzle_map", new ConfigOptionInts{0}); }},
    };
    for (const Refusal &row : refusals) {
        DYNAMIC_SECTION(row.name) {
            DynamicPrintConfig config = full_coupon_config();
            row.change(config);
            Model model;
            Print print;
            init_coupon(model, print, config);
            const StringObjectException refusal = print.validate();
            INFO(refusal.string);
            REQUIRE_FALSE(refusal.string.empty());
            CHECK(refusal.opt_key == row.key);
            CHECK(refusal.string.find(row.code) != std::string::npos);
        }
    }
}

// Text sunk into a base: a fine part 1 mm tall sunk 0.2 mm into a coarse base. The smaller part keeps
// the shared space at its own nozzle and layer height, the base loses it, and a warning names both
// parts. The base's top stays at its own height around the text, and the result does not depend on
// which part was added first.
TEST_CASE("Body Split gives the overlap of a part sunk into a bigger one to the smaller part", "[TestRebuild][BodySplit]")
{
    DynamicPrintConfig config = coupon_config_with_filaments(2);
    config.set_key_value("nozzle_diameter", new ConfigOptionFloats{0.2, 0.6});
    config.set_key_value("max_layer_height", new ConfigOptionFloats{0.20, 0.42});
    config.set_key_value("mixed_nozzle_allowed_cadence_ratios", new ConfigOptionInts{3});
    fill_per_filament_values(config);
    constexpr double base_top = 3.;
    constexpr double sink = 0.2;
    for (const bool text_first : {false, true}) {
        CAPTURE(text_first);
        Model model;
        Print print;
        ModelObject *object = model.add_object();
        object->name = "sunk-text";
        const auto add_base = [&] {
            ModelVolume *base = object->add_volume(make_cube(20., 20., base_top), ModelVolumeType::MODEL_PART, false);
            base->name = "base";
            base->config.set_key_value("extruder", new ConfigOptionInt(2));
            base->config.set_key_value("regional_layer_height", new ConfigOptionFloat(0.30));
        };
        const auto add_text = [&] {
            ModelVolume *text = object->add_volume(make_cube(8., 8., 1.), ModelVolumeType::MODEL_PART, false);
            text->name = "text";
            text->set_offset(Vec3d(4., 4., base_top - sink));
            text->config.set_key_value("extruder", new ConfigOptionInt(1));
            text->config.set_key_value("regional_layer_height", new ConfigOptionFloat(0.10));
        };
        if (text_first) { add_text(); add_base(); } else { add_base(); add_text(); }
        object->add_instance();
        object->ensure_on_bed();
        print.apply(model, config);
        print.set_status_silent();
        const CadenceTest::Facts facts = CadenceTest::slice(print);
        INFO(facts.refusal.string);
        REQUIRE(facts.refusal.string.empty());
        REQUIRE_FALSE(facts.gcode.empty());

        const PrintStateBase::StateWithWarnings state = print.objects().front()->step_state_with_warnings(posSlice);
        const std::string expected =
            "Part \"text\" overlaps part \"base\" by 0.2 mm; \"text\" was kept there, at its own nozzle and layer height.";
        CHECK(std::any_of(state.warnings.begin(), state.warnings.end(), [&expected](const PrintStateBase::Warning &warning) {
            return warning.message.find(expected) != std::string::npos;
        }));

        const Vec2d origin = model.objects.front()->instance_bounding_box(0).min.head<2>();
        double coarse_top = 0., coarse_top_under = 0., fine_bottom_over = 1e9, fine_top = 0.;
        size_t coarse_in_text = 0;
        Vec3f previous = Vec3f::Zero();
        for (const Move &move : facts.moves) {
            const Vec3f from = previous;
            previous = move.position;
            if (!CadenceTest::model_road(move))
                continue;
            const double z = double(move.position.z());
            const double x = 0.5 * double(move.position.x() + from.x()) - origin.x();
            const double y = 0.5 * double(move.position.y() + from.y()) - origin.y();
            // Well inside the text's footprint, clear of its walls.
            const bool under_text = x > 5. && x < 11. && y > 5. && y < 11.;
            if (move.physical_tool_id == 1) {
                coarse_top = std::max(coarse_top, z);
                if (under_text) {
                    coarse_top_under = std::max(coarse_top_under, z);
                    if (z > base_top - sink + 1e-3)
                        ++coarse_in_text;
                }
            } else {
                fine_top = std::max(fine_top, z);
                if (under_text)
                    fine_bottom_over = std::min(fine_bottom_over, z);
            }
        }
        CAPTURE(coarse_top, coarse_top_under, fine_bottom_over, fine_top, coarse_in_text);
        // The base keeps its own top around the text, neither raised nor lowered.
        CHECK(std::abs(coarse_top - base_top) < 1e-3);
        // Under the text the base stops where the text starts, and the text prints the overlap at 0.1.
        CHECK(std::abs(coarse_top_under - (base_top - sink)) < 1e-3);
        CHECK(coarse_in_text == 0);
        CHECK(std::abs(fine_bottom_over - (base_top - sink + 0.1)) < 1e-3);
        CHECK(std::abs(fine_top - (base_top - sink + 1.)) < 1e-3);
    }
}

// Two parts that overlap heavily, or two parts of the same size, have no clear smaller part to keep
// the shared space, so they are still refused as overlapping.
TEST_CASE("Body Split still refuses a heavy overlap or an overlap of two parts of the same size", "[TestRebuild][BodySplit]")
{
    DynamicPrintConfig config = coupon_config_with_filaments(2);
    config.set_key_value("nozzle_diameter", new ConfigOptionFloats{0.2, 0.6});
    config.set_key_value("max_layer_height", new ConfigOptionFloats{0.20, 0.42});
    config.set_key_value("mixed_nozzle_allowed_cadence_ratios", new ConfigOptionInts{3});
    fill_per_filament_values(config);
    struct Row { const char *name; Vec3d coarse; Vec3d fine; Vec3d fine_offset; };
    const Row rows[] = {
        // Same size, overlapping by half.
        {"half", Vec3d(10., 10., 3.), Vec3d(10., 10., 3.), Vec3d(5., 0., 0.)},
        // Same size, overlapping by 0.5 mm only: neither part is the smaller one.
        {"same size", Vec3d(10., 10., 3.), Vec3d(10., 10., 3.), Vec3d(9.5, 0., 0.)},
        // A quarter-size part wholly inside the bigger one: a quarter of the bigger part is shared.
        {"heavy", Vec3d(20., 20., 3.), Vec3d(10., 10., 3.), Vec3d(5., 5., 0.)},
    };
    for (const Row &row : rows) {
        CAPTURE(row.name);
        Model model;
        Print print;
        ModelObject *object = model.add_object();
        object->name = row.name;
        ModelVolume *coarse = object->add_volume(make_cube(row.coarse.x(), row.coarse.y(), row.coarse.z()),
                                                 ModelVolumeType::MODEL_PART, false);
        coarse->config.set_key_value("extruder", new ConfigOptionInt(2));
        coarse->config.set_key_value("regional_layer_height", new ConfigOptionFloat(0.30));
        ModelVolume *fine = object->add_volume(make_cube(row.fine.x(), row.fine.y(), row.fine.z()),
                                               ModelVolumeType::MODEL_PART, false);
        fine->set_offset(row.fine_offset);
        fine->config.set_key_value("extruder", new ConfigOptionInt(1));
        fine->config.set_key_value("regional_layer_height", new ConfigOptionFloat(0.10));
        object->add_instance();
        object->ensure_on_bed();
        print.apply(model, config);
        print.set_status_silent();
        REQUIRE(print.validate().string.empty());
        REQUIRE_THROWS_WITH(print.process(), Catch::Matchers::ContainsSubstring("SRL-B01"));
    }
}
