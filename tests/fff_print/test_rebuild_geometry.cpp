#include <catch2/catch_all.hpp>

#include "libslic3r/Model.hpp"
#include "libslic3r/Print.hpp"
#include "rebuild_harness.hpp"
#include "srl_fixtures.hpp"

#include <algorithm>
#include <cmath>
#include <map>
#include <set>
#include <string>
#include <vector>

using namespace Slic3r;
using namespace srl_fixtures;

namespace {

using Move = GCodeProcessorResult::MoveVertex;
using Slic3r::Emitted::EmittedRoleFact;

DynamicPrintConfig geometry_config()
{
    DynamicPrintConfig config = coupon_config(true);
    fill_per_filament_values(config);
    return config;
}

ModelObject *add_body_pair(Model &model, const std::string &name, const std::vector<Vec3d> &copies)
{
    ModelObject *object = model.add_object();
    object->name = name;
    ModelVolume *fine = object->add_volume(make_cube(10., 20., 4.), ModelVolumeType::MODEL_PART, false);
    fine->config.set_key_value("extruder", new ConfigOptionInt(1));
    fine->config.set_key_value("regional_layer_height", new ConfigOptionFloat(0.10));
    ModelVolume *coarse = object->add_volume(make_cube(20., 20., 4.), ModelVolumeType::MODEL_PART, false);
    coarse->set_offset(Vec3d(10., 0., 0.));
    coarse->config.set_key_value("extruder", new ConfigOptionInt(2));
    coarse->config.set_key_value("regional_layer_height", new ConfigOptionFloat(0.20));
    for (const Vec3d &offset : copies)
        object->add_instance()->set_offset(offset);
    object->ensure_on_bed();
    return object;
}

CadenceTest::Facts require_slice(Print &print)
{
    CadenceTest::Facts facts = CadenceTest::slice(print);
    INFO(facts.refusal.string);
    REQUIRE(facts.refusal.string.empty());
    REQUIRE_FALSE(facts.gcode.empty());
    CHECK_FALSE(facts.emitted.plan_malformed);
    return facts;
}

std::set<int> model_levels(const CadenceTest::Facts &facts, unsigned tool)
{
    std::set<int> out;
    for (const EmittedRoleFact &fact : facts.emitted.role_facts)
        if (fact.physical_tool == tool && fact.role != erWipeTower &&
            fact.role != erSupportMaterial && fact.role != erSupportMaterialInterface &&
            fact.role != erSkirt && fact.role != erBrim)
            out.insert(int(std::lround(double(fact.z) * 1000.)));
    return out;
}

bool has_warning(const PrintStateBase::StateWithWarnings &state, PrintStateBase::SlicingNotificationType code)
{
    return std::any_of(state.warnings.begin(), state.warnings.end(), [code](const PrintStateBase::Warning &warning) {
        return warning.message_id == code;
    });
}

} // namespace

TEST_CASE("A scaled multi-object Body Split plate deposits every body and publishes each object's plan",
          "[TestRebuild][BodyGeometry]")
{
    DynamicPrintConfig config = geometry_config();
    Model model;
    Print print;
    ModelObject *multi = add_body_pair(model, "two-copies", {Vec3d(0., 0., 0.), Vec3d(45., 0., 0.)});
    ModelVolume *third = multi->add_volume(make_cube(10., 20., 4.), ModelVolumeType::MODEL_PART, false);
    third->set_offset(Vec3d(31., 0., 0.));
    third->config.set_key_value("extruder", new ConfigOptionInt(2));
    third->config.set_key_value("regional_layer_height", new ConfigOptionFloat(0.20));
    third->config.set_key_value("sparse_infill_density", new ConfigOptionPercent(30.));
    ModelObject *scaled = add_body_pair(model, "scaled-pair", {Vec3d(100., 0., 0.)});
    scaled->instances.front()->set_scaling_factor(Vec3d(1.2, 1., 1.));
    scaled->ensure_on_bed();
    print.apply(model, config);
    print.set_status_silent();
    const CadenceTest::Facts facts = require_slice(print);

    CHECK(facts.emitted.plan_start_markers == 2);
    CHECK(facts.emitted.plan_end_markers == 2);
    CHECK(facts.emitted.plan_grids.size() == 5);
    size_t fine_roads = 0, coarse_roads = 0;
    std::set<int> occupied_columns;
    for (const Move &move : facts.moves) {
        if (!CadenceTest::model_road(move))
            continue;
        if (move.physical_tool_id == 0)
            ++fine_roads;
        else if (move.physical_tool_id == 1)
            ++coarse_roads;
        occupied_columns.insert(int(std::floor(double(move.position.x()) / 10.)));
    }
    CHECK(fine_roads > 0);
    CHECK(coarse_roads > 0);
    CHECK(occupied_columns.size() >= 5);
    CHECK_FALSE(facts.emitted.tool_changes.empty());
}

TEST_CASE("A coarse Body alone on the bed exports its first cell with brim and skirt",
          "[TestRebuild][BodyGeometry][BodyFirstCellExport]")
{
    DynamicPrintConfig config = geometry_config();
    config.set_key_value("layer_height", new ConfigOptionFloat(0.08));
    config.set_key_value("initial_layer_print_height", new ConfigOptionFloat(0.10));
    config.set_key_value("nozzle_diameter", new ConfigOptionFloats{0.2, 0.8});
    config.set_key_value("min_layer_height", new ConfigOptionFloats{0.04, 0.16});
    config.set_key_value("max_layer_height", new ConfigOptionFloats{0.14, 0.56});
    config.set_key_value("mixed_nozzle_allowed_cadence_ratios", new ConfigOptionInts{5});
    config.set_key_value("brim_type", new ConfigOptionEnum<BrimType>(btOuterOnly));
    config.set_key_value("brim_width", new ConfigOptionFloat(3.));
    config.set_key_value("skirt_loops", new ConfigOptionInt(1));
    config.set_key_value("skirt_height", new ConfigOptionInt(1));
    config.set_key_value("skirt_distance", new ConfigOptionFloat(4.));
    fill_per_filament_values(config);

    Model model;
    Print print;
    ModelObject *object = model.add_object();
    ModelVolume *coarse = object->add_volume(make_cube(20., 20., 0.26), ModelVolumeType::MODEL_PART, false);
    coarse->config.set_key_value("extruder", new ConfigOptionInt(2));
    coarse->config.set_key_value("regional_layer_height", new ConfigOptionFloat(0.40));
    ModelVolume *fine = object->add_volume(make_cube(20., 20., 0.8), ModelVolumeType::MODEL_PART, false);
    fine->set_offset(Vec3d(0., 0., 0.26));
    fine->config.set_key_value("extruder", new ConfigOptionInt(1));
    fine->config.set_key_value("regional_layer_height", new ConfigOptionFloat(0.08));
    object->add_instance();
    object->ensure_on_bed();
    print.apply(model, config);
    print.set_status_silent();
    const CadenceTest::Facts facts = require_slice(print);
    CHECK_FALSE(has_warning(print.step_state_with_warnings(psGCodeExport),
                            PrintStateBase::SlicingEmptyGcodeLayers));
    CHECK_FALSE(has_warning(print.objects().front()->step_state_with_warnings(posSupportMaterial),
                            PrintStateBase::SlicingNeedSupportOn));

    double first_model_z = 1e9;
    size_t first_model = 0, brim = 0, skirt = 0, fine_above = 0;
    for (const Move &move : facts.moves)
        if (CadenceTest::model_road(move) && move.physical_tool_id == 1)
            first_model_z = std::min(first_model_z, double(move.position.z()));
    REQUIRE(first_model_z < 1e8);
    for (const Move &move : facts.moves) {
        if (move.type != EMoveType::Extrude)
            continue;
        const double z = double(move.position.z());
        if (CadenceTest::model_road(move) && move.physical_tool_id == 1 &&
            std::abs(z - first_model_z) < 0.002)
            ++first_model;
        if (move.extrusion_role == erBrim || move.extrusion_role == erSkirt) {
            if (std::abs(z - first_model_z) < 0.002 && move.physical_tool_id == 1)
                (move.extrusion_role == erBrim ? brim : skirt)++;
        }
        if (CadenceTest::model_road(move) && move.physical_tool_id == 0 && z > first_model_z + 0.02)
            ++fine_above;
    }
    CAPTURE(first_model_z, first_model, brim, skirt, fine_above);
    CHECK(first_model_z > 0.16);
    CHECK(first_model_z > 0.10);
    CHECK(first_model_z <= 0.56);
    CHECK(first_model > 0);
    CHECK(brim > 0);
    CHECK(skirt > 0);
    CHECK(fine_above > 0);
}

TEST_CASE("A Body Split pocket and density modifier change emitted geometry without moving the other body",
          "[TestRebuild][BodyGeometry]")
{
    const bool cross_tool = GENERATE(false, true);
    DynamicPrintConfig config = geometry_config();
    Model model;
    Print print;
    ModelObject *object = add_body_pair(model, "pocket-and-modifier", {Vec3d(0., 0., 0.)});
    ModelVolume *pocket = object->add_volume(make_cube(4., 4., 2.), ModelVolumeType::NEGATIVE_VOLUME, false);
    pocket->set_offset(Vec3d(3., 8., 1.));
    ModelVolume *modifier = object->add_volume(make_cube(8., 20., 4.), ModelVolumeType::PARAMETER_MODIFIER, false);
    modifier->set_offset(Vec3d(20., 0., 0.));
    modifier->config.set_key_value("sparse_infill_density", new ConfigOptionPercent(30.));
    if (cross_tool)
        modifier->config.set_key_value("extruder", new ConfigOptionInt(1));
    print.apply(model, config);
    print.set_status_silent();
    const CadenceTest::Facts facts = require_slice(print);
    const double left = model.objects.front()->instance_bounding_box(0).min.x();

    size_t pocket_wall = 0, pocket_void = 0, coarse_region = 0, modifier_region = 0;
    for (const Move &move : facts.moves) {
        if (!CadenceTest::model_road(move))
            continue;
        const double x = double(move.position.x()) - left;
        const double y = double(move.position.y()) - model.objects.front()->instance_bounding_box(0).min.y();
        const double z = double(move.position.z());
        const bool pocket_xy = x > 3.4 && x < 6.6 && y > 8.4 && y < 11.6;
        const bool around_pocket = x > 2.5 && x < 7.5 && y > 7.5 && y < 12.5;
        pocket_wall += around_pocket && !pocket_xy && z > 1.2 && z < 2.8 ? 1 : 0;
        pocket_void += pocket_xy && z > 1.2 && z < 2.8 ? 1 : 0;
        coarse_region += x > 11. && x < 19. && move.physical_tool_id == 1 ? 1 : 0;
        modifier_region += x > 21. && x < 28. && move.physical_tool_id == (cross_tool ? 0 : 1) ? 1 : 0;
    }
    CAPTURE(cross_tool, pocket_wall, pocket_void, coarse_region, modifier_region);
    CHECK(pocket_wall > 0);
    CHECK(pocket_void == 0);
    CHECK(coarse_region > 0);
    CHECK(modifier_region > 0);
    CHECK(facts.emitted.plan_grids.size() >= 3);
}

TEST_CASE("Tapered and curved coarse bodies retain their cadence as their emitted footprint narrows",
          "[TestRebuild][BodyGeometry]")
{
    const bool curved = GENERATE(false, true);
    TriangleMesh coarse;
    if (curved) {
        std::vector<Vec2d> outline{{0., 0.}, {20., 0.}};
        for (int i = 1; i <= 16; ++i) {
            const double z = 4. * double(i) / 16.;
            outline.emplace_back(20. - 4. * std::sin(0.5 * M_PI * z / 4.), z);
        }
        outline.emplace_back(0., 4.);
        coarse = extruded_xz_outline(outline, 20.);
    } else
        coarse = extruded_xz_outline({{0., 0.}, {20., 0.}, {16., 4.}, {0., 4.}}, 20.);
    DynamicPrintConfig config = geometry_config();
    Model model;
    Print print;
    init_coupon(model, print, config, coarse);
    declare_native_cadences(model, print, config);
    const CadenceTest::Facts facts = require_slice(print);
    const double left = model.objects.front()->instance_bounding_box(0).min.x();

    double low_edge = -1e9, high_edge = -1e9;
    size_t coarse_roads = 0, wrong_height = 0;
    for (const EmittedRoleFact &fact : facts.emitted.role_facts) {
        if (fact.physical_tool != 1 || fact.role == erWipeTower || fact.role == erBrim || fact.role == erSkirt)
            continue;
        coarse_roads += fact.move_count;
        if (fact.z > 0.5 && fact.z < 1.)
            low_edge = std::max(low_edge, double(fact.max_x) - left);
        if (fact.z > 3. && fact.z < 3.7)
            high_edge = std::max(high_edge, double(fact.max_x) - left);
        if (fact.z > 0.11 && std::abs(fact.height - 0.20f) > 1e-3f)
            wrong_height += fact.move_count;
    }
    CAPTURE(curved, coarse_roads, low_edge, high_edge, wrong_height);
    CHECK(coarse_roads > 0);
    CHECK(low_edge > 0.);
    CHECK(high_edge > 0.);
    CHECK(low_edge > high_edge + 1.5);
    CHECK(wrong_height == 0);
    CHECK(facts.emitted.plan_grids.size() == 2);
}

TEST_CASE("A sloped coarse Body wall contours at its own cell plane in emitted G-code",
          "[TestRebuild][BodyGeometry][BodyZContour]")
{
    struct WallSample { size_t roads{0}; double min_z{1e9}; double min_x{1e9}; double max_x{-1e9}; };
    const auto sample = [](bool contouring) {
        DynamicPrintConfig config = geometry_config();
        config.set_key_value("zaa_enabled", new ConfigOptionBool(contouring));
        config.set_key_value("zaa_min_z", new ConfigOptionFloat(0.03));
        config.set_key_value("zaa_minimize_perimeter_height", new ConfigOptionFloat(0.));
        config.set_key_value("machine_start_gcode", new ConfigOptionString("G28\nM1020 S[initial_extruder]\n"));
        Model model;
        Print print;
        print.is_BBL_printer() = true;
        const TriangleMesh ramp = extruded_xz_outline(
            {{0., 0.}, {10., 0.}, {10., 2.}, {0., 3.}}, 20.);
        init_native_coupon(model, print, config, ramp, ramp);
        const CadenceTest::Facts facts = require_slice(print);
        const double left = model.objects.front()->instance_bounding_box(0).min.x();
        WallSample wall;
        // At the 2.3 mm coarse event the cell began at 2.1 mm and was sliced at 2.13 mm.
        // Its sloping right perimeter is around X=28.5 mm. An event-plane ray at 2.23 mm
        // misses that surface, while a cell-plane ray drops the emitted wall toward 2.13 mm.
        for (const Move &move : facts.moves) {
            if (move.type != EMoveType::Extrude || move.extrusion_role != erExternalPerimeter ||
                move.physical_tool_id != 1 || std::abs(double(move.print_z) - 2.3) > 0.025)
                continue;
            const double x = double(move.position.x()) - left;
            if (x < 27.0 || x > 29.5)
                continue;
            ++wall.roads;
            wall.min_z = std::min(wall.min_z, double(move.position.z()));
            wall.min_x = std::min(wall.min_x, x);
            wall.max_x = std::max(wall.max_x, x);
        }
        return wall;
    };
    const WallSample plain = sample(false);
    const WallSample contoured = sample(true);
    CAPTURE(plain.roads, contoured.roads, plain.min_z, contoured.min_z,
            plain.min_x, plain.max_x, contoured.min_x, contoured.max_x);
    REQUIRE(plain.roads > 0);
    REQUIRE(contoured.roads > 0);
    CHECK(plain.min_x > 27.0);
    CHECK(contoured.min_x > 27.0);
    CHECK(std::abs(plain.min_z - 2.3) < 0.02);
    CHECK(contoured.min_z < plain.min_z - 0.10);
    CHECK(contoured.min_z > 2.09);
    // A wall sampled at the lower cell plane intersects the sloped top
    // farther out than one sampled at the event plane. The contour should
    // therefore move right as well as down. The ramp's right edge runs from
    // X=30 at Z=2 to X=20 at Z=3 in this placed coupon.
    CHECK(contoured.max_x > plain.max_x + 0.4);
    CHECK_THAT(contoured.max_x,
               Catch::Matchers::WithinAbs(20. + 10. * (3. - contoured.min_z), .20));
}

TEST_CASE("Painted coarse bodies keep fine skin roads on the fine tool across multiple regions",
          "[TestRebuild][BodyGeometry]")
{
    DynamicPrintConfig config = coupon_config_with_filaments(3);
    config.set_key_value("filament_map", new ConfigOptionInts{1, 1, 2});
    config.set_key_value("nozzle_diameter", new ConfigOptionFloats{0.2, 0.4});
    fill_per_filament_values(config);
    Model model;
    Print print;
    ModelObject *object = model.add_object();
    ModelVolume *fine = object->add_volume(make_cube(10., 20., 4.), ModelVolumeType::MODEL_PART, false);
    fine->config.set_key_value("extruder", new ConfigOptionInt(1));
    fine->config.set_key_value("regional_layer_height", new ConfigOptionFloat(0.10));
    ModelVolume *coarse = object->add_volume(make_cube(10., 20., 4.), ModelVolumeType::MODEL_PART, false);
    coarse->set_offset(Vec3d(10., 0., 0.));
    coarse->config.set_key_value("extruder", new ConfigOptionInt(3));
    coarse->config.set_key_value("regional_layer_height", new ConfigOptionFloat(0.20));
    coarse->config.set_key_value("mixed_nozzle_body_fine_skins", new ConfigOptionBool(true));
    paint_cadence_facets(*coarse, {4, 5}, EnforcerBlockerType::Extruder2);
    object->add_instance();
    object->ensure_on_bed();
    print.apply(model, config);
    print.set_status_silent();
    const CadenceTest::Facts facts = require_slice(print);
    const double left = object->instance_bounding_box(0).min.x();

    size_t painted = 0, coarse_middle = 0, fine_roof = 0, wrong_roof = 0;
    for (const EmittedRoleFact &fact : facts.emitted.role_facts) {
        if (fact.role == erWipeTower || fact.role == erBrim || fact.role == erSkirt)
            continue;
        if (fact.logical_filament == 1 && fact.max_x > left + 10.) {
            painted += fact.move_count;
            CHECK(fact.physical_tool == 0);
        }
        if (fact.physical_tool == 1 && fact.min_x > left + 10. && fact.z > 1. && fact.z < 2.5)
            coarse_middle += fact.move_count;
        if (fact.physical_tool == 0 && fact.max_x > left + 11. && fact.z > 3.7) {
            fine_roof += fact.move_count;
            wrong_roof += std::abs(fact.height - 0.10f) > 1e-3f ? fact.move_count : 0;
        }
    }
    CAPTURE(painted, coarse_middle, fine_roof, wrong_roof);
    CHECK(painted > 0);
    CHECK(coarse_middle > 0);
    CHECK(fine_roof > 0);
    CHECK(wrong_roof == 0);
    CHECK(facts.emitted.plan_grids.size() >= 3);
}

TEST_CASE("Fine skins preserve emitted beams across a touching Body Split joint", "[TestRebuild][BodyGeometry]")
{
    DynamicPrintConfig config = fine_skin_ratio3_config();
    config.set_key_value("interlocking_beam", new ConfigOptionBool(true));
    config.set_key_value("interlocking_beam_layer_count", new ConfigOptionInt(3));
    config.set_key_value("interlocking_beam_width", new ConfigOptionFloat(0.88));
    config.set_key_value("interlocking_depth", new ConfigOptionInt(2));
    config.set_key_value("interlocking_boundary_avoidance", new ConfigOptionInt(2));
    config.set_key_value("interlocking_orientation", new ConfigOptionFloat(22.5));
    fill_per_filament_values(config);
    struct Outcome { size_t coarse_into_fine{0}; size_t fine_roof{0}; size_t coarse_middle{0}; };
    const auto slice_with_skins = [&](bool skins) {
        Model model;
        Print print;
        ModelObject *object = model.add_object();
        ModelVolume *fine = object->add_volume(make_cube(10., 20., 3.), ModelVolumeType::MODEL_PART, false);
        fine->config.set_key_value("extruder", new ConfigOptionInt(1));
        fine->config.set_key_value("regional_layer_height", new ConfigOptionFloat(0.10));
        ModelVolume *coarse = object->add_volume(
            box_with_internal_cavity(Vec3d(20., 20., 3.), Vec3d(8.5, 8.5, 1.7), Vec3d(3., 3., 1.)),
            ModelVolumeType::MODEL_PART, false);
        coarse->set_offset(Vec3d(10., 0., 0.));
        coarse->config.set_key_value("extruder", new ConfigOptionInt(2));
        coarse->config.set_key_value("regional_layer_height", new ConfigOptionFloat(0.30));
        coarse->config.set_key_value("mixed_nozzle_body_fine_skins", new ConfigOptionBool(skins));
        object->add_instance();
        object->ensure_on_bed();
        print.apply(model, config);
        print.set_status_silent();
        const CadenceTest::Facts facts = require_slice(print);
        const double left = object->instance_bounding_box(0).min.x();
        Outcome out;
        for (const Move &move : facts.moves) {
            if (!CadenceTest::model_road(move))
                continue;
            const double x = double(move.position.x()) - left;
            const double z = double(move.position.z());
            out.coarse_into_fine += move.physical_tool_id == 1 && x < 9.7 && z > 0.3 && z < 2.4 ? 1 : 0;
            out.fine_roof += move.physical_tool_id == 0 && x > 15. && z > 2.7 ? 1 : 0;
            out.coarse_middle += move.physical_tool_id == 1 && x > 11. && z > 0.5 && z < 1.2 ? 1 : 0;
        }
        return out;
    };
    const Outcome plain = slice_with_skins(false);
    const Outcome skinned = slice_with_skins(true);
    CAPTURE(plain.coarse_into_fine, skinned.coarse_into_fine, plain.fine_roof, skinned.fine_roof,
            skinned.coarse_middle);
    CHECK(plain.coarse_into_fine > 0);
    CHECK(skinned.coarse_into_fine > 0);
    CHECK(plain.fine_roof == 0);
    CHECK(skinned.fine_roof > 0);
    CHECK(skinned.coarse_middle > 0);
}

TEST_CASE("A raised Body Split joint deposits both beam directions without an empty-layer warning",
          "[TestRebuild][BodyGeometry]")
{
    const int ratio = GENERATE(2, 3);
    const double coarse_cadence = 0.08 * ratio;
    Model model;
    Print print;
    ModelObject *object = model.add_object();
    ModelVolume *coarse = object->add_volume(extruded_xz_outline(
        {{0., 0.}, {40., 0.}, {40., 1.30}, {20., 1.30}, {20., 2.18}, {0., 2.18}}, 20.),
        ModelVolumeType::MODEL_PART, false);
    coarse->config.set_key_value("extruder", new ConfigOptionInt(2));
    coarse->config.set_key_value("regional_layer_height", new ConfigOptionFloat(coarse_cadence));
    ModelVolume *fine = object->add_volume(extruded_xz_outline(
        {{20., 1.30}, {40., 1.30}, {40., 4.98}, {20., 4.98}}, 20.),
        ModelVolumeType::MODEL_PART, false);
    fine->config.set_key_value("extruder", new ConfigOptionInt(1));
    fine->config.set_key_value("regional_layer_height", new ConfigOptionFloat(0.08));
    object->config.set_key_value("interlocking_beam", new ConfigOptionBool(true));
    object->config.set_key_value("interlocking_beam_layer_count", new ConfigOptionInt(ratio));
    object->config.set_key_value("interlocking_beam_width", new ConfigOptionFloat(0.8));
    object->config.set_key_value("interlocking_depth", new ConfigOptionInt(2));
    object->config.set_key_value("interlocking_boundary_avoidance", new ConfigOptionInt(2));
    object->config.set_key_value("interlocking_orientation", new ConfigOptionFloat(22.5));
    object->add_instance();
    object->ensure_on_bed();

    DynamicPrintConfig config = geometry_config();
    config.set_key_value("layer_height", new ConfigOptionFloat(0.08));
    config.set_key_value("initial_layer_print_height", new ConfigOptionFloat(0.10));
    config.set_key_value("max_layer_height", new ConfigOptionFloats{0.14, 0.28});
    config.set_key_value("mixed_nozzle_allowed_cadence_ratios", new ConfigOptionInts{2, 3});
    config.set_key_value("support_top_z_distance", new ConfigOptionFloat(0.));
    config.set_key_value("support_bottom_z_distance", new ConfigOptionFloat(0.));
    fill_per_filament_values(config);
    print.apply(model, config);
    print.set_status_silent();
    const CadenceTest::Facts facts = require_slice(print);
    CHECK_FALSE(has_warning(print.step_state_with_warnings(psGCodeExport),
                            PrintStateBase::SlicingEmptyGcodeLayers));
    CHECK_FALSE(has_warning(print.objects().front()->step_state_with_warnings(posSupportMaterial),
                            PrintStateBase::SlicingNeedSupportOn));

    const double left = object->instance_bounding_box(0).min.x();
    size_t fine_into_coarse = 0, coarse_into_fine = 0;
    for (const Move &move : facts.moves) {
        if (!CadenceTest::model_road(move))
            continue;
        const double z = double(move.position.z());
        if (z <= 1.30 || z > 2.18)
            continue;
        const double x = double(move.position.x()) - left;
        fine_into_coarse += move.physical_tool_id == 0 && x < 20. - 0.3 ? 1 : 0;
        coarse_into_fine += move.physical_tool_id == 1 && x > 20. + 0.3 ? 1 : 0;
    }
    CAPTURE(ratio, fine_into_coarse, coarse_into_fine);
    CHECK(fine_into_coarse > 0);
    CHECK(coarse_into_fine > 0);
}

TEST_CASE("Body Split support warnings distinguish a grounded pair from a real shelf", "[TestRebuild][BodyGeometry]")
{
    for (const bool shelf : {false, true}) {
        DYNAMIC_SECTION("shelf " << shelf) {
            Model model;
            Print print;
            ModelObject *object = model.add_object();
            ModelVolume *coarse = object->add_volume(make_cube(20., 20., 4.), ModelVolumeType::MODEL_PART, false);
            coarse->config.set_key_value("extruder", new ConfigOptionInt(2));
            coarse->config.set_key_value("regional_layer_height", new ConfigOptionFloat(0.24));
            ModelVolume *fine = object->add_volume(extruded_xz_outline(shelf ?
                std::vector<Vec2d>{{20., 0.}, {30., 0.}, {30., 2.}, {40., 2.}, {40., 4.}, {20., 4.}} :
                std::vector<Vec2d>{{20., 0.}, {40., 0.}, {40., 4.}, {20., 4.}}, 20.),
                ModelVolumeType::MODEL_PART, false);
            fine->config.set_key_value("extruder", new ConfigOptionInt(1));
            fine->config.set_key_value("regional_layer_height", new ConfigOptionFloat(0.08));
            object->add_instance();
            object->ensure_on_bed();
            DynamicPrintConfig config = geometry_config();
            config.set_key_value("layer_height", new ConfigOptionFloat(0.08));
            config.set_key_value("initial_layer_print_height", new ConfigOptionFloat(0.10));
            config.set_key_value("max_layer_height", new ConfigOptionFloats{0.14, 0.28});
            config.set_key_value("mixed_nozzle_allowed_cadence_ratios", new ConfigOptionInts{2, 3});
            config.set_key_value("enable_support", new ConfigOptionBool(false));
            fill_per_filament_values(config);
            print.apply(model, config);
            print.set_status_silent();
            const CadenceTest::Facts facts = require_slice(print);
            CHECK_FALSE(model_levels(facts, 0).empty());
            CHECK_FALSE(model_levels(facts, 1).empty());
            const bool warned = has_warning(print.objects().front()->step_state_with_warnings(posSupportMaterial),
                                            PrintStateBase::SlicingNeedSupportOn);
            CHECK(warned == shelf);
        }
    }
}
